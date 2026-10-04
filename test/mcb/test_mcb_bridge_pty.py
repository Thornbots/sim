# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The MCB emulator against the real dji_serial_bridge over a pty, no gz.

The firmware runs on IdealHardware at wall-clock 1 kHz in a thread; the
bridge is its own process on a private ROS domain. Shows what crosses the
wire: POSE and REF_SYS arrive, NAV_GOAL drives, RELOCALIZE moves odometry,
and CV_TARGET aims and fires, with or without firmware_fixes.
"""
import math
import os
import shutil
import signal
import subprocess
import threading
import time

from dji_serial_bridge.msg import CVTarget, RefSysStatus, RobotPose
from geometry_msgs.msg import PointStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sim.mcb_emulator.protocol import CV_TARGET, NAV_GOAL, RELOCALIZE
from sim.mcb_emulator.pty_link import PtyLink
from sim.mcb_emulator.sentry import DRIVE_AUTO, IdealHardware, Sentry
from sim.mcb_emulator.subsystems import RefSerial

DOMAIN_ID = 87  # away from any live stack
pytestmark = pytest.mark.skipif(shutil.which('ros2') is None, reason='needs a ROS environment')


class WallClockMcb:
    """Steps the firmware in real time against the pty until stopped."""

    def __init__(self, link, drive, firmware_fixes=True):
        self.hw = IdealHardware()
        self.sentry = Sentry(self.hw, RefSerial(robot_id=107), drive=drive,
                             firmware_fixes=firmware_fixes)
        self.pty = PtyLink(link)
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def _loop(self):
        t0, done = time.monotonic(), 0
        while not self._stop.is_set():
            due = int((time.monotonic() - t0) * 1000) - done
            if due > 0:
                with self.lock:
                    self.sentry.receive(self.pty.read(), due)
                    for _ in range(due):
                        self.sentry.step()
                        self.hw.advance(0.001)
                    out = self.sentry.drain_tx()
                if out:
                    self.pty.write(out)
                done += due
            time.sleep(0.001)

    def start(self):
        self.thread.start()

    def stop(self):
        self._stop.set()
        self.thread.join()
        self.pty.close()


@pytest.fixture
def link(tmp_path):
    return str(tmp_path / 'mcb_pty')


@pytest.fixture
def ros():
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=DOMAIN_ID)
    node = rclpy.create_node('mcb_bridge_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        yield node, executor
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def _bridge(link):
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN_ID))
    return subprocess.Popen(
        ['ros2', 'run', 'dji_serial_bridge', 'dji_serial_bridge_node', '--ros-args',
         '-p', f'device:={link}', '-p', 'debug_log:=false', '-p', 'read_poll_ms:=2'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def _spin(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


def _wait_for_pose(executor, poses, timeout=15.0):
    end = time.monotonic() + timeout
    while not poses and time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)
    assert poses, 'no /dji_serial_bridge/pose: the bridge never decoded a POSE'


@pytest.fixture
def stack(request, link, ros):
    drive, fixes = getattr(request, 'param', ('stop', True))
    mcb = WallClockMcb(link, drive, fixes)
    mcb.start()
    bridge = _bridge(link)
    try:
        yield mcb, ros
    finally:
        # Signal the group: `ros2 run` alone leaves the node orphaned on macOS.
        os.killpg(bridge.pid, signal.SIGINT)
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(bridge.pid, signal.SIGKILL)
        mcb.stop()


def _subscribe(node, msg_type, topic):
    got = []
    node.create_subscription(msg_type, topic, got.append, qos_profile_sensor_data)
    return got


def test_pose_and_ref_sys_reach_ros(stack):
    mcb, (node, executor) = stack
    with mcb.lock:
        mcb.sentry.auto_fire_enabled = False  # no patrol: the turret stays where it's put
        mcb.hw.x, mcb.hw.y, mcb.hw.yaw = 1.25, -0.75, 0.3
    poses = _subscribe(node, RobotPose, '/dji_serial_bridge/pose')
    refs = _subscribe(node, RefSysStatus, '/dji_serial_bridge/ref_sys')
    _wait_for_pose(executor, poses)
    poses.clear()
    refs.clear()
    _spin(executor, 2.0)
    assert 150 <= len(poses) <= 200, f'{len(poses)} poses in 2 s; the firmware sends 90 Hz'
    assert 16 <= len(refs) <= 22, f'{len(refs)} ref_sys in 2 s; the firmware sends 10 Hz'
    p = poses[-1]
    # Odometry's x right, y forward on the wire as REP-105 (firmware_fixes).
    assert (p.x, p.y) == (pytest.approx(-0.75), pytest.approx(-1.25))
    assert p.head_yaw == pytest.approx(0.3, abs=0.01)
    assert p.odom_status == RobotPose.ODOM_PODS
    r = refs[-1]
    assert r.is_on_blue_team and r.robot_id == 7 and r.robot_hp == 400
    assert r.game_stage == 4 and r.chassis_has_power and r.gimbal_has_power
    assert r.delta_angle_got_hit_in == pytest.approx(123.0)


def _send_cv_and_relocalize(node, executor):
    """2 s of fire frames 2 m ahead at 30 Hz, a RELOCALIZE to (1, 2) every 0.5 s."""
    cv_pub = node.create_publisher(CVTarget, '/dji_serial_bridge/cv_target',
                                   qos_profile_sensor_data)
    reloc_pub = node.create_publisher(PointStamped, '/dji_serial_bridge/relocalize', 10)
    target = CVTarget(x=2.0, y=0.0, z=0.3, fire=True, delay_ms=10)
    reloc = PointStamped()
    reloc.point.x, reloc.point.y = 1.0, 2.0
    for i in range(60):
        cv_pub.publish(target)
        if i % 15 == 0:
            reloc_pub.publish(reloc)
        _spin(executor, 1.0 / 30)
    _spin(executor, 0.2)


def test_cv_target_aims_and_fires_and_relocalize_moves_odometry(stack):
    mcb, (node, executor) = stack
    poses = _subscribe(node, RobotPose, '/dji_serial_bridge/pose')
    _wait_for_pose(executor, poses)
    _send_cv_and_relocalize(node, executor)
    with mcb.lock:
        uart = mcb.sentry.drivers.uart
        cv_used, reloc_used = uart.consumed[CV_TARGET], uart.consumed[RELOCALIZE]
        shots = list(mcb.hw.shots)
        odo = (mcb.sentry.odo.get_x(), mcb.sentry.odo.get_y())
    assert cv_used >= 50, f'only {cv_used} CV_TARGET frames read'
    assert reloc_used >= 3
    assert odo == (pytest.approx(-2.0), pytest.approx(1.0))  # (1, 2) in odometry's axes
    assert len(shots) >= 15, f'{len(shots)} shots in 2 s of 30 Hz fire frames'


@pytest.mark.parametrize('stack', [('stop', False)], indirect=True)
def test_without_firmware_fixes_cv_target_fits(stack):
    """The bridge's 15-byte CV_TARGET is position-based-cv's CvTarget as it is."""
    mcb, (node, executor) = stack
    poses = _subscribe(node, RobotPose, '/dji_serial_bridge/pose')
    _wait_for_pose(executor, poses)
    _send_cv_and_relocalize(node, executor)
    with mcb.lock:
        uart = mcb.sentry.drivers.uart
        cv_in = uart.received[(CV_TARGET, 15)]
        refused = sum(uart.size_mismatch.values())
        shots = list(mcb.hw.shots)
    assert cv_in >= 50, f'only {cv_in} 15-byte CV_TARGET frames arrived'
    assert refused == 0 and uart.consumed[CV_TARGET] > 0
    assert shots


@pytest.mark.parametrize('stack', [(DRIVE_AUTO, True)], indirect=True)
def test_nav_goal_drives_auto_drive_command(stack):
    """NAV_GOAL reaches AutoDriveCommand, which the sentry never schedules."""
    mcb, (node, executor) = stack
    poses = _subscribe(node, RobotPose, '/dji_serial_bridge/pose')
    goal_pub = node.create_publisher(PointStamped, '/dji_serial_bridge/nav_goal', 10)
    _wait_for_pose(executor, poses)
    goal = PointStamped()
    goal.point.x, goal.point.y = 0.8, 0.4
    goal_pub.publish(goal)
    _spin(executor, 3.0)
    with mcb.lock:
        assert mcb.sentry.drivers.uart.consumed[NAV_GOAL] == 1
        assert math.hypot(mcb.hw.x + 0.4, mcb.hw.y - 0.8) < 0.05  # x right, y forward
    assert math.hypot(poses[-1].x - 0.8, poses[-1].y - 0.4) < 0.05
