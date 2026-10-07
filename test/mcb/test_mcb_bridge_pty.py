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

"""The compiled MCB against the real ROS serial bridge, using fake hardware."""
import os
import shutil
import signal
import subprocess
import threading
import time
from types import SimpleNamespace

from dji_serial_bridge.msg import CVTarget, RefSysStatus, RobotPose
from geometry_msgs.msg import PointStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sim.mcb_emulator.pty_link import PtyLink
from sim.mcb_firmware import default_binary, Firmware

pytestmark = pytest.mark.skipif(
    not os.access(default_binary(), os.X_OK) or shutil.which('ros2') is None,
    reason='needs ROS and the compiled MCB firmware')


def test_native_mcb_through_real_ros_bridge(tmp_path):
    link = PtyLink(str(tmp_path / 'mcb'))
    firmware = Firmware(link.master)
    ref = SimpleNamespace(game_type=4, game_stage=4, stage_time_remaining=300,
                          robot_id=107, current_hp=400, max_hp=400, shooter_power=True,
                          restoration_zone=False, exchange_zone=False, central_buff_zone=False)
    stop = threading.Event()
    shots, errors = [], []

    def tick():
        try:
            while not stop.wait(0.005):
                shots.extend(firmware.step(5, [0.0] * 10, ref)[3])
        except BaseException as error:
            errors.append(error)

    context = rclpy.Context()
    rclpy.init(context=context, domain_id=87)
    node = rclpy.create_node('native_mcb_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    poses, refs = [], []
    node.create_subscription(RobotPose, '/dji_serial_bridge/pose', poses.append,
                             qos_profile_sensor_data)
    node.create_subscription(RefSysStatus, '/dji_serial_bridge/ref_sys', refs.append,
                             qos_profile_sensor_data)
    cv = node.create_publisher(CVTarget, '/dji_serial_bridge/cv_target', qos_profile_sensor_data)
    reloc = node.create_publisher(PointStamped, '/dji_serial_bridge/relocalize', 10)
    log = (tmp_path / 'bridge.log').open('w')
    bridge = subprocess.Popen(
        ['ros2', 'run', 'dji_serial_bridge', 'dji_serial_bridge_node', '--ros-args',
         '-p', f'device:={link.link}', '-p', 'debug_log:=false', '-p', 'read_poll_ms:=2'],
        env=dict(os.environ, ROS_DOMAIN_ID='87'), stdout=log, stderr=log,
        start_new_session=True)
    thread = threading.Thread(target=tick, daemon=True)
    thread.start()

    def wait_for(predicate, publish=None):
        deadline = time.monotonic() + 15
        while not predicate() and time.monotonic() < deadline:
            assert not errors, str(errors)
            assert bridge.poll() is None, (tmp_path / 'bridge.log').read_text()
            if publish:
                publish()
            executor.spin_once(timeout_sec=0.03)
        assert predicate(), (tmp_path / 'bridge.log').read_text()

    try:
        wait_for(lambda: poses and refs)
        assert poses[-1].x == pytest.approx(firmware.start_x)
        assert refs[-1].is_on_blue_team and refs[-1].robot_id == 7
        target = CVTarget(x=firmware.start_x - 3, y=1.0, z=0.2, fire=True, delay_ms=100)
        # Send below the 80 ms firing latency's reset rate so each timer can expire.
        next_cv = [0.0]

        def send_cv():
            if time.monotonic() >= next_cv[0]:
                cv.publish(target)
                next_cv[0] = time.monotonic() + 0.1

        wait_for(lambda: len(shots) >= 3, send_cv)
        point = PointStamped()
        point.point.x, point.point.y = 1.25, -0.75
        wait_for(lambda: poses[-1].x == pytest.approx(1.25)
                 and poses[-1].y == pytest.approx(-0.75), lambda: reloc.publish(point))
    finally:
        stop.set()
        thread.join(timeout=6)
        os.killpg(bridge.pid, signal.SIGINT)
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(bridge.pid, signal.SIGKILL)
            bridge.wait()
        firmware.close()
        link.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        log.close()
