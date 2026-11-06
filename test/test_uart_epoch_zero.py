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
"""Exercise paused epoch-zero bootstrap with the real plugin and detector."""
import os
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header

ROBOT = ('<robot name="test"><link name="root"/><link name="head_pitch"/>'
         '<link name="camera"/><link name="armor_front"><visual><geometry>'
         '<box size="0.01 0.2 0.1"/></geometry></visual></link>'
         '<joint name="head" type="fixed"><parent link="root"/><child link="head_pitch"/></joint>'
         '<joint name="camera_joint" type="fixed"><parent link="head_pitch"/>'
         '<child link="camera"/></joint><joint name="armor_joint" type="fixed">'
         '<parent link="root"/><child link="armor_front"/></joint></robot>')


def run_case(firmware, mode):
    node = rclpy.create_node('epoch_zero_test')
    clocks, native_clocks, ready, ticks = [], [], [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [node.create_subscription(Clock, '/clock', clocks.append, qos_profile_sensor_data),
            node.create_subscription(Clock, '/test/native_clock', native_clocks.append,
                                     qos_profile_sensor_data),
            node.create_subscription(Header, '/mcb_emulator/epoch_zero_ready', ready.append, qos),
            node.create_subscription(Header, '/mcb_emulator/tick', ticks.append, 100)]
    device = f'/tmp/epoch-zero-{os.getpid()}-{time.monotonic_ns()}'
    log = tempfile.TemporaryFile(mode='w+')
    processes = []

    def start(args):
        processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True))

    def wait(predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        log.flush()
        log.seek(0)
        raise AssertionError(f'{label}: {log.read()}')

    def detector():
        start(['ros2', 'run', 'sim', 'detector_standin_node', '--ros-args',
               '-p', 'use_sim_time:=true', '-p', 'deterministic_epoch_zero:=true',
               '-p', 'robot_description:=' + ROBOT])

    def failed(message):
        log.flush()
        pos = log.tell()
        log.seek(0)
        output = log.read()
        log.seek(pos)
        return message in output

    try:
        if mode in ('early', 'after_ready'):
            detector()
        start(['ros2', 'launch', 'sim', 'sim.launch.py', 'gui:=false', 'rviz:=false',
               'foxglove:=false', 'camera:=false', 'pose_emulator:=false', 'mcb_physics:=true',
               'uart_deterministic:=true', 'uart_epoch_zero:=true', 'mcb_auto_fire:=false',
               'lockstep:=' + ('true' if mode in ('early', 'late') else 'false'),
               'mcb_device_link:=' + device, 'firmware_binary:=' + firmware])
        start(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
               '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '--ros-args',
               '-r', '/clock:=/test/native_clock'])
        wait(lambda: os.path.exists(device), 30, 'PTY')
        start(['ros2', 'run', 'dji_serial_bridge', 'dji_serial_bridge_node', '--ros-args',
               '-p', 'device:=' + device, '-p', 'use_sim_time:=true',
               '-p', 'sim_frame_metadata:=true'])
        if mode not in ('early', 'after_ready'):
            wait(lambda: len(clocks) >= 2, 20, 'repeated zero clock')
            assert not ready
            if mode == 'late':
                detector()
            elif mode == 'nonzero':
                pub = node.create_publisher(Clock, '/clock', 10)
                wait(lambda: pub.get_subscription_count() > 0, 5, 'clock subscriber')
                clock = Clock()
                clock.clock.nanosec = 1
                pub.publish(clock)
                wait(lambda: failed('UART epoch zero missed'), 5, 'nonzero clock rejection')
                assert not ready and not ticks
                print('epoch zero nonzero: PASS', flush=True)
                return
            else:
                wait(lambda: failed('UART epoch zero acknowledgment timeout'), 35, 'timeout')
                assert not ready and not ticks
                print('epoch zero timeout: PASS', flush=True)
                return
        wait(lambda: bool(ready), 20, 'bootstrap readiness')
        wait(lambda: bool(native_clocks), 5, 'native Gazebo clock')
        assert clocks and all(c.clock.sec == 0 and c.clock.nanosec == 0 for c in clocks)
        assert all(c.clock.sec == 0 and c.clock.nanosec == 0 for c in native_clocks)
        assert ready[-1].stamp.sec == 0 and ready[-1].stamp.nanosec == 0
        assert not ticks
        if mode == 'after_ready':
            pub = node.create_publisher(Clock, '/clock', 10)
            wait(lambda: pub.get_subscription_count() > 0, 5, 'clock subscriber')
            clock = Clock()
            clock.clock.nanosec = 1
            pub.publish(clock)
            wait(lambda: failed('UART epoch zero missed'), 5, 'clock after readiness')
            assert not ticks
        print(f'epoch zero {mode}: PASS clocks={len(clocks)}', flush=True)
    finally:
        for process in reversed(processes):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for process in reversed(processes):
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        log.close()
        del subs


def main():
    if len(sys.argv) != 2 or not os.access(sys.argv[1], os.X_OK):
        raise SystemExit('pass explicit hosted firmware executable')
    # Loopback-only UDP: no Tailscale peers, and no SHM (LOCALHOST would add it).
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = os.path.join(
        get_package_share_directory('thornbots_pkg'), 'config', 'fastdds_udp_only_local.xml')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'SYSTEM_DEFAULT'
    rclpy.init()
    try:
        for mode in ('early', 'late', 'nonzero', 'after_ready', 'timeout'):
            run_case(sys.argv[1], mode)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
