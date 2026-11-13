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
"""Observe the real E2E launch per stage; pass the hosted firmware ELF as argv[1]."""
import os
import signal
import subprocess
import sys
import tempfile
import time

from dji_serial_bridge.msg import PanelDetectionArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def launch_environment():
    env = os.environ.copy()
    env['GZ_PARTITION'] = f'e2e-epoch-{os.getpid()}-{time.monotonic_ns()}'
    return env


def partition_pids(partition):
    marker = f'GZ_PARTITION={partition}'.encode()
    pids = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/environ', 'rb') as environ:
                if marker in environ.read().split(b'\0'):
                    pids.append(int(pid))
        except OSError:
            pass
    return pids


def stop(proc, partition):
    """
    Stop the launch and every process it left, even outside its group.

    A gz server outliving one launch latched stale MCB readiness for the next
    (same ROS domain), releasing lockstep before the new plugin booted.
    """
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    deadline = time.monotonic() + 15
    while (pids := partition_pids(partition)) and time.monotonic() < deadline:
        time.sleep(0.1)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    while partition_pids(partition):
        time.sleep(0.1)


def run(firmware, delay, stage):
    node = rclpy.create_node('e2e_epoch_observer')
    ready, clock, detection, aim, tick, path, frame_ack = {}, [], [], [], [], [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topics = ['/detector_standin/ready', '/point_to_cv_target/epoch_zero_ready',
              '/opponent_driver/ready', '/mcb_emulator/epoch_zero_ready',
              '/robot_spawn_ready/ready', '/target_manager/epoch_zero_ready']
    topics += ['/target_driver/ready' if stage == 'mcb_parked' else '/match_environment/ready']
    if stage == 'mcb_match':
        topics += ['/opponent_driver_opponent_1/ready', '/opponent_driver_ally_0/ready']
    subs = [node.create_subscription(Header, topic,
            lambda msg, topic=topic: ready.setdefault(topic, ns(msg.stamp)), qos)
            for topic in topics]
    subs += [node.create_subscription(Clock, '/clock', clock.append, qos_profile_sensor_data),
             node.create_subscription(PanelDetectionArray, '/cv/panel_detections',
                                      detection.append, 100),
             node.create_subscription(Header, '/sim/e2e/aim_tick', aim.append, 100),
             node.create_subscription(Header, '/mcb_emulator/tick', tick.append, 100),
             node.create_subscription(Header, '/target_manager/frame_ack', frame_ack.append, 100),
             node.create_subscription(Odometry, '/target/ground_truth_odom' if stage ==
                                      'mcb_parked' else '/sim/match/opponent_0/path',
                                      path.append, 100)]
    observer_ready = node.create_publisher(Header, '/sim/e2e/observer_ready', qos)
    log = tempfile.TemporaryFile(mode='w+')
    env = launch_environment()
    proc = subprocess.Popen(['ros2', 'launch', 'sim', 'e2e.launch.py', 'run_tests:=false',
                             'stage:=' + stage, 'backend:=hosted',
                             'firmware_binary:=' + firmware,
                             'uart_deterministic:=true', 'deterministic_epoch_zero:=true',
                             'epoch_coordinator_delay_s:=' + str(delay),
                             'epoch_readiness_extra:=/sim/e2e/observer_ready',
                             'headless:=true', 'foxglove:=false'], stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True, env=env)
    try:
        deadline = time.monotonic() + 35
        announced = False
        while time.monotonic() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
            if (not announced and len(ready) == len(topics) and clock and
                    all(sub.get_publisher_count() for sub in subs)):
                assert all(ns(msg.clock) == 0 for msg in clock), clock
                assert not tick, tick
                observer_ready.publish(Header())
                announced = True
            if (len(ready) == len(topics) and len(detection) >= 2 and len(aim) >= 2 and
                    len(tick) >= 2 and len(path) >= 2 and len(frame_ack) >= 2):
                break
        if (len(ready) != len(topics) or len(detection) < 2 or len(aim) < 2 or
                len(tick) < 2 or len(path) < 2 or len(frame_ack) < 2):
            log.seek(0)
            raise AssertionError(f'incomplete: {ready} {len(clock)} {len(detection)} '
                                 f'{len(aim)} {len(tick)}\n{log.read()}')
        assert all(value == 0 for value in ready.values()), ready
        assert any(ns(msg.clock) == 0 for msg in clock), 'no observed zero clock'
        assert [ns(msg.stamp) for msg in frame_ack[:2]] == [17000000, 34000000], frame_ack
        values = ([ns(msg.header.stamp) for msg in detection[:2]],
                  [ns(msg.stamp) for msg in aim[:2]], [ns(msg.stamp) for msg in tick[:2]],
                  [ns(msg.header.stamp) for msg in path[:2]])
        print(stage, 'first detection, aim, firmware, path stamps:', values, flush=True)
        return values
    finally:
        stop(proc, env['GZ_PARTITION'])
        node.destroy_node()
        del subs
        log.close()


def main():
    rclpy.init()
    try:
        for stage in ('mcb_parked', 'mcb_drive', 'mcb_match'):
            first = run(sys.argv[1], 0, stage)
            second = run(sys.argv[1], 6, stage)
            assert first == second, (first, second)
            assert first[0] == [17000000, 34000000], first
            assert first[1] == [25000000, 50000000], first
            assert first[2] == [1000000, 2000000], first
            assert first[3] == ([17000000, 34000000] if stage == 'mcb_parked' else
                                [10000000, 20000000]), first
            missing_ready(sys.argv[1], stage)
    finally:
        rclpy.shutdown()


def missing_ready(firmware, stage):
    node = rclpy.create_node('e2e_missing_observer')
    clocks, ticks = [], []
    subs = [node.create_subscription(Clock, '/clock', clocks.append, qos_profile_sensor_data),
            node.create_subscription(Header, '/mcb_emulator/tick', ticks.append, 100)]
    log = tempfile.TemporaryFile(mode='w+')
    env = launch_environment()
    proc = subprocess.Popen(['ros2', 'launch', 'sim', 'e2e.launch.py', 'run_tests:=false',
                             'stage:=' + stage, 'backend:=hosted', 'firmware_binary:=' + firmware,
                             'uart_deterministic:=true', 'deterministic_epoch_zero:=true',
                             'headless:=true', 'foxglove:=false',
                             'epoch_readiness_extra:=/sim/e2e/never_ready',
                             'epoch_startup_timeout_s:=10'], stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True, env=env)
    try:
        deadline = time.monotonic() + 25
        while proc.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        log.seek(0)
        output = log.read()
        assert proc.poll() not in (None, 0), f'failure did not propagate\n{output}'
        assert 'startup timeout' in output, output
        assert clocks and all(ns(msg.clock) == 0 for msg in clocks), clocks
        assert not ticks, ticks
        print(stage, 'missing readiness failed closed and propagated', flush=True)
    finally:
        stop(proc, env['GZ_PARTITION'])
        node.destroy_node()
        del subs
        log.close()


if __name__ == '__main__':
    main()
