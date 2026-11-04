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
"""Exercise the Gazebo plugin, firmware, PTY and bridge with an ack proxy."""
import copy
import os
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from dji_serial_bridge.msg import RobotPose, SimFrameAck, SimFrameMetadata
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def run_case(mode, firmware):
    node = rclpy.create_node('uart_plugin_test_' + mode)
    metadata, acks, poses, clocks = [], [], [], []
    ack_sub = node.create_subscription(SimFrameAck, '/test/bridge_ack', acks.append, 32)
    subs = [node.create_subscription(SimFrameMetadata, '/dji_serial_bridge/sim_frame_metadata',
                                     metadata.append, 32),
            ack_sub,
            node.create_subscription(RobotPose, '/dji_serial_bridge/pose', poses.append,
                                     qos_profile_sensor_data),
            node.create_subscription(Clock, '/clock', clocks.append, qos_profile_sensor_data)]
    device = f'/tmp/uart-plugin-test-{mode}-{os.getpid()}-{time.monotonic_ns()}'
    log = tempfile.TemporaryFile(mode='w+')
    processes = []

    def start(args):
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def spin_until(predicate, seconds, label):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if predicate():
                return
        log.flush()
        log.seek(0)
        raise AssertionError(f'{mode}: {label}; metadata={metadata[:1]}, '
                             f'acks={acks[:1]}, clocks={clocks[-3:]}\n{log.read()}')

    def failed(reason):
        log.flush()
        pos = log.tell()
        log.seek(0)
        output = log.read()
        log.seek(pos)
        return reason in output

    try:
        start(['ros2', 'launch', 'sim', 'sim.launch.py', 'gui:=false', 'rviz:=false',
               'foxglove:=false', 'camera:=false', 'pose_emulator:=false', 'mcb_physics:=true',
               'uart_deterministic:=true', 'lockstep:=true', 'mcb_auto_fire:=false',
               'mcb_device_link:=' + device, 'firmware_binary:=' + firmware])
        spin_until(lambda: os.path.exists(device), 30, 'PTY not created')
        start(['ros2', 'run', 'dji_serial_bridge', 'dji_serial_bridge_node', '--ros-args',
               '-p', 'device:=' + device, '-p', 'use_sim_time:=true',
               '-p', 'sim_frame_metadata:=true', '-p', 'sim_frame_timeout_ms:=1000',
               '-r', '/dji_serial_bridge/sim_frame_ack:=/test/bridge_ack'])
        spin_until(lambda: ack_sub.get_publisher_count() > 0, 10, 'bridge discovery')
        ack_pub = node.create_publisher(SimFrameAck, '/dji_serial_bridge/sim_frame_ack', 32)
        spin_until(lambda: any(a.frame_id == m.frame_id for a in acks for m in metadata) and
                   bool(clocks) and
                   ack_pub.get_subscription_count() > 0, 30, 'first frame')
        first = next(a for a in acks if any(m.frame_id == a.frame_id for m in metadata))
        frame = next(m for m in metadata if m.frame_id == first.frame_id)
        assert first.published and ns(first.header.stamp) == ns(frame.delivery_time), (
            first, frame)
        spin_until(lambda: clocks and ns(clocks[-1].clock) >= ns(frame.delivery_time),
                   2, 'delivery clock')
        held = ns(frame.delivery_time)
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < 0.25:
            rclpy.spin_once(node, timeout_sec=0.01)
            assert ns(clocks[-1].clock) == held, (mode, held, ns(clocks[-1].clock))
        if mode == 'timeout':
            spin_until(lambda: failed('UART acknowledgment or ROS clock timeout'), 2.5,
                       'missing ack did not time out')
            assert ns(clocks[-1].clock) == held
            elapsed = time.monotonic() - start_wait
            assert 1.0 <= elapsed <= 2.5, elapsed
        else:
            injected = copy.deepcopy(first)
            if mode == 'wrong_id':
                injected.frame_id += 100
            elif mode in ('early', 'stale', 'future'):
                delta = {'early': -1, 'stale': -1000000, 'future': 1000000}[mode]
                value = held + delta
                injected.header.stamp.sec, injected.header.stamp.nanosec = divmod(
                    value, 1000000000)
            ack_pub.publish(injected)
            if mode == 'duplicate':
                ack_pub.publish(injected)
            if mode in ('wrong_id', 'early', 'stale', 'future', 'duplicate'):
                spin_until(lambda: failed('invalid UART acknowledgment'), 2.5,
                           'invalid ack accepted')
            else:
                forwarded = 1

                def progressed():
                    nonlocal forwarded
                    while forwarded < len(acks):
                        ack_pub.publish(acks[forwarded])
                        forwarded += 1
                    return len(acks) >= 20 and len(poses) >= 3 and ns(clocks[-1].clock) > held
                spin_until(progressed, 10, 'clock did not resume')
                spin_until(lambda: all(any(m.frame_id == a.frame_id for m in metadata)
                                       for a in acks), 2, 'metadata observation lag')
                by_id = {m.frame_id: m for m in metadata}
                assert len({a.frame_id for a in acks}) == len(acks), acks
                assert all(a.published and a.frame_id in by_id and
                           ns(a.header.stamp) == ns(by_id[a.frame_id].delivery_time)
                           for a in acks), (acks, metadata)
                pose_stamps = {ns(m.header.stamp) for m in metadata if len(m.frame) >= 7 and
                               int.from_bytes(m.frame[5:7], 'little') == 2}
                assert all(ns(p.header.stamp) in pose_stamps for p in poses), (poses, metadata)
        observed = [ns(c.clock) for c in clocks]
        assert all(a < b for a, b in zip(observed, observed[1:])), observed
        print(f'{mode}: PASS held={held} frames={len(acks)} poses={len(poses)}', flush=True)
    finally:
        rows = subprocess.check_output(['ps', '-eo', 'pid=,ppid=,pgid='], text=True)
        entries = [tuple(map(int, row.split())) for row in rows.splitlines()]
        descendants = {p.pid for p in processes}
        while True:
            expanded = descendants | {pid for pid, parent, _ in entries if parent in descendants}
            if expanded == descendants:
                break
            descendants = expanded
        groups = {group for pid, _, group in entries
                  if pid in descendants and group != os.getpgrp()}
        for group in groups:
            try:
                os.killpg(group, signal.SIGINT)
            except ProcessLookupError:
                pass
        for process in reversed(processes):
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                pass
        for group in groups:
            try:
                os.killpg(group, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for process in processes:
            process.wait()
        node.destroy_node()
        log.close()
        del subs


def main():
    if len(sys.argv) not in (2, 3) or not os.access(sys.argv[1], os.X_OK):
        raise SystemExit('pass the explicit hosted firmware executable and optional case')
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = os.path.join(
        get_package_share_directory('thornbots_pkg'), 'config', 'fastdds_udp_only.xml')
    rclpy.init()
    try:
        cases = ('success', 'timeout', 'wrong_id', 'early', 'stale', 'future', 'duplicate')
        for mode in (sys.argv[2],) if len(sys.argv) == 3 else cases:
            if mode not in cases:
                raise ValueError(mode)
            run_case(mode, sys.argv[1])
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
