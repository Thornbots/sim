# Copyright 2026 Thornbots
# Licensed under the Apache License, Version 2.0.
"""Observe the real parked launch; pass the hosted firmware ELF as argv[1]."""
import os
import signal
import subprocess
import sys
import tempfile
import time

from dji_serial_bridge.msg import PanelDetectionArray
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


def run(firmware, delay):
    node = rclpy.create_node('e2e_epoch_observer')
    ready, clock, detection, aim, tick = {}, [], [], [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topics = ['/detector_standin/ready', '/point_to_cv_target/epoch_zero_ready',
              '/opponent_driver/ready', '/mcb_emulator/epoch_zero_ready',
              '/robot_spawn_ready/ready']
    subs = [node.create_subscription(Header, topic,
            lambda msg, topic=topic: ready.setdefault(topic, ns(msg.stamp)), qos)
            for topic in topics]
    subs += [node.create_subscription(Clock, '/clock', clock.append, qos_profile_sensor_data),
             node.create_subscription(PanelDetectionArray, '/cv/panel_detections',
                                      detection.append, 100),
             node.create_subscription(Header, '/sim/e2e/aim_tick', aim.append, 100),
             node.create_subscription(Header, '/mcb_emulator/tick', tick.append, 100)]
    log = tempfile.TemporaryFile(mode='w+')
    proc = subprocess.Popen(['ros2', 'launch', 'sim', 'e2e.launch.py', 'run_tests:=false',
                             'stage:=mcb_parked', 'backend:=hosted',
                             'firmware_binary:=' + firmware,
                             'uart_deterministic:=true', 'deterministic_epoch_zero:=true',
                             'epoch_coordinator_delay_s:=' + str(delay),
                             'headless:=true', 'foxglove:=false'], stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True,
                            env=launch_environment())
    try:
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
            if len(ready) == 5 and len(detection) >= 2 and len(aim) >= 2 and len(tick) >= 2:
                break
        if len(ready) != 5 or len(detection) < 2 or len(aim) < 2 or len(tick) < 2:
            log.seek(0)
            raise AssertionError(f'incomplete: {ready} {len(clock)} {len(detection)} '
                                 f'{len(aim)} {len(tick)}\n{log.read()}')
        assert all(value == 0 for value in ready.values()), ready
        assert any(ns(msg.clock) == 0 for msg in clock), 'no observed zero clock'
        values = ([ns(msg.header.stamp) for msg in detection[:2]],
                  [ns(msg.stamp) for msg in aim[:2]], [ns(msg.stamp) for msg in tick[:2]])
        print('first detection, aim, firmware stamps:', values, flush=True)
        return values
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        node.destroy_node()
        del subs
        log.close()


def main():
    rclpy.init()
    try:
        first = run(sys.argv[1], 0)
        second = run(sys.argv[1], 6)
        assert first == second, (first, second)
        assert first[0] == [17000000, 34000000], first
        assert first[1] == [25000000, 50000000], first
        assert first[2] == [1000000, 2000000], first
        missing_ready(sys.argv[1])
    finally:
        rclpy.shutdown()


def missing_ready(firmware):
    node = rclpy.create_node('e2e_missing_observer')
    clocks, ticks = [], []
    subs = [node.create_subscription(Clock, '/clock', clocks.append, qos_profile_sensor_data),
            node.create_subscription(Header, '/mcb_emulator/tick', ticks.append, 100)]
    log = tempfile.TemporaryFile(mode='w+')
    proc = subprocess.Popen(['ros2', 'launch', 'sim', 'e2e.launch.py', 'run_tests:=false',
                             'backend:=hosted', 'firmware_binary:=' + firmware,
                             'uart_deterministic:=true', 'deterministic_epoch_zero:=true',
                             'headless:=true', 'foxglove:=false',
                             'epoch_readiness_extra:=/sim/e2e/never_ready',
                             'epoch_startup_timeout_s:=10'], stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True,
                            env=launch_environment())
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
        print('missing readiness failed closed and propagated', flush=True)
    finally:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        node.destroy_node()
        del subs
        log.close()


if __name__ == '__main__':
    main()
