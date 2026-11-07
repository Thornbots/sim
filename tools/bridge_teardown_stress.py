#!/usr/bin/env python3
"""Early-interrupt stress for the gz_bridge teardown (Fast DDS) race.

Each iteration starts four gz_bridge processes with the sim's message types (the
publishers whose Fast DDS receive threads handle a peer's reader removal) and
three rclpy subscribers (those peers), waits a random time, then SIGINTs every
process back to back, as launch does, and records how each exited. A bridge must
exit 0 without its teardown watchdog firing; the subscribers are only peers (an rclpy node interrupted before it
exists exits 1, which is not counted). Run it in a container with sim built and
sourced; set GZ_BRIDGE to test another binary.

  bridge_teardown_stress.py ITERATIONS MIN_S MAX_S OUTDIR [SEED]

Exit status is 1 if any bridge died. Per-iteration logs are kept in OUTDIR.
"""
import os
import random
import signal
import subprocess
import sys
import time

BRIDGES = [
    ['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '--ros-args', '-r', '/scan:=/sim/scan'],
    ['/m/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model', '--ros-args', '-r',
     '/m/joint_state:=/sim/raw_joint_states'],
    ['/m/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry', '--ros-args', '-r',
     '/m/odometry:=/sim/raw_odom'],
    ['/cmd@geometry_msgs/msg/Twist]gz.msgs.Twist', '--ros-args', '-r', '/cmd:=/sim/cmd_vel'],
]
SUBSCRIBER = '''
import sys
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
rclpy.init()
node = Node(sys.argv[1])
for topic, kind in (('/sim/scan', LaserScan), ('/sim/raw_joint_states', JointState),
                    ('/sim/raw_odom', Odometry)):
    node.create_subscription(kind, topic, lambda m: None, 10)
node.create_publisher(Twist, '/sim/cmd_vel', 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
'''


def prefix_binary():
    out = subprocess.run(['ros2', 'pkg', 'prefix', 'sim'], capture_output=True, text=True, check=True)
    return os.path.join(out.stdout.strip(), 'lib', 'sim', 'gz_bridge')


def iteration(binary, outdir, low, high):
    os.makedirs(outdir, exist_ok=True)
    procs = []

    def spawn(name, argv):
        log = open(os.path.join(outdir, name + '.log'), 'w')
        procs.append((name, subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                             start_new_session=True)))

    order = list(range(len(BRIDGES)))
    random.shuffle(order)
    for k in order:
        spawn(f'bridge{k}', [binary] + BRIDGES[k])
        time.sleep(random.uniform(0, 0.15))
    for k in range(3):
        spawn(f'sub{k}', [sys.executable, '-c', SUBSCRIBER, f'sub{k}'])
        time.sleep(random.uniform(0, 0.15))
    time.sleep(random.uniform(low, high))
    for _, proc in procs:
        if proc.poll() is None:
            os.kill(proc.pid, signal.SIGINT)
    deadline = time.time() + 20
    failed = {}
    for name, proc in procs:
        try:
            status = proc.wait(max(0.1, deadline - time.time()))
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            status = 'hung'
        if name.startswith('bridge') and status != 0:
            failed[name] = status
        elif name.startswith('bridge') and 'teardown exceeded' in open(
                os.path.join(outdir, name + '.log')).read():
            failed[name] = 'watchdog'  # exited 0 only because the teardown watchdog fired
    return failed


def main():
    iterations, low, high, out = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    random.seed(int(sys.argv[5]) if len(sys.argv) > 5 else 1)
    binary = os.environ.get('GZ_BRIDGE') or prefix_binary()
    deaths = 0
    for i in range(iterations):
        failed = iteration(binary, os.path.join(out, f'{i:04d}'), low, high)
        deaths += len(failed)
        print(f'iter {i}: ' + (f'FAIL {failed}' if failed else 'ok'), flush=True)
    print(f'bridge_teardown_stress: {iterations} iterations, {deaths} bridge deaths', flush=True)
    return 1 if deaths else 0


if __name__ == '__main__':
    sys.exit(main())
