#!/usr/bin/env python3
"""
argparse wrapper around test_ekf_ground_truth.py, keeping the documented
`ros2 run sim ekf_ground_truth_diag.py` / `python3 .../
ekf_ground_truth_diag.py` workflows working now that the assertion lives
in a standard pytest suite. Exits 0 if EKF beat raw /odom on mean error.
"""
import argparse
import os
import subprocess
import sys

TEST_FILE = 'test_ekf_ground_truth.py'
# See run_localization_drift_tests.py for why the fallback exists.
SOURCE_FALLBACK = '/workspaces/isaac_ros-dev/src/sim/test/localization'


def test_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (here, SOURCE_FALLBACK):
        if os.path.exists(os.path.join(candidate, TEST_FILE)):
            return candidate
    sys.exit(f'could not find {TEST_FILE} in {here} or {SOURCE_FALLBACK}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true',
                        help='run gz-sim without its GUI')
    parser.add_argument('--slip-ratio', type=float, default=0.05,
                        help='fraction of every driven meter lost from '
                             'reported odometry (default 0.05)')
    parser.add_argument('--drift-stddev', type=float, default=0.002,
                        help='per-sample stddev of the odometry drift random '
                             'walk (default 0.002)')
    parser.add_argument('--seconds', type=float, default=45.0,
                        help='how long to drive the cornering loop')
    args, extra = parser.parse_known_args()

    cmd = [sys.executable, '-m', 'pytest', os.path.join(test_dir(), TEST_FILE),
           '-m', 'integration', '-v', '-s',
           '--ekf-slip-ratio', str(args.slip_ratio),
           '--ekf-drift-stddev', str(args.drift_stddev),
           '--ekf-seconds', str(args.seconds)]
    if args.headless:
        cmd.append('--headless')
    cmd += extra

    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
