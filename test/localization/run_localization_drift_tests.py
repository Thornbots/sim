#!/usr/bin/env python3
"""
argparse wrapper around test_localization_drift.py, so the documented
`ros2 run sim run_localization_drift_tests.py --backend slam` and
`python3 .../run_localization_drift_tests.py` workflows keep working now
that the suite is a standard pytest suite. Every flag maps 1:1 onto a
pytest option declared in test/conftest.py; `-m integration` overrides
setup.cfg's default deselection. Exits with pytest's status.
"""
import os
import subprocess
import sys

TEST_FILE = 'test_localization_drift.py'
# Installed into lib/sim/ by setup.py's `scripts=`, where the test files
# aren't; fall back to the bind-mounted source tree, same absolute path
# drift_harness.WORKSPACE already assumes.
SOURCE_FALLBACK = '/workspaces/isaac_ros-dev/src/sim/test/localization'


def test_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (here, SOURCE_FALLBACK):
        if os.path.exists(os.path.join(candidate, TEST_FILE)):
            return candidate
    sys.exit(f'could not find {TEST_FILE} in {here} or {SOURCE_FALLBACK}')


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['slam', 'amcl', 'none'],
                        default='amcl',
                        help="auto.launch.py's localization_mode -- who owns "
                             "map->odom (default: amcl). 'mapping' isn't "
                             'offered; see README.md BACKENDS.')
    parser.add_argument('--use-ekf', action='store_true',
                        help='EKF-fuse odom->root instead of passing /odom '
                             'through raw. Independent of --backend; the old '
                             'standalone ekf backend is --backend none '
                             '--use-ekf.')
    parser.add_argument('--scenario',
                        help='run only this scenario (default: all, in suite '
                             'order)')
    parser.add_argument('--headless', action='store_true',
                        help="skip gz-sim's GUI window and rviz2 (both on by "
                             'default)')
    parser.add_argument('--speed', type=float,
                        help='m/s for the cornering loop and its start-corner '
                             'reposition; other speeds are not re-validated '
                             'against the thresholds')
    args, extra = parser.parse_known_args()

    cmd = [sys.executable, '-m', 'pytest', os.path.join(test_dir(), TEST_FILE),
           '-m', 'integration', '-v', '-s',
           '--backend', args.backend]
    if args.use_ekf:
        cmd.append('--use-ekf')
    if args.scenario:
        cmd += ['--scenario', args.scenario]
    if args.headless:
        cmd.append('--headless')
    if args.speed is not None:
        cmd += ['--speed', str(args.speed)]
    cmd += extra

    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
