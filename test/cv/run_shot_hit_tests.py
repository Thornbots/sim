#!/usr/bin/env python3
"""
argparse wrapper around test_shot_hit.py, keeping the documented
`python3 .../run_shot_hit_tests.py` / `ros2 run sim
run_shot_hit_tests.py` workflows working now that the suite is a
standard pytest suite. Every flag maps 1:1 onto a pytest option declared
in test/conftest.py. Exits with pytest's status.
"""
import argparse
import os
import subprocess
import sys

TEST_FILE = 'test_shot_hit.py'
# Installed into lib/sim/ by setup.py's `scripts=`, where the test files
# aren't; fall back to the bind-mounted source tree.
SOURCE_FALLBACK = '/workspaces/isaac_ros-dev/src/sim/test/cv'


def test_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (here, SOURCE_FALLBACK):
        if os.path.exists(os.path.join(candidate, TEST_FILE)):
            return candidate
    sys.exit(f'could not find {TEST_FILE} in {here} or {SOURCE_FALLBACK}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', type=float, nargs='+',
                        help='target speeds (m/s) for the sweep')
    parser.add_argument('--duration', type=float,
                        help='seconds of steady-state sampling per case '
                             '(wall-clock)')
    parser.add_argument('--hit-radius', type=float,
                        help='perpendicular miss distance (m) still counted '
                             'as a hit')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--skip-stationary', action='store_true',
                        help='skip the speed=0/spin=0 baseline case')
    parser.add_argument('--only-stationary', action='store_true',
                        help='run only the speed=0/spin=0 baseline case')
    parser.add_argument('--lead', choices=['off', 'on', 'both'],
                        default='both',
                        help="point_to_cv_target's lead_enabled -- 'both' "
                             'runs every case twice for a before/after '
                             'hit-rate table')
    parser.add_argument('--log-dir', default='/tmp/shot_hit_test_logs')
    args, extra = parser.parse_known_args()

    cmd = [sys.executable, '-m', 'pytest', os.path.join(test_dir(), TEST_FILE),
           '-m', 'integration', '-v', '-s']
    if args.speeds:
        cmd += ['--shot-speeds', ','.join(str(v) for v in args.speeds)]
    cmd += ['--lead', args.lead, '--log-dir', args.log_dir]
    if args.duration is not None:
        cmd += ['--shot-duration', str(args.duration)]
    if args.hit_radius is not None:
        cmd += ['--hit-radius', str(args.hit_radius)]
    if args.headless:
        cmd.append('--headless')
    if args.skip_stationary:
        cmd.append('--skip-stationary')
    if args.only_stationary:
        cmd.append('--only-stationary')
    cmd += extra

    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
