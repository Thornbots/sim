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
Shared options and fixtures for sim's test suites.

Both stack-launching suites (test/localization, test/cv) carry the
`integration` marker and are deselected by setup.cfg's default addopts,
so a plain `colcon test --packages-select sim` stays fast and doesn't
collide with a live sim session. Opt in with
`colcon test --packages-select sim --pytest-args ' -m integration'`, or
use `ros2 launch sim localization_tests.launch.py` or `ros2 launch sim
shot_hit.launch.py`, whose args map onto the options declared here.
"""
import pytest


def pytest_addoption(parser):
    group = parser.getgroup('sim integration')
    group.addoption(
        '--headless', action='store_true',
        help="skip gz-sim's GUI window and rviz2 (both on by default, per "
             'sim/AGENTS.md\'s standing "watch sim live" rule)')
    group.addoption(
        '--restart-sim', action='store_true',
        help='bring the sim up fresh for every localization scenario instead '
             'of once per run (the old behaviour; compare verdicts with it)')
    group.addoption(
        '--real-time-factor', default='0',
        help="sim.launch.py's real_time_factor for every stack a suite "
             'launches; 0 (default) runs unthrottled, the suites time in sim '
             'seconds')

    # test/localization
    group.addoption(
        '--backend', choices=['slam', 'amcl', 'none'], default='amcl',
        help="auto.launch.py's localization_mode -- who owns map->odom")
    group.addoption(
        '--use-ekf', action='store_true', dest='use_ekf', default=True,
        help="auto.launch.py's use_ekf, on by default to match its default "
             "(independent axis; the old standalone 'ekf' backend is "
             '--backend none)')
    group.addoption(
        '--no-use-ekf', action='store_false', dest='use_ekf',
        help='forward use_ekf:=false instead -- raw /odom passthrough, no '
             'ekf_node and no rf2o')
    group.addoption(
        '--scenario', default=None,
        help='run only this drift scenario (default: all, in suite order)')
    group.addoption(
        '--speed', type=float, default=None,
        help='m/s for the cornering loop; see drift_harness.DRIVE_SPEED, '
             'other speeds are not re-validated against the thresholds')

    group.addoption(
        '--ekf-slip-ratio', type=float, default=0.05,
        help='fraction of every driven meter lost from reported odometry '
             '(test_ekf_ground_truth.py)')
    group.addoption(
        '--ekf-drift-stddev', type=float, default=0.002,
        help='per-sample stddev of the odometry drift random walk')
    group.addoption(
        '--ekf-seconds', type=float, default=45.0,
        help='sim seconds test_ekf_ground_truth.py drives the cornering loop')

    # test/cv
    group.addoption(
        '--shot-speeds', default=None,
        help='comma-separated target speeds (m/s) for the shot-hit sweep, '
             'e.g. 0.5,1,2,4')
    group.addoption(
        '--shot-duration', type=float, default=None,
        help='seconds of steady-state sampling per shot-hit case (sim time)')
    group.addoption(
        '--hit-radius', type=float, default=None,
        help='perpendicular miss distance (m) still counted as a hit')
    group.addoption(
        '--panel-layout', choices=['flat', 'staggered', 'both'], default='both',
        help='target panel heights: flat, staggered by 90% of a panel, or both')
    group.addoption(
        '--target-path', choices=['lateral', 'radial', 'diagonal'], default='lateral',
        help="target_driver's path for every case: across the view, down the "
             'camera ray, or both (shot_hit_harness.TARGET_PATHS)')
    group.addoption(
        '--shooter-speed', type=float, default=0.0,
        help='our own chassis speed (m/s) the stack was launched with, for labels '
             "and shots.jsonl; shot_hit.launch.py's shooter_speed sets the motion")
    group.addoption(
        '--blackout', action='store_true',
        help='estimation bench: drop every detection for 0.3 s in each 2 s '
             '(estimation_harness.BLACKOUT)')
    group.addoption(
        '--camera-latency', type=float, default=0.0,
        help='estimation bench: the camera_latency_s the stack was launched with, '
             'for cell names; estimation.launch.py sets it')
    group.addoption(
        '--skip-stationary', action='store_true',
        help='skip the speed=0/spin=0 baseline case')
    group.addoption(
        '--only-stationary', action='store_true',
        help='run only the speed=0/spin=0 baseline case')
    group.addoption(
        '--log-dir', default=None,
        help='where the stack log, shots.jsonl and panel_hits.jsonl are written')
    group.addoption(
        '--external-stack', action='store_true',
        help='the stack is already up (shot_hit.launch.py started this pytest); '
             'wait for it instead of launching one')


@pytest.fixture(scope='session')
def ros_context():
    """
    One rclpy context for the whole session, not one per test.

    Both suites launch and tear down several stacks under a single
    context. Imported lazily so the pure-Python unit tests still collect
    on a bare pytest install with no ROS.
    """
    import rclpy
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


@pytest.fixture
def gui(request):
    return not request.config.getoption('--headless')
