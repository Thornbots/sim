"""
Shared options and fixtures for sim's test suites.

Both stack-launching suites (test/localization, test/cv) carry the
`integration` marker and are deselected by setup.cfg's default addopts,
so a plain `colcon test --packages-select sim` stays fast and doesn't
collide with a live sim session. Opt in with
`colcon test --packages-select sim --pytest-args ' -m integration'`, or
use the argparse wrappers next to each suite, whose flags map 1:1 onto
the options declared here.
"""
import pytest


def pytest_addoption(parser):
    group = parser.getgroup('sim integration')
    group.addoption(
        '--headless', action='store_true',
        help='skip gz-sim\'s GUI window and rviz2 (both on by default, per '
             'sim/AGENTS.md\'s standing "watch sim live" rule)')

    # test/localization
    group.addoption(
        '--backend', choices=['slam', 'amcl', 'none'], default='amcl',
        help="auto.launch.py's localization_mode -- who owns map->odom")
    group.addoption(
        '--use-ekf', action='store_true',
        help='forward use_ekf:=true to auto.launch.py (independent axis; '
             "the old standalone 'ekf' backend is --backend none --use-ekf)")
    group.addoption(
        '--scenario', default=None,
        help='run only this drift scenario (default: all, in suite order)')
    group.addoption(
        '--speed', type=float, default=None,
        help="m/s for the cornering loop; see drift_harness.DRIVE_SPEED, "
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
        help='how long test_ekf_ground_truth.py drives the cornering loop')

    # test/cv
    group.addoption(
        '--shot-speeds', default=None,
        help='comma-separated target speeds (m/s) for the shot-hit sweep, '
             'e.g. 0.5,1,2,4')
    group.addoption(
        '--shot-duration', type=float, default=None,
        help='seconds of steady-state sampling per shot-hit case')
    group.addoption(
        '--hit-radius', type=float, default=None,
        help='perpendicular miss distance (m) still counted as a hit')
    group.addoption(
        '--lead', choices=['off', 'on', 'both'], default='both',
        help="point_to_cv_target's lead_enabled; 'both' runs every case "
             'twice for a before/after hit-rate table')
    group.addoption(
        '--skip-stationary', action='store_true',
        help='skip the speed=0/spin=0 baseline case')
    group.addoption(
        '--only-stationary', action='store_true',
        help='run only the speed=0/spin=0 baseline case')
    group.addoption(
        '--log-dir', default=None,
        help='where per-node launch logs are written')


@pytest.fixture(scope='session')
def ros_context():
    """One rclpy context for the whole session, not one per test -- both
    suites launch and tear down several stacks under a single context.
    Imported lazily so the pure-Python unit tests still collect on a bare
    pytest install with no ROS."""
    import rclpy
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


@pytest.fixture
def gui(request):
    return not request.config.getoption('--headless')
