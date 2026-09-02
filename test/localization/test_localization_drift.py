"""
Integration suite for sentry_localization's map-relative drift/jerk
correction, against sim's synthetic odom noise model
(sim/pose_emulator.py). One test per scenario, in suite order; the stack
lifecycle and the scenarios themselves live in drift_harness.py.

Launches gz-sim and the full sentry stack, so every test here is marked
`integration` and skipped by a plain `colcon test`. Options:
--backend {slam,amcl,none}, --use-ekf, --scenario NAME, --headless,
--speed M/S (see test/conftest.py). See README.md for WHY THIS EXISTS,
BACKENDS, and SCENARIOS.
"""
import pytest

import drift_harness

pytestmark = pytest.mark.integration


def pytest_generate_tests(metafunc):
    if 'scenario_name' in metafunc.fixturenames:
        only = metafunc.config.getoption('--scenario')
        if only is not None and only not in drift_harness.SCENARIOS:
            raise pytest.UsageError(
                f'--scenario {only!r} is not one of '
                f'{sorted(drift_harness.SCENARIOS)}')
        names = [only] if only else list(drift_harness.SCENARIOS)
        metafunc.parametrize('scenario_name', names)


@pytest.fixture(scope='module', autouse=True)
def orphan_check():
    """Warns about a colliding sim/localization stack this suite didn't
    start. A live session shares topics and services with ours and
    silently corrupts every measurement below; see sim/AGENTS.md."""
    drift_harness.check_no_orphans('pre-flight')
    yield
    drift_harness.check_no_orphans(
        'post-flight (should be empty if teardown worked)')


@pytest.fixture(scope='module', autouse=True)
def drive_speed(request):
    speed = request.config.getoption('--speed')
    if speed is not None:
        drift_harness.set_drive_speed(speed)


def test_scenario(scenario_name, request, gui, ros_context):
    backend = request.config.getoption('--backend')
    use_ekf = request.config.getoption('--use-ekf')

    sc = drift_harness.run_scenario(scenario_name, gui, backend, use_ekf)

    detail = '\n'.join(sc.details)
    if sc.skipped:
        pytest.skip(detail)
    assert sc.passed, f'{scenario_name} (backend={backend}, ' \
                      f'use_ekf={use_ekf}) failed:\n{detail}'
