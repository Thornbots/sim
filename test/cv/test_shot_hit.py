"""
Black-box shot-hit test: scores sentry_pkg's fire decisions against
ground truth, consuming only the final FireCommand on
/dji_serial_bridge/fire_command (the topic that would reach the real
launcher per mcb_relay.py's "sole relay" design). Scoring geometry and
stack lifecycle live in shot_hit_harness.py.

One test per (lead, case) cell, printing a hit-rate line each, so a full
run is the before/after lead table. The assertion is that shots are
actually observed; the stationary baseline additionally asserts hits,
since a working pipeline hits a motionless target trivially (see
shot_hit_harness.STATIONARY_MIN_HITS).

Launches gz-sim, so marked `integration` and skipped by a plain
`colcon test`. Options: --shot-speeds, --shot-duration, --hit-radius,
--lead, --skip-stationary, --only-stationary, --headless, --log-dir.
"""
import os
import time

import pytest

import shot_hit_harness as harness

pytestmark = pytest.mark.integration

STATIONARY = 'stationary'


def _speeds(config):
    raw = config.getoption('--shot-speeds')
    if not raw:
        return harness.DEFAULT_SPEEDS
    return [float(v) for v in raw.split(',') if v.strip()]


def pytest_generate_tests(metafunc):
    if 'case' not in metafunc.fixturenames:
        return
    config = metafunc.config
    speeds = _speeds(config)
    lead = config.getoption('--lead')
    lead_values = [False, True] if lead == 'both' else [lead == 'on']

    cases = []
    if not config.getoption('--skip-stationary'):
        cases.append(STATIONARY)
    if not config.getoption('--only-stationary'):
        cases.extend(speeds)

    params = [(lead_enabled, case)
              for lead_enabled in lead_values for case in cases]
    ids = [f'lead-{"on" if lead_enabled else "off"}-'
           f'{case if case == STATIONARY else f"speed{case}"}'
           for lead_enabled, case in params]
    metafunc.parametrize('lead_enabled,case', params, ids=ids)


def test_shot_hit(lead_enabled, case, request, gui, ros_context):
    config = request.config
    speeds = _speeds(config)
    duration = config.getoption('--shot-duration') or harness.DEFAULT_DURATION
    hit_radius = config.getoption('--hit-radius') or harness.DEFAULT_HIT_RADIUS
    log_dir = config.getoption('--log-dir') or harness.DEFAULT_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)

    if case == STATIONARY:
        speed, spin_hz = 0.0, 0.0
    else:
        speed = case
        spin_hz = harness.spin_hz_for_speed(speed, min(speeds), max(speeds))

    label = (f'{"stationary baseline" if case == STATIONARY else f"speed={speed} m/s"}'
             f', spin={spin_hz:.2f} Hz | lead={"on" if lead_enabled else "off"}')
    print(f'\n=== {label} ===')

    sampler, dropped = harness.run_one_speed(
        speed, spin_hz, duration, not gui, log_dir, hit_radius,
        lead_enabled=lead_enabled)
    harness.summarize(label, sampler, dropped)
    # Let the previous case's stack fully release its topics/services
    # before the next one launches its own.
    time.sleep(1.0)

    assert sampler.shots_fired > 0, (
        f'no shots observed in {label} -- something in the launched stack is '
        'broken (mcb_relay not relaying, point_to_cv_target not firing, or a '
        f'node failed to start; check the per-node logs in {log_dir})')

    if case == STATIONARY:
        assert sampler.hits >= harness.STATIONARY_MIN_HITS, (
            f'{sampler.hits} hits out of {sampler.shots_fired} shots at a '
            f'motionless target ({label}) -- see STATIONARY_MIN_HITS')
