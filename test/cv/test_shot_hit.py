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
Black-box shot-hit test: scores thornbots_pkg's fire decisions against ground truth.

Consumes only the final FireCommand on /dji_serial_bridge/fire_command
(the topic that would reach the real launcher per mcb_relay.py's "sole
relay" design). Scoring geometry and stack lifecycle live in
shot_hit_harness.py.

One test per (lead, case) cell, printing a hit-rate line each, so a full
run is the before/after lead table. Every cell asserts that shots are
observed. The moving lead=ON cells assert a hit RATE
(shot_hit_harness.MOVING_MIN_HIT_RATE) -- CV aiming at a moving, spinning
target is what this bench is for, so that is its pass condition. The
lead=OFF cells stay measurement-only: they are the control leg, expected
to aim worse at speed. The stationary cells assert at least one hit
(STATIONARY_MIN_HITS) as a baseline sanity check, not as a difficulty.

Two tests outside the parameterization carry the real aiming pass
conditions, each launching its own stacks:
test_stationary_hit_rate_meets_floor (a hit RATE, not a count) and
test_lead_does_not_regress_hit_rate_at_slowest_speed (lead compared
against no-lead within one test). Both passed on 2026-09-17; the moving
lead=ON cells are red (CV_TEST_GAPS.md gap 8). Do not relax thresholds.

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


@pytest.fixture(autouse=True)
def settle_between_cases():
    """
    Let a case's stack fully release its topics/services before the next launches.

    Teardown rather than test body, so it still runs when a case fails
    and the next case starts against a clean graph either way.
    """
    yield
    time.sleep(1.0)


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

    assert sampler.shots_fired > 0, (
        f'no shots observed in {label} -- something in the launched stack is '
        'broken (mcb_relay not relaying, point_to_cv_target not firing, or a '
        f'node failed to start; check the per-node logs in {log_dir})')

    if case == STATIONARY:
        assert sampler.hits >= harness.STATIONARY_MIN_HITS, (
            f'{sampler.hits} hits out of {sampler.shots_fired} shots at a '
            f'motionless target ({label}) -- see STATIONARY_MIN_HITS')
    elif lead_enabled:
        # The bench's whole purpose: aiming at a moving, spinning target.
        # Only the lead=ON leg is held to it; lead=OFF is the control.
        hit_rate = sampler.hits / sampler.shots_fired
        assert hit_rate >= harness.MOVING_MIN_HIT_RATE, (
            f'{sampler.hits}/{sampler.shots_fired} = {hit_rate:.0%} hit rate '
            f'({label}), below {harness.MOVING_MIN_HIT_RATE:.0%} -- this is '
            'the moving-target aiming condition this suite exists to check; '
            'see MOVING_MIN_HIT_RATE')


def _run_case(request, gui, speed, spin_hz, lead_enabled, label):
    """Launch one stack, score it, and return the hit rate."""
    config = request.config
    duration = config.getoption('--shot-duration') or harness.DEFAULT_DURATION
    hit_radius = config.getoption('--hit-radius') or harness.DEFAULT_HIT_RADIUS
    log_dir = config.getoption('--log-dir') or harness.DEFAULT_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)

    print(f'\n=== {label} ===')
    sampler, dropped = harness.run_one_speed(
        speed, spin_hz, duration, not gui, log_dir, hit_radius,
        lead_enabled=lead_enabled)
    harness.summarize(label, sampler, dropped)
    assert sampler.shots_fired > 0, (
        f'no shots observed in {label} -- see the per-node logs in {log_dir}')
    return sampler.hits / sampler.shots_fired


def test_stationary_hit_rate_meets_floor(request, gui, ros_context):
    """
    Assert a hit RATE at a motionless target, not merely one hit.

    test_shot_hit's stationary cell asserts >= 1 hit, which a pipeline
    landing 1 shot in 200 at a target that isn't moving would pass.

    Measured 2026-09-17: 78.6% (11/14). Before the gap 7 fix it was 0/27
    with every shot missing by 0.884m. See
    shot_hit_harness.STATIONARY_MIN_HIT_RATE.
    """
    rate = _run_case(request, gui, 0.0, 0.0, False,
                     'stationary hit-rate floor')
    print(f'\nstationary hit rate {rate:.1%} vs floor '
          f'{harness.STATIONARY_MIN_HIT_RATE:.0%}')
    assert rate >= harness.STATIONARY_MIN_HIT_RATE, (
        f'{rate:.1%} hit rate at a motionless target, below '
        f'{harness.STATIONARY_MIN_HIT_RATE:.0%} -- the pipeline is aiming '
        'badly, not merely firing rarely; see STATIONARY_MIN_HIT_RATE')


def test_lead_does_not_regress_hit_rate_at_slowest_speed(request, gui, ros_context):
    """
    Give the lead feature a pass/fail, not just a printed number.

    The parameterized cells above run lead on and off in separate tests,
    so nothing compares them -- lead could regress from 60% to 5% and the
    suite stays green. This runs both legs itself at the slowest moving
    speed (highest hit rate, lowest run-to-run variance) and asserts the
    one-sided condition: enabling lead must not cost more than
    LEAD_REGRESSION_MARGIN. It launches two stacks of its own rather than
    caching the parameterized results, so it holds under -k, --lead, and
    any case selection.
    """
    speeds = _speeds(request.config)
    speed = min(speeds)
    spin_hz = harness.spin_hz_for_speed(speed, min(speeds), max(speeds))

    rates = {}
    for lead_enabled in (False, True):
        rates[lead_enabled] = _run_case(
            request, gui, speed, spin_hz, lead_enabled,
            f'lead comparison, speed={speed} m/s, spin={spin_hz:.2f} Hz | '
            f'lead={"on" if lead_enabled else "off"}')
        time.sleep(1.0)  # release the graph before the next launch

    lead_off, lead_on = rates[False], rates[True]
    print(f'\nlead off {lead_off:.1%} -> lead on {lead_on:.1%} '
          f'(margin {harness.LEAD_REGRESSION_MARGIN:.0%})')
    assert lead_on >= lead_off - harness.LEAD_REGRESSION_MARGIN, (
        f'lead ON hit {lead_on:.1%} vs lead OFF {lead_off:.1%} at '
        f'{speed} m/s -- enabling the intercept solve made aiming worse by '
        f'more than {harness.LEAD_REGRESSION_MARGIN:.0%}; see '
        'LEAD_REGRESSION_MARGIN')
