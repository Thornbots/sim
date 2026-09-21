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

Consumes only the final CVTarget on /dji_serial_bridge/cv_target, whose
fire/delay_ms carry the fire decision (the topic that would reach the
real launcher per mcb_relay.py's "sole relay" design). Scoring geometry
and stack lifecycle live in shot_hit_harness.py.

One test per case, stationary then each speed, all with point_to_cv_target's
lead on, printing a hit-rate line each. The stationary case asserts
STATIONARY_MIN_HIT_RATE, a floor that catches gross aiming breakage. The
moving cases assert MOVING_MIN_HIT_RATE, since aiming at a moving, spinning
target is what this bench is for. Do not relax thresholds.

Launches gz-sim, so marked `integration` and skipped by a plain
`colcon test`. Options: --shot-speeds, --shot-duration, --hit-radius,
--skip-stationary, --only-stationary, --headless, --log-dir.
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
    cases = []
    if not config.getoption('--skip-stationary'):
        cases.append(STATIONARY)
    if not config.getoption('--only-stationary'):
        cases.extend(_speeds(config))
    ids = [case if case == STATIONARY else f'speed{case}' for case in cases]
    metafunc.parametrize('case', cases, ids=ids)


def test_shot_hit(case, request, gui, ros_context):
    config = request.config
    speeds = _speeds(config)
    duration = config.getoption('--shot-duration') or harness.DEFAULT_DURATION
    hit_radius = config.getoption('--hit-radius') or harness.DEFAULT_HIT_RADIUS
    log_dir = config.getoption('--log-dir') or harness.DEFAULT_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)

    if case == STATIONARY:
        speed, spin_hz = 0.0, 0.0
        label, floor, floor_name = ('stationary, spin=0.00 Hz',
                                    harness.STATIONARY_MIN_HIT_RATE,
                                    'STATIONARY_MIN_HIT_RATE')
    else:
        speed = case
        spin_hz = harness.spin_hz_for_speed(speed, min(speeds), max(speeds))
        label, floor, floor_name = (f'speed={speed} m/s, spin={spin_hz:.2f} Hz',
                                    harness.MOVING_MIN_HIT_RATE,
                                    'MOVING_MIN_HIT_RATE')
    print(f'\n=== {label} ===')

    sampler, dropped = harness.run_one_speed(
        speed, spin_hz, duration, not gui, log_dir, hit_radius)
    harness.summarize(label, sampler, dropped)

    assert sampler.shots_fired > 0, (
        f'no shots observed in {label} -- something in the launched stack is '
        'broken (mcb_relay not relaying, point_to_cv_target not firing, or a '
        f'node failed to start; check the per-node logs in {log_dir})')
    hit_rate = sampler.hits / sampler.shots_fired
    assert hit_rate >= floor, (
        f'{sampler.hits}/{sampler.shots_fired} = {hit_rate:.0%} hit rate '
        f'({label}), below {floor:.0%}; see {floor_name}')
