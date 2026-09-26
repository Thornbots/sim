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
C2 estimation bench: target_tracker's TargetState against the truth, no gz.

The same cells as the aim bench (test_shot_hit.py): stationary, then each
speed, flat and staggered. Each case restarts the track and scores every
state at its own stamp (estimation_harness.py). A cell asserts that states
arrive and mostly go valid, plus its p95 limits once LIMITS has them (the
worst of three runs plus a margin, tools/estimation_limits.py). Nothing fires.
Launches the whole stack, so marked `integration`; `ros2 launch sim estimation.launch.py`
runs it. Options: --shot-speeds, --shot-duration, --panel-layout,
--target-path, --shooter-speed, --blackout, --camera-latency,
--skip-stationary, --only-stationary, --headless, --log-dir, --external-stack.
"""
import os

import estimation_harness as harness
from estimation_metrics import cell_id
import pytest
import shot_hit_harness

pytestmark = pytest.mark.integration

STATIONARY = 'stationary'
LAYOUTS = {'flat': 0.0, 'staggered': shot_hit_harness.STAGGERED_PANEL_M}
MIN_VALID_FRACTION = 0.5  # liveness, not a tuned figure


@pytest.fixture(scope='module')
def est_stack(request, ros_context):
    """Launch the C2 stack once for every case in this module."""
    config = request.config
    log_dir = config.getoption('--log-dir') or harness.DEFAULT_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)
    stack = harness.EstimationStack(config.getoption('--headless'), log_dir,
                                    external=config.getoption('--external-stack'))
    try:
        stack.start()
        yield stack
    finally:
        stack.stop()


def _speeds(config):
    raw = config.getoption('--shot-speeds')
    if not raw:
        return shot_hit_harness.DEFAULT_SPEEDS
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
    layout = config.getoption('--panel-layout')
    layouts = list(LAYOUTS) if layout == 'both' else [layout]
    params = [(lay, case) for lay in layouts for case in cases]
    ids = [f'{lay}-{case if case == STATIONARY else f"speed{case}"}' for lay, case in params]
    metafunc.parametrize('layout,case', params, ids=ids)


def test_estimation(layout, case, request, est_stack):
    config = request.config
    speeds = _speeds(config)
    duration = config.getoption('--shot-duration') or harness.DEFAULT_DURATION
    path = config.getoption('--target-path')
    shooter_speed = config.getoption('--shooter-speed')
    blackout = config.getoption('--blackout')
    if case == STATIONARY:
        speed, spin_hz = 0.0, 0.0
    else:
        speed = case
        spin_hz = shot_hit_harness.spin_hz_for_speed(speed, min(speeds), max(speeds))
    cell = cell_id(layout, speed, path, shooter_speed, blackout,
                   config.getoption('--camera-latency'))
    print(f'\n=== {cell}, spin={spin_hz:.2f} Hz ===')

    summary = harness.run_case(est_stack, cell, speed, spin_hz, duration,
                               stagger=LAYOUTS[layout], path=path, blackout=blackout,
                               shooter_speed=shooter_speed)
    harness.print_summary(cell, summary)

    assert summary['states'] > 0, (
        f'no TargetState scored in {cell}: check target_tracker and the emulator '
        '(stack.log in --log-dir, or the launch log)')
    assert summary['valid_fraction'] >= MIN_VALID_FRACTION, (
        f'{cell}: only {summary["valid_fraction"]:.0%} of states valid')
    limits = harness.LIMITS.get(cell)
    if limits is None:
        print(f'{cell}: no limits yet, reporting only')
        return
    over = {k: (summary[k]['p95'], lim) for k, lim in limits.items()
            if summary.get(k) and summary[k]['p95'] > lim}
    assert not over, f'{cell}: p95 over LIMITS (value, limit): {over}'
