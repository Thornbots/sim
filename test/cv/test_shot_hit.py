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
lead on and firing at up to TEST_FIRE_HZ (40), far above the real launcher, to
stress tracking. The stack launches once per run (the cv_stack fixture); each
case only changes how the target moves. Every case runs twice: flat panels,
then staggered (neighbours STAGGERED_PANEL_M, 90% of a panel's height,
apart); --panel-layout picks one. Each case prints and asserts a score,
the mean of hit rate and hits per expected shot (shot_hit_harness.score), so
falling behind 40 Hz costs points. Floors are per cell (shot_hit_harness.FLOORS,
three runs' lowest minus 10 points), falling back to STATIONARY_MIN_HIT_RATE or
MOVING_MIN_HIT_RATE for a cell not measured yet. Do not relax them.

The C1 aim bench: no gz, a point shooter with a perfect gimbal and a
perfectly known target (see shot_hit_harness.py). Launches a ROS stack, so
marked `integration` and skipped by a plain `colcon test`; `ros2 launch sim
shot_hit.launch.py` runs it. Options: --shot-speeds, --shot-duration,
--hit-radius, --panel-layout, --skip-stationary, --only-stationary,
--headless, --log-dir, --external-stack, --target-path
(lateral/radial/diagonal), --shooter-speed.
"""
import os

import pytest

import shot_hit_harness as harness

pytestmark = pytest.mark.integration

STATIONARY = 'stationary'


@pytest.fixture(scope='module')
def cv_stack(request, ros_context):
    """Launch the aim bench's stack once for every case in this module."""
    config = request.config
    log_dir = config.getoption('--log-dir') or harness.DEFAULT_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)
    stack = harness.CvStack(config.getoption('--headless'), log_dir,
                            external=config.getoption('--external-stack'),
                            shooter_speed=config.getoption('--shooter-speed'))
    try:
        stack.start()
        yield stack
    finally:
        stack.stop()


def _speeds(config):
    raw = config.getoption('--shot-speeds')
    if not raw:
        return harness.DEFAULT_SPEEDS
    return [float(v) for v in raw.split(',') if v.strip()]


LAYOUTS = {'flat': 0.0, 'staggered': harness.STAGGERED_PANEL_M}


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


def test_shot_hit(layout, case, request, cv_stack):
    config = request.config
    speeds = _speeds(config)
    duration = config.getoption('--shot-duration') or harness.DEFAULT_DURATION
    hit_radius = config.getoption('--hit-radius') or harness.DEFAULT_HIT_RADIUS
    path = config.getoption('--target-path')
    shooter_speed = config.getoption('--shooter-speed')
    motion = f', {path} path'
    if shooter_speed > 0.0:
        motion += f', shooter {shooter_speed} m/s'

    if case == STATIONARY:
        speed, spin_hz = 0.0, 0.0
        label, floor, floor_name = (f'{layout} stationary, spin=0.00 Hz{motion}',
                                    harness.STATIONARY_MIN_HIT_RATE,
                                    'STATIONARY_MIN_HIT_RATE')
    else:
        speed = case
        spin_hz = harness.spin_hz_for_speed(speed, min(speeds), max(speeds))
        label, floor, floor_name = (f'{layout} {speed} m/s, spin={spin_hz:.2f} Hz{motion}',
                                    harness.MOVING_MIN_HIT_RATE,
                                    'MOVING_MIN_HIT_RATE')
    print(f'\n=== {label} ===')

    sampler, dropped = harness.run_case(cv_stack, speed, spin_hz, duration, hit_radius,
                                        stagger=LAYOUTS[layout], path=path)
    total = harness.summarize(label, sampler, dropped, duration)
    cell = harness.cell_id(layout, speed, path, shooter_speed)
    harness.record_score(cv_stack, cell, sampler, duration)
    if cell in harness.FLOORS:
        floor, floor_name = harness.FLOORS[cell], f'FLOORS[{cell!r}]'
    else:
        print(f'{cell}: no measured floor yet, using {floor_name}')

    assert sampler.shots_fired > 0, (
        f'no shots observed in {label} -- something in the launched stack is '
        'broken (mcb_relay not relaying, point_to_cv_target not firing, or a '
        'node failed to start; check stack.log in --log-dir, or the launch log '
        'under ~/.ros/log when run from shot_hit.launch.py)')
    assert total >= floor, (
        f'score {total:.0%} ({label}: {sampler.hits} hits from '
        f'{sampler.shots_fired} shots), below {floor:.0%}; see {floor_name} '
        'and shot_hit_harness.score()')
