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
Match test, stage E1: our robot parked, one opponent, the real CV chain.

One test per cell, opponent speed by target_driver path, spin swept against
speed as on the aiming bench. Each asserts the hit rate of the firmware-rule
shots (e2e_harness) against FLOORS, or PLACEHOLDER_FLOOR for a cell not
measured yet. `ros2 launch sim e2e.launch.py` runs it; options:
--e2e-speeds, --e2e-paths, --e2e-spin, --e2e-duration, --headless, --log-dir,
--external-stack.
"""
import os

import e2e_harness as harness
import pytest
import shot_hit_harness as bench

pytestmark = pytest.mark.integration


@pytest.fixture(scope='module')
def e2e_stack(request, ros_context):
    config = request.config
    log_dir = config.getoption('--log-dir') or harness.DEFAULT_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)
    stack = harness.E2EStack(config.getoption('--headless'), log_dir,
                             external=config.getoption('--external-stack'))
    try:
        stack.start()
        yield stack
    finally:
        stack.stop()


def _list(config, option, default, cast):
    raw = config.getoption(option)
    return default if not raw else [cast(v) for v in raw.split(',') if v.strip()]


def pytest_generate_tests(metafunc):
    if 'cell' not in metafunc.fixturenames:
        return
    config = metafunc.config
    speeds = _list(config, '--e2e-speeds', harness.DEFAULT_SPEEDS, float)
    paths = _list(config, '--e2e-paths', harness.DEFAULT_PATHS, str)
    moving = [s for s in speeds if s > 0.0] or [1.0]
    spin = config.getoption('--e2e-spin')
    cells = [(speed, path, spin if spin is not None else 0.0 if speed == 0.0 else
              bench.spin_hz_for_speed(speed, min(moving), max(moving)))
             for path in paths for speed in speeds]
    metafunc.parametrize('cell', cells, ids=[harness.cell_id(s, p, spin) for s, p, _ in cells])


def test_e1(cell, request, e2e_stack):
    speed, path, spin_hz = cell
    duration = request.config.getoption('--e2e-duration') or harness.DEFAULT_DURATION
    name = harness.cell_id(speed, path, request.config.getoption('--e2e-spin'))
    print(f'\n=== {name}, spin {spin_hz:.2f} Hz ===')
    shots = harness.run_case(e2e_stack, speed, spin_hz, path, duration)
    harness.record_score(e2e_stack, name, shots, duration)
    rate, flag = shots['rate'], shots['flag']
    print(f'{name}: {sum(s["hit"] for s in rate)}/{len(rate)} firmware-rule shots hit '
          f'({harness.hit_rate(rate):.0%}), {sum(s["hit"] for s in flag)}/{len(flag)} '
          f'fire-flag shots ({harness.hit_rate(flag):.0%})')
    floor = harness.FLOORS.get(name)
    if floor is None:
        floor = harness.PLACEHOLDER_FLOOR
        print(f'{name}: no measured floor yet, using PLACEHOLDER_FLOOR')
    assert rate, (f'{name}: no shots; /cv/target never held confidence >= '
                  f'{harness.FIRE_MIN_CONFIDENCE}. Check stack.log in --log-dir')
    assert harness.hit_rate(rate) >= floor, (
        f'{name}: hit rate {harness.hit_rate(rate):.0%} below {floor:.0%}')
