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
MCB emulator, parked: one opponent per cell, the real CV chain and firmware.

One test per cell, opponent speed by target_driver path, spin swept against
speed as on the aiming bench. Shots are the ones the compiled MCB fires
(/mcb_emulator/shot), falling under gravity; each cell asserts their hit
rate against FLOORS, or PLACEHOLDER_FLOOR for a cell not measured yet.
`ros2 launch sim e2e.launch.py` runs it; options: --e2e-speeds,
--e2e-paths, --e2e-spin, --e2e-duration, --headless, --log-dir,
--external-stack. See README.md "MCB emulator".
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
                             external=config.getoption('--external-stack'), stage='mcb_parked',
                             real_time_factor=config.getoption('--real-time-factor'),
                             firmware_fixes=not config.getoption('--no-firmware-fixes'))
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


def test_mcb_parked(cell, request, e2e_stack):
    speed, path, spin_hz = cell
    duration = request.config.getoption('--e2e-duration') or harness.DEFAULT_DURATION
    name = 'mcb_parked-' + harness.cell_id(speed, path, request.config.getoption('--e2e-spin'))
    print(f'\n=== {name}, spin {spin_hz:.2f} Hz ===')
    shots = harness.run_case(e2e_stack, speed, spin_hz, path, duration)
    harness.record_score(e2e_stack, name, shots, duration)
    mcb = shots['mcb']
    print(f'{name}: {sum(s["hit"] for s in mcb)}/{len(mcb)} MCB shots hit '
          f'({harness.hit_rate(mcb):.0%})')
    floor = harness.FLOORS.get(name, harness.PLACEHOLDER_FLOOR)
    assert mcb, f'{name}: the MCB emulator fired no shots; check stack.log in --log-dir'
    assert harness.hit_rate(mcb) >= floor, (
        f'{name}: hit rate {harness.hit_rate(mcb):.0%} below {floor:.0%}')
