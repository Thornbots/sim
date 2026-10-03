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
Match test, stage E2: E1's cells through the wire, the MCB emulator firing.

`ros2 launch sim e2e.launch.py stage:=e2` runs it. Shots are the ones the
firmware port fires (/mcb_emulator/shot), scored as E1's are. Expected to
fail until our odom and the MCB's odometry agree: the frames get through
(with firmware_fixes), but the gun turns away (README.md "MCB emulator").
"""
import os

import e2e_harness as harness
import pytest
import test_e1

pytestmark = pytest.mark.integration
FRAME_GAP = ('POSE is x right, y forward and pose_translator reads it as REP-105, '
             'so the MCB aims our odom points in a turned frame')


@pytest.fixture(scope='module')
def e2e_stack(request, ros_context):
    config = request.config
    log_dir = config.getoption('--log-dir') or harness.DEFAULT_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)
    stack = harness.E2EStack(config.getoption('--headless'), log_dir,
                             external=config.getoption('--external-stack'), stage='e2',
                             firmware_fixes=not config.getoption('--no-firmware-fixes'))
    try:
        stack.start()
        yield stack
    finally:
        stack.stop()


pytest_generate_tests = test_e1.pytest_generate_tests


@pytest.mark.xfail(strict=True, reason=FRAME_GAP)
def test_e2(cell, request, e2e_stack):
    speed, path, spin_hz = cell
    duration = request.config.getoption('--e2e-duration') or harness.DEFAULT_DURATION
    name = 'e2-' + harness.cell_id(speed, path, request.config.getoption('--e2e-spin'))
    print(f'\n=== {name}, spin {spin_hz:.2f} Hz ===')
    shots = harness.run_case(e2e_stack, speed, spin_hz, path, duration)
    harness.record_score(e2e_stack, name, shots, duration)
    mcb = shots['mcb']
    print(f'{name}: {sum(s["hit"] for s in mcb)}/{len(mcb)} MCB shots hit '
          f'({harness.hit_rate(mcb):.0%})')
    floor = harness.FLOORS.get(name, harness.PLACEHOLDER_FLOOR)
    assert mcb, f'{name}: the MCB emulator fired no shots'
    assert harness.hit_rate(mcb) >= floor, (
        f'{name}: hit rate {harness.hit_rate(mcb):.0%} below {floor:.0%}')
