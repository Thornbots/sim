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

"""Exercise real launch exit propagation and misleading benchmark log summaries."""
import importlib.util
from pathlib import Path
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import pytest
from sim.suite_exit import finish_suite

spec = importlib.util.spec_from_file_location(
    'check_bench_log', Path(__file__).parents[1] / 'tools/check_bench_log.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


@pytest.mark.parametrize('code', [0, 1, 2])
def test_child_result_propagates_through_launch_service(code):
    child = ExecuteProcess(cmd=[sys.executable, '-c', f'raise SystemExit({code})'])
    handler = RegisterEventHandler(OnProcessExit(
        target_action=child,
        on_exit=lambda event, context: finish_suite(event, context, 'probe')))
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([handler, child]))
    assert service.run() == (0 if code == 0 else 1)


@pytest.mark.parametrize('text,failed', [
    ('================ 12 passed in 1.0s ================', False),
    ('================ 1 failed, 11 passed in 1.0s ================', True),
    ('================ 1 error in 1.0s ================', True),
    ('lockstep: 0 lockstep timeouts', False),
    ('lockstep: 2 lockstep timeouts', True),
    ('E2E clock stalled during scoring', True),
    ('lockstep: /cv/target/state_ack timed out after 0.50 s wall', True),
])
def test_checker_rejects_failed_tests_and_stalls(tmp_path, text, failed):
    log = tmp_path / 'suite.log'
    log.write_text('[tests-1] ==== suite timing ====\n[tests-1] ' + text + '\n')
    fatal, _, _ = checker.check(log)
    assert bool(fatal) == failed
