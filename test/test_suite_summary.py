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

"""Exercise the guarded runner against excerpts from the 2026-11-21 demo logs."""
from importlib.util import module_from_spec, spec_from_file_location
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest


SIM = Path(__file__).resolve().parents[1]
FIXTURES = Path(__file__).with_name('fixtures') / 'suite_summary'
SPEC = spec_from_file_location('suite_summary', SIM / 'tools' / 'suite_summary.py')
SUMMARY = module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)


@pytest.fixture
def runner(tmp_path, monkeypatch):
    """Use the actual checker with a launch process that streams fixture evidence."""
    bin_dir = tmp_path / 'bin'
    native = tmp_path / 'lib' / 'sim'
    bin_dir.mkdir()
    native.mkdir(parents=True)
    (native / 'check_bench_log').symlink_to(Path(os.environ['SIM_CHECK_BENCH_LOG']).resolve())
    executable(native / 'guarded_exec', '#!/bin/bash\nexec "$@"\n')
    executable(bin_dir / 'gz', '#!/bin/bash\nexit 0\n')
    executable(bin_dir / 'ros2', """#!/bin/bash
if [ "$1" = pkg ]; then
    echo "$TEST_PREFIX"
else
    if [ -f "$TEST_PREFIX/match.json" ]; then
        cp "$TEST_PREFIX/match.json" "$TEST_PREFIX/run/match.json"
        touch "$TEST_PREFIX/run/stack.log"
    fi
    cat "$TEST_LAUNCH"
    if [ "${TEST_INTERRUPT:-0}" = 1 ]; then
        trap 'exit 0' INT
        touch "$TEST_PREFIX/launch-ready"
        for _ in {1..100}; do sleep .05; done
    fi
    exit "${TEST_LAUNCH_STATUS:-0}"
fi
""")
    monkeypatch.setenv('PATH', f'{bin_dir}:{os.environ["PATH"]}')
    monkeypatch.setenv('TEST_PREFIX', str(tmp_path))
    monkeypatch.delenv('SIM_BUILD_REVISIONS', raising=False)
    monkeypatch.delenv('SIM_SUITE_CAPTURE', raising=False)
    directory = tmp_path / 'run'
    directory.mkdir()

    def run(suite, fixture, launch_status=0, capture=None, interrupt=False):
        monkeypatch.setenv('TEST_LAUNCH', str(FIXTURES / (fixture + '.txt')))
        monkeypatch.setenv('TEST_LAUNCH_STATUS', str(launch_status))
        if capture is not None:
            monkeypatch.setenv('SIM_SUITE_CAPTURE', '1')
        if interrupt:
            monkeypatch.setenv('TEST_INTERRUPT', '1')
        command = [SIM / 'tools/run_suite.sh', suite, 'headless:=true', f'log_dir:={directory}']
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True) as process:
            if interrupt:
                await_file(directory.parent / 'launch-ready', process)
                process.send_signal(signal.SIGTERM)
            if capture is not None:
                await_file(directory / '.launch-complete', process)
                SUMMARY.write_json(directory / '.capture-result.json', capture)
            output = process.communicate(timeout=10)[0]
        summary = json.loads((directory / 'summary.json').read_text())
        assert process.returncode == (0 if summary['verdict'] == 'PASS' else 1)
        assert output.splitlines()[-1] == f'{summary["verdict"]} {suite} {directory}/summary.json'
        return summary

    return directory, run


def await_file(path, process):
    """Wait for the fixture to reach a known point in its lifecycle."""
    deadline = time.monotonic() + 10
    while not path.exists():
        assert process.poll() is None and time.monotonic() < deadline
        time.sleep(.01)


def executable(path, contents):
    """Write a fixture program with execute permission."""
    path.write_text(contents)
    path.chmod(0o755)


def test_tracking_passed_cases_do_not_override_checker(runner):
    """The 12 native successes retain the historical watchdog rejection."""
    _, run = runner
    report = run('tracking', 'tracking-fixed')
    assert report['verdict'] == 'FAIL'
    assert report['test_cases']['passed'] == 12
    assert report['checker']['verdict'] == 'FAIL'
    assert any('truth at 34.5 s' in reason for reason in report['reasons'])


@pytest.mark.parametrize(('launch_status', 'capture', 'reason'), [
    (2, None, 'Launch exited with status 2'),
    (0, {'status': 'FAILED', 'reason': 'Capture interrupted'}, 'Capture interrupted'),
])
def test_localization_passed_cases_do_not_override_wrapper(runner, launch_status, capture, reason):
    """A complete drift result remains a failed run when its wrapper fails."""
    _, run = runner
    result = run('localization_drift', 'localization_drift-fixed', launch_status, capture)
    assert result['test_cases']['passed'] == 10
    assert result['checker']['verdict'] == 'PASS'
    assert result['verdict'] == 'FAIL'
    assert reason in result['reasons']


def test_interrupt_after_all_cases_pass(runner):
    """A runner signal takes precedence over completed native cases."""
    _, run = runner
    result = run('localization_drift', 'localization_drift-fixed', interrupt=True)
    assert result['test_cases']['passed'] == 10
    assert result['checker']['verdict'] == 'PASS'
    assert result['interrupted'] is True
    assert result['verdict'] == 'FAIL'
    assert 'Run interrupted' in result['reasons']


def test_capture_publishes_pass_after_its_own_completion(runner):
    """The runner waits for the recorder before publishing the shared verdict."""
    _, run = runner
    capture = {'status': 'COMPLETE'}
    result = run('localization_drift', 'localization_drift-fixed', capture=capture)
    assert result['capture'] == capture
    assert result['verdict'] == 'PASS'


def test_ekf_teardown_escalation(runner):
    """Preserve the native success and the RViz teardown diagnosis."""
    _, run = runner
    result = run('ekf', 'ekf-fixed')
    assert result['test_cases']['passed'] == 1
    assert result['metrics']['cells'][0]['samples'] == 24
    assert result['teardown']['clean'] is False
    assert result['checker']['verdict'] == 'FAIL'
    assert any('SIGTERM' in reason for reason in result['teardown']['reasons'])


@pytest.mark.parametrize('scored', ['match', 'match-zero-attempts'])
def test_match_reports_native_scoring(runner, scored):
    """Retain official phase counts and ignore the extra impact rows."""
    directory, run = runner
    metrics_path = directory.parent / 'match.json'
    shutil.copyfile(FIXTURES / (scored + '.json'), metrics_path)
    shutil.copy(FIXTURES / 'match-shots.jsonl', directory / 'shots.jsonl')
    report = run('mcb_match', 'match')
    metrics, reasons = report['metrics'], report['reasons']
    phases = metrics['phases']
    if scored == 'match':
        assert reasons == []
        assert metrics['all_robots_friendly_contacts'] == 3
        assert metrics['sentry_friendly_contacts'] == 0
        assert phases['spin']['accuracy_attempts'] == 36
        assert phases['spin']['referee_credits'] == 31
        assert phases['spin']['valid_enemy_panel_contacts'] == 31
        assert sum(row['sentry_scored_shots'] for row in phases.values()) == 52
        assert len((directory / 'shots.jsonl').read_text().splitlines()) > 52
        assert report['verdict'] == 'PASS'
    else:
        assert phases['parked']['accuracy_attempts'] == 0
        assert 'parked: no accuracy attempts against living opponents' in reasons
        assert 'Missing match phases: drive' in reasons
        assert report['verdict'] == 'FAIL'


def test_aborted_and_skipped_cells(runner):
    """An aborted or skipped selection falls short of the planned count."""
    _, run = runner
    report = run('localization_drift', 'localization-aborted', 1)
    assert report['verdict'] == 'FAIL'
    assert report['test_cases']['passed'] == 0
    assert report['test_cases']['failed'] == 1

    lines = (FIXTURES / 'localization_drift-fixed.txt').read_text().splitlines()
    lines = [SUMMARY.PREFIX.sub('', line) for line in lines]
    for index, line in enumerate(lines):
        if '[       OK ]' in line:
            lines[index] = line.replace('[       OK ]', '[  SKIPPED ]')
            break
    reasons = []
    counts = SUMMARY.native_results(lines, reasons)
    assert (counts['expected'], counts['executed'], counts['passed'], counts['skipped']) == (
        10, 9, 9, 1)
    assert 'Incomplete native suite: executed 9 of 10 expected cases' in reasons
