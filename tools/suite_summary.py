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

"""Write the authoritative verdict for one guarded integration run."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import subprocess


ANSI = re.compile(r'\x1b\[[0-9;]*m')
PREFIX = re.compile(r'^\[[\w.-]+-\d+\] ')
START = re.compile(r'\[=+\] Running (\d+) tests? from \d+ test suites?\.')
FINISH = re.compile(r'\[=+\] (\d+) tests? from \d+ test suites? ran\.')
CASE = re.compile(r'\[\s*(RUN|OK|FAILED|SKIPPED)\s*\] (.+)')
FOOTER = re.compile(r'=== \d+ passed(?:, \d+ failed)? in [\d.]+s ===')
PHASES = ('approach', 'drive', 'spin', 'parked')
TELEMETRY = {
    'aim': 'scores.jsonl',
    'tracking': 'estimation.jsonl',
    'mcb_parked': 'scores.jsonl',
    'mcb_drive': 'segments.json',
    'mcb_match': 'match.json',
}


def write_json(path, data):
    """Publish complete JSON by replacing a temporary file in the same directory."""
    temporary = path.with_name('.' + path.name + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def native_results(lines, reasons):
    """Reconcile the GTest plan, individual outcomes and completion records."""
    planned, finished, cases = [], [], []
    footer = False
    for line in lines:
        if match := START.match(line):
            planned.append(int(match[1]))
        elif match := FINISH.match(line):
            finished.append(int(match[1]))
        elif match := CASE.fullmatch(line):
            status, detail = match.groups()
            if status == 'RUN' or re.search(r' \(\d+ ms\)$', detail):
                cases.append({'name': detail.split()[0].rstrip(','), 'status': status})
        elif FOOTER.fullmatch(line):
            footer = True
    totals = Counter(case['status'] for case in cases)
    started = Counter(case['name'] for case in cases if case['status'] == 'RUN')
    outcomes = [case for case in cases if case['status'] != 'RUN']
    ended = Counter(case['name'] for case in outcomes)
    expected = sum(planned) if planned else None
    executed = totals['OK'] + totals['FAILED']
    if not expected:
        reasons.append('GTest did not schedule any test cases')
    if planned != finished or started != ended or executed != expected or not footer:
        reasons.append(f'Incomplete native suite: executed {executed} of '
                       f'{expected if expected is not None else "unknown"} expected cases')
    if totals['FAILED'] or totals['SKIPPED']:
        reasons.append(f'Native cases: {totals["FAILED"]} failed, {totals["SKIPPED"]} skipped')
    return {
        'expected': expected, 'executed': executed, 'passed': totals['OK'],
        'failed': totals['FAILED'], 'skipped': totals['SKIPPED'],
        'cases': outcomes,
    }


def checker_result(checker, paths):
    """Run the native checker and preserve its reasons, notes and shutdown findings."""
    if not paths:
        return {'verdict': 'NOT_RUN', 'exit_code': None, 'reasons': [], 'logs': []}
    code = None
    try:
        result = subprocess.run([checker, '--json', *paths], capture_output=True, text=True)
        code = result.returncode
        report = json.loads(result.stdout)
        if (code, report['verdict']) not in ((0, 'PASS'), (1, 'FAIL')):
            raise ValueError(f'inconsistent checker exit code {code}')
        logs = report['logs']
        reasons = [f'{Path(log["path"]).name}: {reason}'
                   for log in logs for reason in log['reasons']]
        return {**report, 'reasons': reasons, 'exit_code': code}
    except (OSError, ValueError, KeyError, TypeError) as error:
        return {'verdict': 'FAIL', 'exit_code': code, 'logs': [],
                'reasons': [f'Could not read checker verdict: {error}']}


def localization_metrics(suite, lines):
    """Retain each drift scenario's native measurements and the EKF sample basis."""
    if suite == 'localization_drift':
        cells = {}
        for line in lines:
            if match := re.fullmatch(r'\s*\[(?:PASS|FAIL|SKIP)\] (\w+): (.+)', line):
                cells.setdefault(match[1], []).append(match[2])
        return {'cells': [{'cell': name, 'measurements': values} for name, values in cells.items()]}
    cell = {'cell': 'EkfGroundTruth.BeatsRawOdometry', 'measurements': []}
    for line in lines:
        if match := re.match(r'=== (\d+) samples, ', line):
            cell['samples'] = int(match[1])
        elif line.startswith(('raw /odom ', 'ekf fused ', 'EKF improvement ')):
            cell['measurements'].append(line)
    return {'cells': [cell] if 'samples' in cell else []}


def match_metrics(report, reasons):
    """Use the native scorer's phase reports and scope friendly contacts by shooter."""
    phases = {}
    for row in report['segments']:
        phase = row['segment']
        phases[phase] = {
            'sentry_scored_shots': row['shots'],
            'accuracy_attempts': row['accuracy_attempts'],
            'referee_credits': row['referee_credited_hits'],
            'valid_enemy_panel_contacts': row['valid_enemy_panel_contacts'],
            'valid_enemy_panel_contact_rate': row['valid_enemy_panel_contact_rate'],
            'localization_p95_m': row['localization_p95_m'],
            'diagnosis': row['diagnosis'],
        }
        if phase != 'approach' and row['accuracy_attempts'] == 0:
            reasons.append(f'{phase}: no accuracy attempts against living opponents')
    missing = [phase for phase in PHASES if phase not in phases]
    if missing:
        reasons.append('Missing match phases: ' + ', '.join(missing))
    return {
        'phases': phases,
        'all_robots_friendly_contacts': report['friendly_contacts'],
        'sentry_friendly_contacts': report['sentry_friendly_contacts'],
        'scored_shots_by_robot': report['shots_by_robot'],
    }


def metrics_for(suite, directory, lines, reasons, counts):
    """Read metrics from native aggregate outputs and scenario measurements."""
    if suite not in TELEMETRY:
        metrics = localization_metrics(suite, lines)
        if len(metrics['cells']) < counts['passed']:
            reasons.append('Launch log has fewer localization measurements than passed cases')
        return metrics
    path = directory / TELEMETRY[suite]
    try:
        if path.suffix == '.jsonl':
            rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
            if len(rows) < counts['passed']:
                reasons.append(f'{path.name} has fewer cell metrics than passed cases')
            return {'cells': rows}
        report = json.loads(path.read_text())
        if suite == 'mcb_match':
            return match_metrics(report, reasons)
        return {'phases': {row['segment']: {k: v for k, v in row.items() if k != 'segment'}
                           for row in report}}
    except (OSError, KeyError, ValueError, TypeError) as error:
        if counts['passed'] or path.exists():
            reasons.append(f'Could not read native metrics from {path.name}: {error}')
        return {}


def revisions(reasons):
    """Record exact build revisions when the build supplied a manifest."""
    path = os.environ.get('SIM_BUILD_REVISIONS')
    if not path:
        return {}
    try:
        data = json.loads(Path(path).read_text())
        if not isinstance(data, dict) or any(
                not isinstance(sha, str) or not re.fullmatch('[a-f0-9]{40}', sha)
                for sha in data.values()):
            raise ValueError('expected a map of package names to full git SHAs')
        return data
    except (OSError, ValueError) as error:
        reasons.append(f'Could not read build revisions: {error}')
        return {}


def summarize(directory, suite, launch, launch_status, interrupted, failure, checker, paths,
              capture_result=None):
    """Combine independent run evidence into one JSON verdict."""
    reasons = [failure] if failure else []
    git_shas = revisions(reasons)
    if interrupted:
        reasons.append('Run interrupted')
    counts = {'expected': None, 'executed': 0, 'passed': 0, 'failed': 0, 'skipped': 0, 'cases': []}
    metrics = {}
    group_reaped = False
    if launch_status is not None:
        if launch_status:
            reasons.append(f'Launch exited with status {launch_status}')
        try:
            log = (directory / 'launch.log').read_text(errors='replace')
            lines = [PREFIX.sub('', ANSI.sub('', line)) for line in log.splitlines()]
            counts = native_results(lines, reasons)
            metrics = metrics_for(suite, directory, lines, reasons, counts)
            group_reaped = 'E2E stack process group reaped' in lines
        except OSError as error:
            reasons.append(f'Could not read launch log: {error}')
    check = checker_result(checker, paths)
    # A completed E2E case and verified process-group cleanup do not depend on
    # how long individual nodes took to honor shutdown. Keep those diagnostics.
    reaped = {
        f'stack.log: {reason}' for log in check['logs'] if Path(log['path']).name == 'stack.log'
        for reason in log['reaped_teardown_reasons']
    } if suite.startswith('mcb_') and launch_status == 0 and group_reaped and not reasons else set()
    reasons.extend(reason for reason in check['reasons'] if reason not in reaped)
    teardown_reasons = [f'{Path(log["path"]).name}: {reason}'
                        for log in check['logs'] for reason in log['teardown_reasons']]
    clean = None
    if check['logs'] and all(log['readable'] for log in check['logs']):
        clean = not teardown_reasons
    if launch_status is not None and clean is not True:
        reasons.extend(reason for reason in
                       teardown_reasons or ['Teardown cleanliness could not be verified']
                       if reason not in reaped)
    if not check['reasons'] and check['verdict'] == 'FAIL':
        reasons.append('Native checker rejected the run')
    if launch_status is None and not reasons:
        reasons.append('Suite was not launched')
    capture = None
    if capture_result is not None:
        try:
            capture = json.loads(capture_result.read_text())
            if capture['status'] != 'COMPLETE':
                reasons.append(capture['reason'])
        except (OSError, ValueError, KeyError, TypeError):
            capture = {'status': 'FAILED', 'reason': 'Capture did not report completion'}
            reasons.append(capture['reason'])
    return {
        'schema_version': 1, 'suite': suite, 'launch_file': launch[0], 'launch_args': launch[1:],
        'git_shas': git_shas, 'test_cases': counts, 'launch_exit_code': launch_status,
        'interrupted': interrupted, 'checker': check,
        'teardown': {'clean': clean, 'reasons': teardown_reasons},
        'metrics': metrics, 'capture': capture,
        'verdict': 'FAIL' if reasons else 'PASS', 'reasons': list(dict.fromkeys(reasons)),
    }


def main():
    """Finish the run and return its verdict as the shell exit status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--suite', required=True)
    parser.add_argument('--launch-status', required=True)
    parser.add_argument('--interrupted', choices=('0', '1'), required=True)
    parser.add_argument('--failure', required=True)
    parser.add_argument('--checker', required=True)
    parser.add_argument('--log', action='append', default=[])
    parser.add_argument('--capture-result', type=Path)
    parser.add_argument('launch', nargs='+')
    args = parser.parse_args()
    summary = summarize(args.directory, args.suite, args.launch,
                        int(args.launch_status) if args.launch_status else None,
                        args.interrupted == '1', args.failure, args.checker, args.log,
                        args.capture_result)
    path = args.directory / 'summary.json'
    write_json(path, summary)
    print(f'{summary["verdict"]} {args.suite} {path}')
    return int(summary['verdict'] != 'PASS')


if __name__ == '__main__':
    raise SystemExit(main())
