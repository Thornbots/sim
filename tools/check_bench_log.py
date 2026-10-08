#!/usr/bin/env python3
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
Say whether a bench run can be trusted, from its launch logs.

`python3 tools/check_bench_log.py LOG [LOG ...]`: the suite's launch log,
plus the drift suite's /tmp/localization_drift_tests/*.log. Prints the
result line, the suite timing table and every problem found; exits 1 if a
node crashed mid-run, a GUI couldn't open, or a wait gave up.
"""
import argparse
import re
import sys

DIED = re.compile(r'\[(ERROR|INFO)\] \[([\w.-]+)\]: process has died .*exit code (-?\d+)')
SIGINT = re.compile(r"sending signal 'SIGINT' to process\[([\w.-]+)\]")
SHUTDOWN = re.compile(r'user interrupted with ctrl-c|tests exited with code')
# (label, pattern, fatal): each a sign the run's numbers aren't the stack's.
PATTERNS = [
    ('no display', re.compile(r'Qt platform plugin|could not connect to display'), True),
    ('wait gave up', re.compile(r'wall-clock cap hit|\[wait_until\] timed out'), True),
    ('stack not ready', re.compile(r'stack NOT ready'), True),
    ('lockstep timeout', re.compile(r'[1-9]\d* lockstep timeouts'), True),
    ('clock failure', re.compile(r'clock stalled|clock moved backwards'), True),
    ('pacing gate dropped', re.compile(r'silent, no longer pacing'), False),
    ('ODE contact overflow', re.compile(r'hash table bucket overflow'), False),
]
RESULT = re.compile(r'=+ .*\b(passed|failed|errors?)\b.* in [\d.]+s')
PREFIX = re.compile(r'^\[([\w.-]+)\] ')


def check(path):
    """Return (fatal, notes, report) for one log."""
    with open(path, errors='replace') as f:
        lines = f.read().splitlines()
    fatal, notes, report = [], [], []
    interrupted = set()
    reported = set()  # processes that printed pytest's result: their exit is the verdict
    shutting_down = in_timing = False
    for line in lines:
        text = re.sub(r'\x1b\[[0-9;]*m', '', line)
        body = re.sub(r'^\[[\w.-]+\] ', '', text)  # launch's per-process prefix
        if 'suite timing' in body:
            in_timing = True
        if RESULT.search(body):
            if m := PREFIX.match(text):
                reported.add(m.group(1))
            if re.search(r'\b[1-9]\d* (failed|errors?)\b', body):
                fatal.append(f'pytest failed: {body.strip()}')
        if in_timing:
            report.append(body)
            if RESULT.search(body):
                in_timing = False
        elif RESULT.search(body):
            report.append(body)
        if SHUTDOWN.search(text):
            shutting_down = True
        if m := SIGINT.search(text):
            interrupted.add(m.group(1))
        elif m := DIED.search(text):
            proc, code = m.group(2), int(m.group(3))
            if proc in reported:
                pass  # failed tests exit 1; the result line above already says so
            elif not shutting_down and proc not in interrupted:
                fatal.append(f'crashed mid-run: {proc} (exit {code})')
            elif code not in (-2, -15, 0):
                notes.append(f'unclean shutdown: {proc} (exit {code})')
        for label, pattern, is_fatal in PATTERNS:
            if pattern.search(text):
                # Notes count per kind; a fatal line keeps its text.
                (fatal if is_fatal else notes).append(
                    f'{label}: {body.strip()[:160]}' if is_fatal else label)
    return fatal, notes, report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('logs', nargs='+')
    args = parser.parse_args(argv)
    bad = False
    for path in args.logs:
        fatal, notes, report = check(path)
        print(f'== {path}')
        for line in report:
            print(f'  {line}')
        for line in dict.fromkeys(fatal):
            print(f'  FATAL {line}')
        for line, n in {line: notes.count(line) for line in notes}.items():
            print(f'  note  {line}' + (f' (x{n})' if n > 1 else ''))
        bad = bad or bool(fatal)
    print('BENCH NOT TRUSTWORTHY' if bad else 'bench ok')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
