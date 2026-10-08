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
Diff two E2E runs' logs record by record and print each file's first divergence.

`python3 tools/compare_runs.py LOG_DIR_A LOG_DIR_B`, each an e2e --log-dir.
Compares shots.jsonl, states.jsonl, poses.jsonl and route.jsonl where either
run has one. Exits 1 on any divergence; `--tol` allows float slack.
"""
import argparse
import json
import math
import os
import sys

FILES = ['shots.jsonl', 'states.jsonl', 'poses.jsonl', 'route.jsonl']


def load(path):
    if not os.path.exists(path):
        return None
    with open(path) as stream:
        return [json.loads(line) for line in stream if line.strip()]


def differ(a, b, tol, key=''):
    """List (key, a, b) for every leaf where a and b differ by more than tol."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            out += differ(a.get(k), b.get(k), tol, f'{key}.{k}' if key else k)
        return out
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += differ(x, y, tol, f'{key}[{i}]')
        return out
    numbers = (int, float)
    if (isinstance(a, numbers) and isinstance(b, numbers)
            and not isinstance(a, bool) and not isinstance(b, bool)):
        if a == b or (math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol):
            return []
        return [(key, a, b)]
    return [] if a == b else [(key, a, b)]


def compare(name, a, b, tol):
    """Print the first diverging record of one file; True if the runs agree."""
    if a is None or b is None:
        print(f'{name}: only in run {"A" if b is None else "B"}')
        return False
    for i, (x, y) in enumerate(zip(a, b)):
        diffs = differ(x, y, tol)
        if diffs:
            print(f'{name}: first divergence at record {i} (t={x.get("t")} / {y.get("t")})')
            for key, va, vb in diffs:
                print(f'  {key}: {va!r} != {vb!r}')
            return False
    if len(a) != len(b):
        print(f'{name}: identical for {min(len(a), len(b))} records, then A has {len(a)}, '
              f'B {len(b)}')
        return False
    print(f'{name}: identical, {len(a)} records')
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('run_a')
    parser.add_argument('run_b')
    parser.add_argument('--tol', type=float, default=0.0,
                        help='largest numeric difference counted as equal')
    args = parser.parse_args(argv)
    same = True
    for name in FILES:
        a = load(os.path.join(args.run_a, name))
        b = load(os.path.join(args.run_b, name))
        if a is None and b is None:
            continue
        same = compare(name, a, b, args.tol) and same
    return 0 if same else 1


if __name__ == '__main__':
    sys.exit(main())
