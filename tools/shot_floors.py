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
Print shot_hit_harness.FLOORS from the aim bench's scores.jsonl over several runs.

`python3 tools/shot_floors.py RUN_DIR [RUN_DIR ...]`, each a --log-dir.
Floor per cell = lowest score - margin (CV_SPLIT_PLAN.md 1.7: three runs,
10 points). Cells seen in fewer runs than given are flagged.
"""
import argparse
import collections
import json
import os
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('runs', nargs='+', help='log dirs, one per run')
    parser.add_argument('--margin', type=float, default=0.10)
    args = parser.parse_args(argv)

    scores = collections.defaultdict(list)
    for run in args.runs:
        with open(os.path.join(run, 'scores.jsonl')) as f:
            for line in f:
                rec = json.loads(line)
                scores[rec['cell']].append(rec['score'])

    print('FLOORS = {')
    for cell in sorted(scores):
        seen = scores[cell]
        note = '' if len(seen) == len(args.runs) else f'  # {len(seen)} of {len(args.runs)} runs'
        spread = ', '.join(f'{s:.3f}' for s in seen)
        print(f"    '{cell}': {min(seen) - args.margin:.3f},  # {spread}{note}")
    print('}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
