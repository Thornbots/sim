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
Print estimation_harness.LIMITS from the C2 bench's estimation.jsonl over several runs.

`python3 tools/estimation_limits.py RUN_DIR [RUN_DIR ...]`, each a log_dir:=.
Limit per cell and metric = worst p95 x (1 + margin), margin 0.25 by default
(CV_SPLIT_PLAN.md 2.0: three runs plus a margin). Cells seen in fewer runs
than given are flagged.
"""
import argparse
import collections
import json
import os
import sys

# estimation_metrics.METRICS
METRICS = ('facing_panel_m', 'panel_m', 'center_m', 'velocity_m_s', 'yaw_rad',
           'yaw_rate_rad_s', 'radius_m', 'z_offset_m')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('runs', nargs='+', help='log dirs, one per run')
    parser.add_argument('--margin', type=float, default=0.25)
    args = parser.parse_args(argv)

    p95 = collections.defaultdict(lambda: collections.defaultdict(list))
    seen = collections.Counter()
    for run in args.runs:
        with open(os.path.join(run, 'estimation.jsonl')) as f:
            for line in f:
                rec = json.loads(line)
                seen[rec['cell']] += 1
                for key in METRICS:
                    if rec.get(key):
                        p95[rec['cell']][key].append(rec[key]['p95'])

    print('LIMITS = {')
    for cell in sorted(p95):
        note = '' if seen[cell] == len(args.runs) else f'  # {seen[cell]} of {len(args.runs)} runs'
        print(f"    '{cell}': {{{note}")
        for key, values in p95[cell].items():
            print(f"        '{key}': {max(values) * (1.0 + args.margin):.4f},"
                  f"  # {', '.join(f'{v:.4f}' for v in values)}")
        print('    },')
    print('}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
