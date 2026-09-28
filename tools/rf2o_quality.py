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
Record and summarise rf2o's match grades, to set config/rf2o.yaml's thresholds.

`record OUT.jsonl`: append every /scan_odom/quality status (one per scan)
until Ctrl-C; run it beside a suite. `summary FILE [FILE ...]`: tier and
reason counts, and percentiles of each signal, per file.
"""
import argparse
import json
import sys

SIGNALS = ('valid_fraction', 'sigma_max_m', 'sigma_min_m', 'speed_mps',
           'scan_gap_periods', 'levels_solved', 'extrinsic_age_s')
PCTS = (50, 90, 99, 99.9, 100)


def record(path):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('rf2o_quality_recorder')
    out = open(path, 'a', buffering=1)

    def on_quality(msg):
        for st in msg.status:
            row = {kv.key: kv.value for kv in st.values}
            row['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            row['message'] = st.message
            out.write(json.dumps(row) + '\n')

    node.create_subscription(DiagnosticArray, '/scan_odom/quality', on_quality,
                             qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()


def _pct(values, p):
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]


def summary(paths):
    for path in paths:
        rows = [json.loads(line) for line in open(path) if line.strip()]
        print(f'{path}: {len(rows)} matches')
        if not rows:
            continue
        tiers, reasons = {}, {}
        for r in rows:
            tiers[r['tier']] = tiers.get(r['tier'], 0) + 1
            msg = r['message']
            if '(' in msg:
                for why in msg[msg.index('(') + 1:msg.rindex(')')].split(','):
                    reasons[why] = reasons.get(why, 0) + 1
        for tier, n in sorted(tiers.items()):
            print(f'  {tier:<10} {n:6d}  {100 * n / len(rows):6.2f}%')
        for why, n in sorted(reasons.items(), key=lambda kv: -kv[1]):
            print(f'    {why:<20} {n:6d}  {100 * n / len(rows):6.2f}%')
        print('  signal            ' + ''.join(f'{"p" + str(p):>10}' for p in PCTS)
              + f'{"min":>10}')
        for key in SIGNALS:
            vals = [float(r[key]) for r in rows if key in r]
            if vals:
                print(f'  {key:<18}' + ''.join(f'{_pct(vals, p):10.4f}' for p in PCTS)
                      + f'{min(vals):10.4f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('record').add_argument('out')
    sub.add_parser('summary').add_argument('files', nargs='+')
    args = parser.parse_args()
    if args.cmd == 'record':
        record(args.out)
    else:
        summary(args.files)


if __name__ == '__main__':
    sys.exit(main())
