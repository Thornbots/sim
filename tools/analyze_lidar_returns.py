# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied.
# See the License for the specific language governing permissions and limitations
# under the License.
"""Summarize recorded raw scans without ROS or writing into bag directories."""
import json
import argparse
from pathlib import Path
import numpy as np
from mcap_ros2.reader import read_ros2_messages

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('bags', nargs='+', type=Path)
parser.add_argument('--topic', default='/scan_raw')
args = parser.parse_args()

for filename in args.bags:
    rows, intensities, stamps = [], [], []
    for item in read_ros2_messages(filename, topics=[args.topic]):
        m = item.ros_msg
        rows.append(m.ranges)
        intensities.append(m.intensities)
        stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    if not rows:
        print(json.dumps({'path': str(filename), 'error': 'no ' + args.topic}))
        continue
    a = np.asarray(rows)
    q = np.asarray(intensities)
    valid = np.isfinite(a)
    missing = ~valid
    fractions = missing.mean(axis=1)
    persistent = missing.mean(axis=0)
    isolated = missing & np.roll(valid, 1, axis=1) & np.roll(valid, -1, axis=1)
    same = valid[:, 1:] & valid[:, :-1] & (a[:, 1:] == a[:, :-1])
    edges = valid[:, 1:] & valid[:, :-1]
    result = {
        'path': str(Path(filename).resolve()), 'topic': args.topic,
        'scans': len(a), 'bins': a.shape[1], 'seconds': stamps[-1] - stamps[0],
        'missing_fraction_p05_median_p95': np.quantile(fractions, [.05, .5, .95]).tolist(),
        'bins_missing_over_95pct': int((persistent > .95).sum()),
        'bins_missing_under_5pct': int((persistent < .05).sum()),
        'isolated_missing_fraction_of_missing': float(isolated.sum() / missing.sum()),
        'adjacent_finite_equal_fraction': float(same.sum() / edges.sum()),
        'finite_range_p05_median_p95': np.quantile(a[valid], [.05, .5, .95]).tolist(),
        'missing_with_positive_quality': int((missing & (q > 0)).sum()),
        'finite_with_zero_quality_fraction': float((valid & (q == 0)).sum() / valid.sum()),
        'missing_by_30deg_sector': missing.mean(axis=0).reshape(12, -1).mean(axis=1).tolist(),
    }
    print(json.dumps(result), flush=True)
