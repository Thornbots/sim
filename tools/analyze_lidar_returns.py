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
import bisect
from pathlib import Path
import numpy as np
from mcap_ros2.reader import read_ros2_messages

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('bags', nargs='+', type=Path)
parser.add_argument('--topic', default='/scan_raw')
parser.add_argument('--joint-topic', default='/joint_states')
parser.add_argument('--head-joint', default='headlink')
parser.add_argument('--max-joint-age', type=float, default=0.05)
args = parser.parse_args()
if not np.isfinite(args.max_joint_age) or args.max_joint_age < 0:
    parser.error('--max-joint-age must be finite and nonnegative')


def stamp(message):
    """Use capture headers, independent of bag receive order/time."""
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def head_summary(filename, stamps, missing, sector):
    """Nearest-header join; unmatched scans never contribute to yaw groups."""
    joints = []
    for item in read_ros2_messages(filename, topics=[args.joint_topic]):
        message = item.ros_msg
        if args.head_joint in message.name:
            index = message.name.index(args.head_joint)
            if index < len(message.position) and np.isfinite(message.position[index]):
                joints.append((stamp(message), message.position[index]))
    joints.sort()
    times = [entry[0] for entry in joints]
    matches, yaws, ages = [], [], []
    for row, time in enumerate(stamps):
        insertion = bisect.bisect_left(times, time)
        candidates = [i for i in (insertion - 1, insertion) if 0 <= i < len(times)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda i: abs(times[i] - time))
        age = abs(times[nearest] - time)
        if age <= args.max_joint_age:
            matches.append(row)
            yaws.append(joints[nearest][1] % (2 * np.pi))
            ages.append(age)
    result = {'joint_samples': len(joints), 'matched_scans': len(matches),
              'unmatched_scans': len(stamps) - len(matches),
              'max_joint_age_s': args.max_joint_age, 'yaw_quadrants': []}
    if not matches:
        return result
    yaw = np.asarray(yaws)
    joined = missing[matches]
    result['match_age_s_p05_median_p95_max'] = np.quantile(ages, [.05, .5, .95, 1]).tolist()
    result['head_yaw_deg_min_max'] = np.degrees([yaw.min(), yaw.max()]).tolist()
    for quadrant in range(4):
        selected = (yaw >= quadrant * np.pi / 2) & (yaw < (quadrant + 1) * np.pi / 2)
        if selected.any():
            result['yaw_quadrants'].append({
                'from_deg': quadrant * 90, 'scans': int(selected.sum()),
                'median_missing_fraction': float(np.median(joined[selected].mean(axis=1))),
                'sensor_0_to_30deg_missing_fraction': (
                    float(joined[selected][:, sector].mean()) if sector.any() else None),
            })
    return result

for filename in args.bags:
    rows, intensities, stamps = [], [], []
    geometry = None
    for item in read_ros2_messages(filename, topics=[args.topic]):
        m = item.ros_msg
        current_geometry = (len(m.ranges), m.angle_min, m.angle_increment)
        if geometry is not None and geometry != current_geometry:
            raise ValueError(f'{filename}: scan angular geometry changed')
        geometry = current_geometry
        rows.append(m.ranges)
        intensities.append(m.intensities)
        stamps.append(stamp(m))
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
    angles = (geometry[1] + np.arange(a.shape[1]) * geometry[2]) % (2 * np.pi)
    sector = angles < np.pi / 6
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
        'missing_by_30deg_sector': [
            float(missing[:, (angles >= start) & (angles < start + np.pi / 6)].mean())
            if ((angles >= start) & (angles < start + np.pi / 6)).any() else None
            for start in np.arange(12) * np.pi / 6],
        'head_angle_control': head_summary(filename, stamps, missing, sector),
    }
    print(json.dumps(result), flush=True)
