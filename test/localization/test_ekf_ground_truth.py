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
Asserts that EKF fusion of /scan_odom into /odom actually beats raw
/odom, scored against /sim/raw_odom ground truth -- the question the
drift suite structurally can't answer (see README.md). Runs the stack at
backend='none' with use_ekf=True, drives the same cornering loop the
drift scenarios use, and compares mean position error.

Marked `integration` (launches gz-sim), so a plain `colcon test` skips
it. Options: --headless, --ekf-slip-ratio, --ekf-drift-stddev,
--ekf-seconds.
"""
import ekf_diag_harness
import pytest

pytestmark = pytest.mark.integration


def test_ekf_beats_raw_odom(request, gui, ros_context):
    slip_ratio = request.config.getoption('--ekf-slip-ratio')
    drift_stddev = request.config.getoption('--ekf-drift-stddev')
    seconds = request.config.getoption('--ekf-seconds')

    result = ekf_diag_harness.run(gui, slip_ratio, drift_stddev, seconds)
    assert result is not None, \
        'run produced no usable samples -- see the printed FAIL line above'

    odom_stats, ekf_stats, n = result
    ekf_diag_harness.report(odom_stats, ekf_stats, n, slip_ratio, drift_stddev)
    improvement = ekf_diag_harness.improvement_pct(odom_stats, ekf_stats)
    assert improvement > 0.0, (
        f'EKF did not beat raw /odom over {n} samples: mean error '
        f'{ekf_stats["mean"]:.4f} m fused vs {odom_stats["mean"]:.4f} m raw '
        f'({improvement:+.1f}%)')
