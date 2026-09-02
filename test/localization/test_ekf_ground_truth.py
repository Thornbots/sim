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
import pytest

import ekf_diag_harness

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
