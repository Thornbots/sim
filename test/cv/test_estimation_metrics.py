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

"""Pin estimation_metrics.py, C2's scoring math. Plain pytest, no ROS."""
import math
from types import SimpleNamespace

from estimation_metrics import cell_id, METRICS, state_errors, summarize_case
import numpy as np

RADII = (0.30, 0.24)
STAGGER = 0.09
TRUTH = (np.array([3.0, 0.5, 0.3]), np.array([0.0, 2.0, 0.0]), 0.4, 9.0)
VIEWER = (0.0, 0.0)


def _state(center=(3.0, 0.5, 0.3), vel=(0.0, 2.0, 0.0), yaw=0.4, w=9.0,
           radius=RADII, z_offset=(STAGGER / 2.0, -STAGGER / 2.0)):
    xyz = SimpleNamespace
    return SimpleNamespace(center=xyz(x=center[0], y=center[1], z=center[2]),
                           velocity=xyz(x=vel[0], y=vel[1], z=vel[2]),
                           yaw=yaw, yaw_rate=w, radius=radius, z_offset=z_offset)


def test_the_truth_scores_zero():
    err = state_errors(_state(), TRUTH, STAGGER, RADII, VIEWER)
    assert all(v < 1e-9 for v in err.values())


def test_tracking_another_panel_is_not_an_error():
    # A quarter turn on: that panel's pair is the other one, radius and height.
    state = _state(yaw=0.4 + math.pi / 2.0, radius=RADII[::-1],
                   z_offset=(-STAGGER / 2.0, STAGGER / 2.0))
    err = state_errors(state, TRUTH, STAGGER, RADII, VIEWER)
    assert all(v < 1e-9 for v in err.values())
    state = _state(yaw=0.4 - math.pi, z_offset=(STAGGER / 2.0, -STAGGER / 2.0))
    assert state_errors(state, TRUTH, STAGGER, RADII, VIEWER)['panel_m'] < 1e-9


def test_errors_read_in_their_own_units():
    err = state_errors(_state(center=(3.05, 0.5, 0.3), vel=(0.0, 2.3, 0.0), yaw=0.45,
                              w=8.0), TRUTH, STAGGER, RADII, VIEWER)
    assert math.isclose(err['center_m'], 0.05)
    assert math.isclose(err['velocity_m_s'], 0.3)
    assert math.isclose(err['yaw_rad'], 0.05)
    assert math.isclose(err['yaw_rate_rad_s'], 1.0)
    assert err['panel_m'] > 0.05  # the center shift plus the yaw error


def test_swapped_pairs_score_as_radius_and_height_error():
    # A quarter turn on with the pairs NOT swapped: the tracker got them wrong.
    err = state_errors(_state(yaw=0.4 + math.pi / 2.0), TRUTH, STAGGER, RADII, VIEWER)
    assert math.isclose(err['radius_m'], 0.06)
    assert math.isclose(err['z_offset_m'], STAGGER)


def _records(panel_errors, valid=True, dt=0.1):
    return [{'t': i * dt, 'valid': valid, 'age_on_arrival_s': 0.01,
             **{k: e for k in METRICS}}
            for i, e in enumerate(panel_errors)]


def test_converges_at_the_first_state_after_the_last_bad_one():
    summary = summarize_case(_records([0.2, 0.01, 0.2, 0.01, 0.01, 0.01]), 0.0, 0.3)
    assert math.isclose(summary['converge_s'], 0.3)
    assert summary['steady_states'] == 3


def test_never_converging_says_so():
    assert summarize_case(_records([0.2, 0.2]), 0.0, 0.0)['converge_s'] is None


def test_cell_names_carry_every_axis():
    assert cell_id('flat', 0.0, 'lateral', 0.0, False, 0.0) == 'flat-stationary-lateral-shooter0'
    assert cell_id('staggered', 2.0, 'radial', 1.0, True, 0.03) == \
        'staggered-speed2-radial-shooter1-blackout-camlat0.03'


def test_facing_panel_error_is_the_panel_facing_us():
    # Panel 0 faces away from us, so panel 2 (same pair) faces us. A wrong
    # side-pair radius is panel error, but not facing-panel error.
    truth = (np.array([3.0, 0.0, 0.3]), np.zeros(3), 0.0, 0.0)
    state = _state(center=(3.0, 0.0, 0.3), vel=(0.0, 0.0, 0.0), yaw=0.0, w=0.0,
                   radius=(0.30, 0.20), z_offset=(0.0, 0.0))
    err = state_errors(state, truth, 0.0, RADII, VIEWER)
    assert err['facing_panel_m'] < 1e-9 and err['panel_m'] > 0.01
    state.radius = (0.28, 0.24)
    err = state_errors(state, truth, 0.0, RADII, VIEWER)
    assert math.isclose(err['facing_panel_m'], 0.02, abs_tol=1e-6)
