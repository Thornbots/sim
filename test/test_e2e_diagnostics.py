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
"""Verify E2E diagnosis follows the hit path and never blames map localization."""
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent / 'e2e'))
import e2e_harness as harness  # noqa: E402, I100
sys.path.insert(0, str(Path(__file__).parent.parent / 'tools'))
import compare_runs  # noqa: E402, I100


def stack(**record):
    return SimpleNamespace(scorer=SimpleNamespace(route_records=[{'segment': 's', **record}]))


def test_map_error_alone_is_never_a_diagnosis():
    shots = [{'aim_off_panel_m': 0.01, 'barrel_off_aim_deg': 0.2}]
    assert harness.segment_diagnostics(stack(localization_error_m=2.0), 's', shots) is None


def test_aim_is_blamed_before_stamps_and_barrel():
    shots = [{'aim_off_panel_m': 0.5, 'barrel_off_aim_deg': 5.0}]
    diagnosis = harness.segment_diagnostics(stack(head_tf_error_deg=5.0), 's', shots)
    assert diagnosis.startswith('target_tracker')


def test_stamps_are_blamed_before_barrel():
    shots = [{'aim_off_panel_m': 0.01, 'barrel_off_aim_deg': 5.0}]
    diagnosis = harness.segment_diagnostics(stack(head_tf_error_deg=5.0), 's', shots)
    assert diagnosis.startswith('pose/TF')
    assert harness.segment_diagnostics(stack(), 's', shots).startswith('MCB')


def test_compare_runs_names_the_first_diverging_leaf():
    a = {'t': 1.0, 'hit': True, 'aim': [0.1, 0.2]}
    assert compare_runs.differ(a, dict(a), 0.0) == []
    assert compare_runs.differ(a, {**a, 'aim': [0.1, 0.25]}, 0.0) == [('aim[1]', 0.2, 0.25)]
    assert compare_runs.differ(a, {**a, 'aim': [0.1, 0.25]}, 0.1) == []
    assert compare_runs.differ(a, {**a, 'hit': False}, 1.0) == [('hit', True, False)]
