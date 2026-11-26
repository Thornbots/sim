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

"""Check the video timeline and verdict without ROS or FFmpeg."""
from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


SPEC = spec_from_file_location(
    'record_e2e', Path(__file__).resolve().parents[1] / 'tools/record_e2e.py')
RECORD = module_from_spec(SPEC)
SPEC.loader.exec_module(RECORD)


def test_retimes_variable_wall_pace_without_drift():
    """Each frame shows its place on the sim timeline regardless of wall pace."""
    sim = [1000. + i / 4 for i in range(2401)]
    wall = [1500. + 2 * i / 4 if i <= 1200 else 2100. + (i - 1200) / 8
            for i in range(len(sim))]
    ranges, plan = RECORD.frame_plan(sim, wall, [(1000., 1600.)], 1500., 60, 30, .5)
    assert ranges == [(1000., 1600.)]
    assert len(plan) == 600 * 30
    for i, (stamp, source_frame) in enumerate(plan):
        elapsed = i / 30
        assert stamp == pytest.approx(1000. + elapsed, abs=1e-10, rel=0)
        expected_wall = 2 * elapsed if elapsed <= 300 else 600 + (elapsed - 300) / 2
        assert abs(source_frame - expected_wall * 60) < .500001


def test_clock_jumps_and_scored_gaps_share_one_frame_grid():
    """Discard uncovered time and carry fractional frame time across cuts."""
    sim = [0., .15, .3, 4., 4.15, 4.3, 8., 8.15, 8.3]
    wall = [100. + stamp for stamp in sim]
    ranges, plan = RECORD.frame_plan(sim, wall, [(0., .15), (4., 8.3)], 100., 10, 10, .5)
    assert ranges == [(0., .15), (4., 4.3), (8., 8.3)]
    expected = [0., .1, 4.05, 4.15, 4.25, 8.05, 8.15, 8.25]
    assert [stamp for stamp, _ in plan] == pytest.approx(expected)
    assert len(plan) == 8


def test_clock_collapses_pauses_and_refuses_rewinds():
    """Keep the latest wall sample for a paused clock and reject a reset."""
    rows = [{'sim': 0., 'wall': 100.}, {'sim': 0., 'wall': 120.}, {'sim': .1, 'wall': 120.1}]
    assert RECORD.clock_axes(rows) == ([0., .1], [120., 120.1])
    with pytest.raises(RuntimeError, match='backwards'):
        RECORD.clock_axes([*rows, {'sim': 0., 'wall': 121.}])


@pytest.mark.parametrize('verdict', ['FAIL', None])
def test_render_refuses_unaccepted_captures(tmp_path, verdict):
    """A failed or incomplete capture cannot create a PASS video."""
    output = tmp_path / 'suite.mp4'
    (tmp_path / 'recording.json').write_text(json.dumps({'suite_exit': 0, 'capture_ok': True}))
    if verdict is not None:
        (tmp_path / 'summary.json').write_text(json.dumps({'schema_version': 1, 'verdict': verdict}))
    with pytest.raises(RuntimeError, match='Run failed|No readable run verdict'):
        RECORD.render(SimpleNamespace(directory=tmp_path, output=output))
    assert not output.exists()


def test_match_banner_names_the_official_denominator(tmp_path):
    """Render the referee credits and native scored-shot count from the run summary."""
    report = Path(__file__).parent / 'fixtures/suite_summary/match.json'
    phases = {row['segment']: {'referee_credits': row['referee_credited_hits'],
                               'sentry_scored_shots': row['shots']}
              for row in json.loads(report.read_text())['segments']}
    summary = {'schema_version': 1, 'verdict': 'PASS', 'suite': 'mcb_match',
               'metrics': {'phases': phases}}
    (tmp_path / 'summary.json').write_text(json.dumps(summary))
    (tmp_path / 'recording.json').write_text(json.dumps({'suite_exit': 1}))
    assert RECORD.banner_details(RECORD.require_pass(tmp_path)) == (
        '1x sim time   |   Run Sentry referee credits / scored shots 39/52')
