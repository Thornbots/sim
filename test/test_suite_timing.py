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

"""sim.suite_timing's phase accounting, on fake wall and sim clocks."""
import pytest
from sim import suite_timing


@pytest.fixture
def clocks(monkeypatch):
    now = {'wall': 0.0, 'sim': 0.0}
    monkeypatch.setattr(suite_timing.time, 'monotonic', lambda: now['wall'])
    monkeypatch.setattr(suite_timing, '_totals', type(suite_timing._totals)(
        suite_timing._totals.default_factory))
    yield now
    suite_timing.set_sim_clock(None)


def _advance(now, wall, sim=0.0):
    now['wall'] += wall
    now['sim'] += sim


def test_gaps_between_phases_count_as_scored(clocks):
    with suite_timing.suite('s'):
        with suite_timing.phase('bringup'):  # module fixture, outside any case
            _advance(clocks, 10.0)
        with suite_timing.case('c'):
            with suite_timing.phase('reset'):
                _advance(clocks, 2.0)
            suite_timing.set_sim_clock(lambda: clocks['sim'])
            _advance(clocks, 1.0, 1.0)  # sim clock starts mid-gap
            _advance(clocks, 4.0, 8.0)
            with suite_timing.phase('teardown'):
                suite_timing.set_sim_clock(None)
                _advance(clocks, 3.0)
    totals = suite_timing._totals
    assert totals[('s', suite_timing.SETUP)]['bringup'][0] == 10.0
    case = totals[('s', 'c')]
    assert case['reset'][0] == 2.0
    assert case['scored'][0] == 5.0
    assert case['teardown'][0] == 3.0
    # The scored gap started before /clock, so it has no sim time.
    assert case['scored'][1] == 0.0


def test_rtf_uses_only_sim_timed_spans(clocks):
    suite_timing.set_sim_clock(lambda: clocks['sim'])
    _advance(clocks, 0.0, 1.0)
    with suite_timing.suite('s'), suite_timing.case('c'):
        _advance(clocks, 4.0, 8.0)
    lines = suite_timing.report()
    assert lines[0].startswith('s:')
    assert lines[2].split()[-1] == '2.00'
    assert lines[-1].split()[0] == 'total'


def test_report_is_empty_when_nothing_ran(clocks):
    assert suite_timing.report() == []
