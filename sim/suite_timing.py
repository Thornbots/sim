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
Wall and sim seconds per phase of each integration test case.

Harnesses wrap their overhead in phase('bringup'|'reset'|'settle'|'teardown');
time a case spends outside every phase counts as 'scored', as does an explicit
phase('scored'), which keeps its sim time when the clock goes before the case
ends. set_sim_clock() hands over a callable returning sim seconds, so spans
get sim time and an RTF. test/conftest.py opens a case per test and prints
report() at the end. No rclpy import, so it loads in the launch files too.
"""
import collections
import contextlib
import time

PHASES = ('sim_start', 'bringup', 'reset', 'settle', 'scored', 'teardown')
SETUP = '(setup)'  # phases outside any test: module fixtures' bring-up and teardown

_sim_now = None
_suite = None
_case = None
_mark = None  # (wall, sim) at the case's last phase boundary
# (suite, case) -> phase -> [wall_s, sim_s, sim-timed wall_s]
_totals = collections.defaultdict(
    lambda: collections.defaultdict(lambda: [0.0, 0.0, 0.0]))


def set_sim_clock(sim_now):
    """Use sim_now() for sim seconds from here on; None stops sim timing."""
    global _sim_now
    _sim_now = sim_now


def _now():
    sim = _sim_now() if _sim_now is not None else None
    return time.monotonic(), sim if sim else None  # 0.0 means no /clock yet


def _add(phase, start, end):
    row = _totals[(_suite, _case or SETUP)][phase]
    row[0] += end[0] - start[0]
    if start[1] is not None and end[1] is not None:
        row[1] += end[1] - start[1]
        row[2] += end[0] - start[0]


@contextlib.contextmanager
def suite(name):
    global _suite
    outer, _suite = _suite, name
    try:
        yield
    finally:
        _suite = outer


@contextlib.contextmanager
def case(name):
    global _case, _mark
    _case, _mark = name, _now()
    try:
        yield
    finally:
        _add('scored', _mark, _now())
        _case = _mark = None


@contextlib.contextmanager
def phase(name):
    """Time the block as `name`; inside a case, the gap before it is scored."""
    global _mark
    start = _now()
    if _case is not None:
        _add('scored', _mark, start)
    try:
        yield
    finally:
        end = _now()
        _add(name, start, end)
        if _case is not None:
            _mark = end


def report():
    """Return the timing table as lines, one block per suite; [] if nothing ran."""
    suites = collections.OrderedDict()
    for (suite_name, case_name), phases in _totals.items():
        suites.setdefault(suite_name, []).append((case_name, phases))
    lines = []
    for suite_name, cases in suites.items():
        width = max(len(c) for c, _ in cases + [('total', None)])
        lines.append(f'{suite_name}: wall seconds per phase; RTF over sim-timed spans')
        lines.append(f'  {"case":{width}s} ' + ' '.join(f'{p:>9s}' for p in PHASES)
                     + f' {"total":>8s} {"RTF":>6s}')
        grand = collections.defaultdict(lambda: [0.0, 0.0, 0.0])
        for case_name, phases in cases + [('total', grand)]:
            if case_name != 'total':
                for p, row in phases.items():
                    grand[p] = [a + b for a, b in zip(grand[p], row)]
            sim = sum(r[1] for r in phases.values())
            sim_wall = sum(r[2] for r in phases.values())
            rtf = f'{sim / sim_wall:6.2f}' if sim_wall > 0 else f'{"-":>6s}'
            lines.append(
                f'  {case_name:{width}s} '
                + ' '.join(f'{phases[p][0]:9.1f}' if p in phases else f'{"":9s}'
                           for p in PHASES)
                + f' {sum(r[0] for r in phases.values()):8.1f} {rtf}')
    return lines
