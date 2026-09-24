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

"""tools/sim_pr_compare.py's PR parsing and report; no container or network."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import sim_pr_compare as spc  # noqa: E402


def _junit(path, cases):
    body = ''
    for name, failed, props in cases:
        props_xml = ''.join(f'<property name="{k}" value="{v}"/>' for k, v in props.items())
        body += (f'<testcase classname="t" name="{name}">'
                 f'{"<failure/>" if failed else ""}'
                 f'<properties>{props_xml}</properties></testcase>')
    with open(path, 'w') as f:
        f.write(f'<testsuites><testsuite>{body}</testsuite></testsuites>')


def test_parse_pr():
    assert spc.parse_pr('https://github.com/Thornbots/sim/pull/2') == ('Thornbots', 'sim', 2)
    assert spc.parse_pr('Thornbots/sim#2') == ('Thornbots', 'sim', 2)
    assert spc.parse_pr('thornbots_workspace#11') == ('Thornbots', 'thornbots_workspace', 11)


def test_report_diffs_means_and_marks_missing_runs(tmp_path):
    results = tmp_path / 'results'
    results.mkdir()
    _junit(results / 'base-drift-0.xml', [('dc', True, {'max_delta_m': 0.42, 'log_errors': 0})])
    _junit(results / 'base-drift-1.xml', [('dc', True, {'max_delta_m': 0.46, 'log_errors': 0})])
    _junit(results / 'head-drift-0.xml', [('dc', False, {'max_delta_m': 0.31, 'log_errors': 0})])
    sha = {p: 'a' * 40 for p in spc.PACKAGES}
    sides = {'base': sha, 'head': {**sha, 'sim': 'b' * 40}}
    pr = {'repo': 'Thornbots/sim', 'number': 2, 'title': 'x'}

    text = spc.report(pr, sides, str(tmp_path), ['drift'], 2)

    assert '| dc | FAIL FAIL | pass none | max_delta_m | 0.44 (0.42..0.46) | 0.31 | -0.13 |' \
        in text
    assert '`sim`: base `aaaaaaaa`, head `bbbbbbbb`' in text
    assert text.index('max_delta_m') < text.index('log_errors')
    assert (tmp_path / 'report.md').read_text() == text
