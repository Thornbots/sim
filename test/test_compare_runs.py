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

"""Exercise the real comparator executable with isolated run evidence."""

import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class CompareRunsTest(unittest.TestCase):
    """Check failure diagnostics and optional-output compatibility without ROS."""

    def setUp(self):
        """Create isolated run directories; require an explicitly built binary."""
        self.binary = os.environ['SIM_COMPARE_RUNS']
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.a = Path(self.temporary.name) / 'a'
        self.b = Path(self.temporary.name) / 'b'
        self.a.mkdir()
        self.b.mkdir()

    def write_rows(self, run, name, rows):
        """Write object records in JSONL format."""
        (run / name).write_text(''.join(json.dumps(row) + '\n' for row in rows))

    def compare(self, *options):
        """Run the production comparator and retain both output streams."""
        result = subprocess.run(
            [self.binary, str(self.a), str(self.b), *options],
            capture_output=True, text=True, check=False, timeout=10)
        return result.returncode, result.stdout + result.stderr

    def matching_shots(self):
        """Populate one shared nonempty trace."""
        for run in (self.a, self.b):
            self.write_rows(run, 'shots.jsonl', [{'t': 1, 'hit': True}])

    def test_empty_directories_fail(self):
        """Missing all recognized files cannot establish equality."""
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn('shots.jsonl: absent in both runs, not compared', output)
        self.assertIn('no comparable records', output)
        self.assertIn('equality not established', output)

    def test_nonexistent_run_fails(self):
        """A nonexistent path is reported as an invalid run directory."""
        self.a.rmdir()
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn('not a run directory: ' + str(self.a), output)

    def test_all_empty_or_blank_files_fail(self):
        """Creating empty output files does not manufacture evidence."""
        for content in ('', '\n \t\r\n'):
            with self.subTest(content=content):
                for run in (self.a, self.b):
                    for name in ('shots.jsonl', 'states.jsonl',
                                 'poses.jsonl', 'route.jsonl'):
                        (run / name).write_text(content)
                code, output = self.compare()
                self.assertNotEqual(code, 0)
                self.assertIn('no records compared', output)
                self.assertIn('no comparable records', output)

    def test_optional_outputs_may_be_empty_or_absent(self):
        """No-shot intervals can compare using pose evidence alone."""
        for run in (self.a, self.b):
            self.write_rows(run, 'poses.jsonl', [{'t': 1, 'xy': [0, 1]}])
            (run / 'shots.jsonl').touch()
        code, output = self.compare()
        self.assertEqual(code, 0, output)
        self.assertIn('shots.jsonl: empty in both runs, no records compared', output)
        self.assertIn('poses.jsonl: identical, 1 records', output)
        self.assertIn('route.jsonl: absent in both runs, not compared', output)

    def test_asymmetric_file_fails_even_with_other_matching_evidence(self):
        """A missing counterpart cannot be hidden by another matching trace."""
        self.matching_shots()
        for run, label in ((self.a, 'A'), (self.b, 'B')):
            with self.subTest(run=run):
                trace = run / 'states.jsonl'
                trace.touch()
                code, output = self.compare()
                self.assertNotEqual(code, 0)
                self.assertIn('states.jsonl: only in run ' + label, output)
                trace.unlink()

    def test_empty_counterpart_fails(self):
        """An empty counterpart is an incomplete trace, not an equal one."""
        self.matching_shots()
        (self.b / 'shots.jsonl').write_text('')
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn('then A has 1, B 0', output)

    def test_truncated_record_sequence_fails(self):
        """Matching prefixes still fail when one run lost its last record."""
        self.matching_shots()
        self.write_rows(self.a, 'shots.jsonl',
                        [{'t': 1, 'hit': True}, {'t': 2, 'hit': False}])
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn('identical for 1 records, then A has 2, B 1', output)

    def test_malformed_second_line_reports_location(self):
        """Even matching corrupt files must fail with their path and line."""
        for run in (self.a, self.b):
            (run / 'shots.jsonl').write_text('{"t": 1}\n{"t":\n')
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn(str(self.a / 'shots.jsonl') + ':2:', output)

    def test_nonobject_records_fail(self):
        """Valid JSON scalars and arrays are not JSONL trace objects."""
        for content in ('null\n', '3\n', '[]\n', '"trace"\n'):
            with self.subTest(content=content):
                for run in (self.a, self.b):
                    (run / 'shots.jsonl').write_text(content)
                code, output = self.compare()
                self.assertNotEqual(code, 0)
                self.assertIn('expected a JSON object record', output)

    def test_corrupt_optional_counterpart_fails(self):
        """Valid shot evidence cannot hide a corrupt optional trace."""
        self.matching_shots()
        self.write_rows(self.a, 'states.jsonl', [{'t': 1}])
        (self.b / 'states.jsonl').write_text('{"t":')
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn(str(self.b / 'states.jsonl') + ':1:', output)

    def test_trace_path_must_be_a_regular_file(self):
        """A directory named like an output is not usable evidence."""
        for run in (self.a, self.b):
            (run / 'shots.jsonl').mkdir()
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn('expected regular trace files', output)

    def test_equivalent_nested_traces_pass(self):
        """Key order and blank lines preserve logical object equality."""
        self.write_rows(self.a, 'shots.jsonl',
                        [{'t': 1, 'aim': {'xy': [2, 3]}, 'hit': True}])
        (self.b / 'shots.jsonl').write_text('\n{"hit":true,"aim":{"xy":[2,3]},"t":1}\n')
        code, output = self.compare()
        self.assertEqual(code, 0, output)
        self.assertIn('shots.jsonl: identical, 1 records', output)

    def test_different_nested_traces_report_first_divergence(self):
        """Changed numeric values retain the existing field-level diagnostic."""
        for run, value in ((self.a, 2), (self.b, 3)):
            self.write_rows(run, 'shots.jsonl', [{'t': 1, 'aim': {'xy': [value, 4]}}])
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn('first divergence at record 0', output)
        self.assertIn('aim.xy[0]: 2 != 3', output)

    def test_missing_field_is_distinct_from_null(self):
        """Incomplete object fields must not compare equal to explicit nulls."""
        for missing_run, null_run in ((self.a, self.b), (self.b, self.a)):
            with self.subTest(missing_run=missing_run):
                self.write_rows(missing_run, 'shots.jsonl', [{'t': 1, 'aim': {}}])
                self.write_rows(null_run, 'shots.jsonl',
                                [{'t': 1, 'aim': {'xy': None}}])
                code, output = self.compare()
                self.assertNotEqual(code, 0)
                self.assertIn('aim.xy:', output)
                self.assertIn('<missing>', output)

    def test_finite_tolerance_preserves_numeric_comparison(self):
        """Explicit tolerance works while exact comparison still detects drift."""
        for run, value in ((self.a, 2), (self.b, 2.05)):
            self.write_rows(run, 'shots.jsonl', [{'t': 1, 'value': value}])
        self.assertNotEqual(self.compare()[0], 0)
        code, output = self.compare('--tol', '0.1')
        self.assertEqual(code, 0, output)

    def test_invalid_tolerances_fail(self):
        """Nonfinite or malformed tolerance cannot turn differences into passes."""
        self.matching_shots()
        for value in ('nan', 'inf', '-1', '0.1junk'):
            with self.subTest(value=value):
                code, output = self.compare('--tol', value)
                self.assertNotEqual(code, 0)
                self.assertIn('--tol requires a finite nonnegative number', output)


if __name__ == '__main__':
    unittest.main()
