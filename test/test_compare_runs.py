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
            self.write_rows(run, 'poses.jsonl', [{'t': 1, 'truth_xy': [0, 1]}])
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
            (run / 'shots.jsonl').write_text('{"t": 1, "hit": true}\n{"t":\n')
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
        self.write_rows(self.a, 'states.jsonl', [{'t': 1, 'valid': False, 'centre_xy_m': 0,
                                                 'vel_err': [0, 0, 0]}])
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
            self.write_rows(run, 'shots.jsonl',
                            [{'t': 1, 'hit': False, 'aim': {'xy': [value, 4]}}])
        code, output = self.compare()
        self.assertNotEqual(code, 0)
        self.assertIn('first divergence at record 0', output)
        self.assertIn('aim.xy[0]: 2 != 3', output)

    def test_missing_field_is_distinct_from_null(self):
        """Incomplete object fields must not compare equal to explicit nulls."""
        for missing_run, null_run in ((self.a, self.b), (self.b, self.a)):
            with self.subTest(missing_run=missing_run):
                self.write_rows(missing_run, 'shots.jsonl', [{'t': 1, 'hit': False, 'aim': {}}])
                self.write_rows(null_run, 'shots.jsonl',
                                [{'t': 1, 'hit': False, 'aim': {'xy': None}}])
                code, output = self.compare()
                self.assertNotEqual(code, 0)
                self.assertIn('aim.xy:', output)
                self.assertIn('<missing>', output)

    def test_finite_tolerance_preserves_numeric_comparison(self):
        """Explicit tolerance works while exact comparison still detects drift."""
        for run, value in ((self.a, 2), (self.b, 2.05)):
            self.write_rows(run, 'shots.jsonl', [{'t': 1, 'hit': False, 'value': value}])
        self.assertNotEqual(self.compare()[0], 0)
        code, output = self.compare('--tol', '0.1')
        self.assertEqual(code, 0, output)

    def test_boolean_number_diagnostics_are_distinct(self):
        """Boolean diagnostics never equal numbers, even inside nested extras."""
        for boolean, number in ((True, 1), (False, 0)):
            for left, right in ((boolean, number), (number, boolean)):
                for field, wrap in (
                        ('eligible', lambda value: {'eligible': value}),
                        ('diagnostics.eligible',
                         lambda value: {'diagnostics': {'eligible': value}}),
                        ('diagnostics[0].flags[1]',
                         lambda value: {'diagnostics': [{'flags': [None, value]}]})):
                    for tolerance in ('0', '100'):
                        with self.subTest(left=left, right=right, field=field,
                                          tolerance=tolerance):
                            for run, value in ((self.a, left), (self.b, right)):
                                row = {'t': 1, 'hit': True, **wrap(value)}
                                self.write_rows(run, 'shots.jsonl', [row])
                            code, output = self.compare('--tol', tolerance)
                            self.assertNotEqual(code, 0, output)
                            self.assertIn('first divergence at record 0', output)
                            self.assertIn(field + ':', output)

    def test_numeric_types_and_optional_nested_diagnostics_pass(self):
        """Integer/float equivalence and tolerance coexist with typed extras."""
        for run, number in ((self.a, 1), (self.b, 1.0)):
            self.write_rows(run, 'shots.jsonl', [{
                't': number, 'hit': False, 'eligible': True,
                'diagnostics': {'values': [number, None, False], 'extra': {}},
            }])
        code, output = self.compare()
        self.assertEqual(code, 0, output)
        self.write_rows(self.b, 'shots.jsonl', [{
            't': 1.0, 'hit': False, 'eligible': True,
            'diagnostics': {'values': [1.05, None, False], 'extra': {}},
        }])
        self.assertNotEqual(self.compare()[0], 0)
        code, output = self.compare('--tol', '0.1')
        self.assertEqual(code, 0, output)

    def test_objects_without_run_payload_fail(self):
        """Object syntax, metadata and timestamps alone are not run evidence."""
        for name in ('shots.jsonl', 'states.jsonl', 'poses.jsonl', 'route.jsonl'):
            for row in ({}, {'unknown': 1}, {'t': None, 'hit': None},
                        {'t': 1}, {'t': 1, 'speed': 0, 'path': 'straight'}):
                with self.subTest(name=name, row=row):
                    for run in (self.a, self.b):
                        self.write_rows(run, name, [row])
                    code, output = self.compare()
                    self.assertNotEqual(code, 0, output)
                    self.assertIn(str(self.a / name) + ':1:', output)
                    self.assertIn('unusable', output)
                    for run in (self.a, self.b):
                        (run / name).unlink()

    def test_invalid_core_values_fail(self):
        """Known field names with wrong types or shapes cannot supply evidence."""
        cases = [
            ('shots.jsonl', {'t': 1, 'hit': value})
            for value in (None, 1, 'false', {}, [])
        ] + [
            ('poses.jsonl', {'t': 1, 'truth_xy': value})
            for value in (None, [], [0], [0, 1, 2], [0, None], [True, 0],
                          ['0', 1], {'x': 0, 'y': 1}, [0, float('inf')])
        ] + [
            ('states.jsonl', {'t': 1, 'valid': True, 'centre_xy_m': 0,
                              'vel_err': value})
            for value in (None, [], [0, 1], [0, 1, '2'])
        ] + [
            ('states.jsonl', {'t': 1, 'valid': 1, 'centre_xy_m': 0,
                              'vel_err': [0, 0, 0]}),
            ('states.jsonl', {'t': 1, 'valid': False, 'centre_xy_m': None,
                              'vel_err': [0, 0, 0]}),
        ] + [
            ('shots.jsonl', {'t': value, 'hit': False})
            for value in (None, True, '1', [], float('nan'), float('inf'))
        ]
        for name, row in cases:
            with self.subTest(name=name, row=row):
                for run in (self.a, self.b):
                    self.write_rows(run, name, [row])
                code, output = self.compare()
                self.assertNotEqual(code, 0, output)
                self.assertIn('unusable', output)
                for run in (self.a, self.b):
                    (run / name).unlink()

    def test_valid_producer_variants_and_null_diagnostics_pass(self):
        """CV shot stamps and unavailable TF diagnostics retain their semantics."""
        traces = {
            'shots.jsonl': [{'t_fire': 0, 'hit': False, 'off_face_m': None}],
            'states.jsonl': [{'t': 0, 'valid': False, 'centre_xy_m': 0,
                              'vel_err': [0, 0, 0], 'association_margin_m': None}],
            'poses.jsonl': [{'t': 0, 'truth_xy': [0, 0], 'localization_error_m': None,
                             'head_tf_error_deg': None, 'tracker_valid': False}],
            'route.jsonl': [{'t': 0, 'truth_xy': [0, 0], 'segment': 'approach'}],
        }
        for name, rows in traces.items():
            with self.subTest(name=name):
                for run in (self.a, self.b):
                    self.write_rows(run, name, rows)
                code, output = self.compare()
                self.assertEqual(code, 0, output)
                for run in (self.a, self.b):
                    (run / name).unlink()

    def test_valid_stream_does_not_hide_unusable_optional_record(self):
        """A populated optional stream must be valid on both sides in full."""
        self.matching_shots()
        for bad_run in (self.a, self.b):
            with self.subTest(bad_run=bad_run):
                for run in (self.a, self.b):
                    self.write_rows(run, 'poses.jsonl', [{'t': 1, 'truth_xy': [0, 0]}])
                with (bad_run / 'poses.jsonl').open('a') as trace:
                    trace.write('{}\n')
                code, output = self.compare()
                self.assertNotEqual(code, 0, output)
                self.assertIn(str(bad_run / 'poses.jsonl') + ':2:', output)
                self.assertIn('unusable', output)

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
