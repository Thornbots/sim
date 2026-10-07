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

"""Score the real firmware while driving from blue spawn into a center fight."""
import json
import os

import e2e_harness as harness
import numpy as np
import pytest

pytestmark = pytest.mark.integration


@pytest.fixture(scope='module')
def e2e_stack(request, ros_context):
    config = request.config
    log_dir = config.getoption('--log-dir') or '/tmp/e3_test_logs'
    os.makedirs(log_dir, exist_ok=True)
    stack = harness.E2EStack(config.getoption('--headless'), log_dir, stage='e3')
    try:
        stack.start()
        yield stack
    finally:
        stack.stop()


def test_e3_spawn_to_center(e2e_stack):
    shots = harness.run_match(e2e_stack)
    harness.record_score(e2e_stack, 'e3-spawn-to-center', shots, harness.match_duration())
    mcb = shots['mcb']
    assert mcb, 'firmware fired no shots on the spawn-to-center route'
    summaries = []
    for segment in ['approach_2', 'parked', 'straight_1', 'turn', 'spin']:
        records = [shot for shot in mcb if shot.get('segment') == segment]
        telemetry = [r for r in e2e_stack.scorer.route_records if r.get('segment') == segment]
        route = [r['route_error_m'] for r in telemetry if 'route_error_m' in r]
        pose = [r['localization_error_m'] for r in telemetry
                if r.get('localization_error_m') is not None]
        diagnosis = harness.segment_diagnostics(e2e_stack, segment, records)
        summary = {'segment': segment, 'shots': len(records),
                   'hit_rate': harness.hit_rate(records), 'diagnosis': diagnosis,
                   'route_p95_m': float(np.percentile(route, 95)) if route else None,
                   'localization_p95_m': float(np.percentile(pose, 95)) if pose else None}
        summaries.append(summary)
        print(summary)
        assert route and np.percentile(route, 95) < 0.40, f'{segment}: route not followed'
        assert pose, f'{segment}: no stamped localization diagnostics'
        # E2E_PLAN's done bar allows a measured diagnosis in place of a passing hit rate.
        assert harness.hit_rate(records) >= harness.PLACEHOLDER_FLOOR or diagnosis, (
            f'{segment}: low hit rate without a diagnostic naming the failing hop')
    with open(os.path.join(e2e_stack.log_dir, 'segments.json'), 'w') as stream:
        json.dump(summaries, stream, indent=2)
