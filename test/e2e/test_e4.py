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

"""Run one fixed-seed 2v2 spawn-to-center fight with real MCB aim and referee UART."""
import json
import os

import e2e_harness as harness
import numpy as np
import pytest

pytestmark = pytest.mark.integration


@pytest.fixture(scope='module')
def e2e_stack(request, ros_context):
    config = request.config
    log_dir = config.getoption('--log-dir') or '/tmp/e4_test_logs'
    os.makedirs(log_dir, exist_ok=True)
    stack = harness.E2EStack(config.getoption('--headless'), log_dir, stage='e4',
                             real_time_factor=config.getoption('--real-time-factor'))
    try:
        stack.start()
        yield stack
    finally:
        stack.stop()


def test_e4_spawn_to_center_fight(e2e_stack):
    shots = harness.run_match(e2e_stack)
    scorer = e2e_stack.scorer
    actual = shots['mcb']
    allies = [shot for shot in actual if shot['friendly_intersection']]
    impacts = sorted([shot for kind, records in shots.items() if kind != 'flag'
                      for shot in records], key=lambda shot: shot['impact_t'])
    summaries = []
    for segment in ['approach_2', 'parked', 'straight_1', 'turn', 'spin']:
        records = [shot for shot in actual if shot.get('segment') == segment]
        telemetry = [r for r in scorer.route_records if r.get('segment') == segment]
        route = [r['route_error_m'] for r in telemetry if 'route_error_m' in r]
        pose = [r['localization_error_m'] for r in telemetry
                if r.get('localization_error_m') is not None]
        rate = sum(shot['enemy_hit'] for shot in records) / len(records) if records else 0.0
        fighting = segment != 'approach_2'
        diagnosis = harness.segment_diagnostics(e2e_stack, segment, records) if fighting else None
        end = max(r['t'] for r in telemetry) if telemetry else scorer.match_start
        completed = [shot for shot in impacts if shot['impact_t'] <= end]
        hp = completed[-1]['hp'] if completed else {name: 400 for name in scorer.referee.hp}
        summary = {'segment': segment, 'fighting': fighting,
                   'shots': len(records), 'enemy_hit_rate': rate,
                   'diagnosis': diagnosis, 'hp': hp,
                   'route_p95_m': float(np.percentile(route, 95)) if route else None,
                   'localization_p95_m': float(np.percentile(pose, 95)) if pose else None}
        summaries.append(summary)
        print(summary)
        assert route and np.percentile(route, 95) < 0.40, f'{segment}: route failed'
        assert pose, f'{segment}: stamped localization data missing'
        if fighting:
            assert rate >= harness.PLACEHOLDER_FLOOR or diagnosis, (
                f'{segment}: unexplained low score')
        else:
            assert not records, 'firmware fired before every robot reached center'
    report = {'segments': summaries, 'friendly_shots': len(allies),
              'damage_taken': 400 - scorer.referee.hp['sentry'],
              'hp': scorer.referee.hp,
              'referee_uart_frames': len(scorer.referee_messages),
              'shots_by_robot': {name: len(shots[kind]) for name, kind in
                                 [('sentry', 'mcb'), ('ally_0', 'ally_0'),
                                  ('opponent_0', 'opponent_0'), ('opponent_1', 'opponent_1')]}}
    with open(os.path.join(e2e_stack.log_dir, 'match.json'), 'w') as stream:
        json.dump(report, stream, indent=2)
    assert actual, 'the real firmware fired no shots in the center fight'
    assert all(shots[name] for name in ['opponent_0', 'opponent_1', 'ally_0']), (
        'a ghost never fired')
    refs = scorer.referee_messages
    assert refs and any(msg.game_stage == 4 for msg in refs), 'no live-match REF_SYS on UART'
    assert refs[-1].robot_hp == scorer.referee.hp['sentry'], 'HP did not return through UART'
    assert refs[-1].is_on_blue_team, 'referee lost our team on the wire'
    assert not allies, f'{len(allies)} firmware shots intersected our ally'
