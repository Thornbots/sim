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

"""Verify E2E wait completion at low RTF and genuine clock failures."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent / 'e2e'))
import e2e_harness as harness  # noqa: E402, I100


def clock_probe(monkeypatch, rate):
    clock = {'sim': 1.0, 'wall': 0.0}
    node = SimpleNamespace(now_s=lambda: clock['sim'])

    def spin_once(node, timeout_sec):
        clock['wall'] += timeout_sec
        clock['sim'] += rate * timeout_sec

    monkeypatch.setattr(harness.time, 'monotonic', lambda: clock['wall'])
    monkeypatch.setattr(harness.rclpy, 'spin_once', spin_once)
    return clock, node


def test_slow_clock_completes_the_entire_scoring_window(monkeypatch):
    clock, node = clock_probe(monkeypatch, 0.3)
    harness.E2EScorer.spin_for(node, 10.0)
    assert clock['sim'] >= 11.0
    assert clock['wall'] > 30.0


def test_stopped_clock_fails_without_waiting_for_the_entire_budget(monkeypatch):
    clock, node = clock_probe(monkeypatch, 0.0)
    with pytest.raises(RuntimeError, match='clock stalled'):
        harness.E2EScorer.spin_for(node, 10.0)
    assert 5.0 < clock['wall'] < 5.2


def test_backwards_clock_cannot_produce_a_valid_score(monkeypatch):
    _, node = clock_probe(monkeypatch, -1.0)
    with pytest.raises(RuntimeError, match='clock moved backwards'):
        harness.E2EScorer.spin_for(node, 10.0)
