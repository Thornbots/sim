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

"""Check first-hit geometry, finite armor faces and front/normal-speed damage rules."""
import json

import numpy as np
from sim.combat import Referee, segment_mesh_hit, segment_panel_hit, ShotResolver


def test_mesh_hit_is_finite_and_selects_the_nearest_obstruction():
    triangles = np.array([[[2, -1, -1], [2, 1, -1], [2, 0, 1]],
                          [[1, -1, -1], [1, 1, -1], [1, 0, 1]]], dtype=float)
    assert segment_mesh_hit(np.array([0., 0, 0]), np.array([3., 0, 0]), triangles) == 1/3
    assert segment_mesh_hit(np.array([0., 0, 0]), np.array([.5, 0, 0]), triangles) is None
    assert segment_mesh_hit(np.array([0., 2, 0]), np.array([3., 2, 0]), triangles) is None


def test_panel_is_a_face_not_a_hit_sphere():
    panel = np.eye(4)
    assert segment_panel_hit(np.array([1., 0, 0]), np.array([-1., 0, 0]), panel) == .5
    assert segment_panel_hit(np.array([1., .08, 0]), np.array([-1., .08, 0]), panel) is None
    assert segment_panel_hit(np.array([1., 0, .07]), np.array([-1., 0, .07]), panel) is None


def test_damage_dead_time_is_per_panel_and_hp_cannot_go_negative():
    referee = Referee({'red': 'red', 'blue': 'blue'})
    impact = {'victim': 'red', 'panel': 0, 'impact_t': 1.0, 'eligible': True}
    assert referee.apply(impact)
    assert not referee.apply(dict(impact, impact_t=1.049))
    assert referee.apply(dict(impact, panel=1, impact_t=1.01))
    assert referee.hp['red'] == 360
    for tick in range(25):
        referee.apply(dict(impact, impact_t=2.0 + tick * .1))
    assert referee.hp['red'] == 0
    assert referee.hp['blue'] == 400
    assert not referee.apply(dict(impact, impact_t=6.0))


def test_ineligible_physical_impact_does_not_damage_armor():
    referee = Referee({'robot': 'red'})
    assert not referee.apply({'victim': 'robot', 'panel': 0,
                              'impact_t': 1.0, 'eligible': False})
    assert referee.hp['robot'] == 400


class MovingRoot:
    def __init__(self, speed=0.0):
        self.speed = speed

    def at(self, seconds):
        root = np.eye(4)
        root[0, 3] = 2.0 + self.speed * seconds
        return root, None


def resolver_scene():
    resolver = object.__new__(ShotResolver)
    resolver.field = np.empty((0, 3, 3))
    resolver.hull = np.array([[[-.20, -1, -1], [-.20, 1, -1], [-.20, 0, 1]]])
    resolver.ghost_hull = resolver.hull.copy()
    panel = np.diag([-1., -1., 1., 1.])
    panel[0, 3], panel[2, 3] = -.25, .2
    resolver.armors = [panel]
    return resolver


def test_ballistic_panel_hit_precedes_hull_and_can_be_logged():
    resolver = resolver_scene()
    result = resolver.resolve('shooter', np.array([0., 0, .2]), np.array([1., 0, 0]),
                              0.0, {'victim': MovingRoot()})
    assert result['impact'] == 'panel'
    assert result['eligible'] is True
    assert result['normal_speed'] > 12.0
    json.dumps(result)
    # A field wall absorbs that same ray before the panel.
    resolver.field = np.array([[[1, -1, -1], [1, 1, -1], [1, 0, 1]]], dtype=float)
    blocked = resolver.resolve('shooter', np.array([0., 0, .2]), np.array([1., 0, 0]),
                               0.0, {'victim': MovingRoot()})
    assert blocked['impact'] == 'field'
    assert blocked['impact_t'] < result['impact_t']


def test_low_relative_normal_speed_and_back_faces_cannot_damage():
    resolver = resolver_scene()
    origin = np.array([0., 0, .2])
    slow = resolver.resolve('shooter', origin, np.array([1., 0, 0]),
                            0.0, {'victim': MovingRoot(speed=16.0)})
    assert not slow['eligible']
    resolver.armors[0][:3, :3] = np.eye(3)
    back = resolver.resolve('shooter', origin, np.array([1., 0, 0]),
                            0.0, {'victim': MovingRoot()})
    assert back['impact'] == 'panel' and not back['eligible']
