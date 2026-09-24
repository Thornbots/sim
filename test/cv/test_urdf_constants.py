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
Pin every hard-coded copy of the head FK chain against thornbots_pkg's URDF.

The root->body->head->head_pitch->camera/muzzle constants are duplicated on
purpose across cv_head_aim_core, cv_target_emulator, shot_hit_harness, sim's
sentry_v2 model and thornbots_pkg's URDF -- see shot_hit_harness.py's module
docstring for why the harness re-derives the chain instead of importing
the emulator's. That is fine for the FK *algebra*; it is not fine for the
*numbers*, which had no cross-check at all. A drifted origin leaves every
other test green (test_cv_head_aim.py imports its constants from the
module it is testing, so both sides of its round-trip move together)
and surfaces only as a collapsed shot-hit rate -- which is how -0.38885
cost a debugging cycle already, see sim/README.md's ## Notes.

So: parse the URDF and assert each copy against it. The emulator and
harness are read with `ast` rather than imported, because both pull in
rclpy and ROS message packages and this suite must stay runnable on a
bare Python 3 + pytest install. Launches nothing, so not `integration`.
"""
import ast
import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, SIM_DIR)

from sim.cv_head_aim_core import (  # noqa: E402
    HEADLINK_ORIGIN_X, HEADLINK_ORIGIN_Y, HEADLINK_ORIGIN_Z, HEADPITCH_ORIGIN,
    MUZZLELINK_ORIGIN,
)

WORKSPACE_SRC = os.path.dirname(SIM_DIR)
# The reference: plain URDF, the whole chain in one file. sim's model is the
# generated URDF plus the muzzle frame its xacro wrapper adds.
URDF = os.path.join(WORKSPACE_SRC, 'thornbots_pkg', 'urdf', 'sentry.urdf.xacro')
SIM_URDF = os.path.join(SIM_DIR, 'urdf', 'sentry_v2', 'sentry_v2.urdf')
SIM_XACRO = os.path.join(SIM_DIR, 'urdf', 'sentry_v2.urdf.xacro')
EMULATOR = os.path.join(SIM_DIR, 'sim', 'cv_target_emulator.py')
HARNESS = os.path.join(os.path.dirname(__file__), 'shot_hit_harness.py')

CHAIN_JOINTS = ('fastened_2', 'headlink', 'headpitch', 'cameralink', 'muzzlelink')
EXACT_TOL = 1e-12


def _joint_origins(xacro_path, names=CHAIN_JOINTS):
    """Return {joint_name: (xyz, rpy, axis_or_None)} for the head chain."""
    root = ET.parse(xacro_path).getroot()
    out = {}
    for joint in root.iter('joint'):
        name = joint.get('name')
        if name not in names:
            continue
        origin = joint.find('origin')
        axis = joint.find('axis')
        out[name] = (
            tuple(float(v) for v in origin.get('xyz').split()),
            tuple(float(v) for v in origin.get('rpy', '0 0 0').split()),
            tuple(float(v) for v in axis.get('xyz').split()) if axis is not None else None,
        )
    missing = set(names) - set(out)
    assert not missing, f'{xacro_path} no longer defines joints {sorted(missing)}'
    return out


def _module_constants(py_path, names):
    """
    Extract top-level constant assignments from a module without importing it.

    Values are evaluated against a namespace holding only `math`, so
    `math.pi` resolves and nothing else can. `_rotation_from_rpy(r, p, y)`
    yields its `(r, p, y)` arguments and `_transform(rot, trans)` yields
    whatever its `rot` argument yields -- the rpy triples are the part
    worth comparing against the URDF, not the assembled matrices.
    """
    with open(py_path) as f:
        tree = ast.parse(f.read(), filename=py_path)

    def value_of(node):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id == '_rotation_from_rpy':
                return tuple(value_of(a) for a in node.args)
            if node.func.id == '_transform':
                return value_of(node.args[0])
        return eval(compile(ast.Expression(node), py_path, 'eval'), {'math': math})

    found = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in names:
                found[target.id] = value_of(node.value)
    missing = set(names) - set(found)
    assert not missing, f'{py_path} no longer defines {sorted(missing)}'
    return found


CHAIN_CONSTANTS = (
    '_T_FASTENED_2', '_HEADLINK_ORIGIN_R', '_HEADLINK_ORIGIN_T', '_HEADLINK_AXIS',
    '_HEADPITCH_ORIGIN_R', '_HEADPITCH_ORIGIN_T', '_HEADPITCH_AXIS',
)


def _assert_chain_matches_urdf(py_path, end_link):
    joints = _joint_origins(URDF)
    end = f'_{end_link.upper()}_T'
    consts = _module_constants(py_path, CHAIN_CONSTANTS + (end,))

    assert consts['_T_FASTENED_2'][2] == pytest.approx(
        joints['fastened_2'][1][2], abs=EXACT_TOL)

    assert consts['_HEADLINK_ORIGIN_R'][2] == pytest.approx(
        joints['headlink'][1][2], abs=EXACT_TOL)
    assert consts['_HEADLINK_ORIGIN_T'] == pytest.approx(joints['headlink'][0], abs=EXACT_TOL)
    assert consts['_HEADLINK_AXIS'] == pytest.approx(joints['headlink'][2], abs=EXACT_TOL)

    assert consts['_HEADPITCH_ORIGIN_R'][2] == pytest.approx(
        joints['headpitch'][1][2], abs=EXACT_TOL)
    assert consts['_HEADPITCH_ORIGIN_T'] == pytest.approx(
        joints['headpitch'][0], abs=EXACT_TOL)
    assert consts['_HEADPITCH_AXIS'] == pytest.approx(joints['headpitch'][2], abs=EXACT_TOL)
    assert consts[end] == pytest.approx(joints[end_link][0], abs=EXACT_TOL)
    assert joints[end_link][1] == (0.0, 0.0, 0.0), f'{end_link} grew a rotation'


def test_head_aim_core_matches_urdf():
    # The parallax solve's lever arms. test_cv_head_aim.py's round-trip
    # imports these from the module under test, so only the URDF can catch
    # drift here.
    joints = _joint_origins(URDF)
    assert (HEADLINK_ORIGIN_X, HEADLINK_ORIGIN_Y, HEADLINK_ORIGIN_Z) == pytest.approx(
        joints['headlink'][0], abs=EXACT_TOL)
    assert HEADPITCH_ORIGIN == pytest.approx(joints['headpitch'][0], abs=EXACT_TOL)
    assert MUZZLELINK_ORIGIN == pytest.approx(joints['muzzlelink'][0], abs=EXACT_TOL)
    for name in ('fastened_2', 'headlink', 'headpitch', 'muzzlelink'):
        assert joints[name][1] == (0.0, 0.0, 0.0), f'{name} grew a rotation'


def test_emulator_fk_constants_match_urdf():
    _assert_chain_matches_urdf(EMULATOR, 'cameralink')


def test_harness_fk_constants_match_urdf():
    _assert_chain_matches_urdf(HARNESS, 'muzzlelink')


def test_sim_model_matches_urdf():
    # test_shot_hit.py scores against shot_hit_harness's FK while the TF
    # tree under test comes from thornbots_pkg's URDF, and the sim renders
    # the camera from its own model. If any two drift, the bench scores a
    # robot the stack isn't driving, and nothing looks wrong on its own.
    ref = _joint_origins(URDF)
    sim = {**_joint_origins(SIM_URDF, CHAIN_JOINTS[:-1]),
           **_joint_origins(SIM_XACRO, ('muzzlelink',))}
    for name in CHAIN_JOINTS:
        for k in range(3):
            if ref[name][k] is None:
                assert sim[name][k] is None, name
            else:
                assert sim[name][k] == pytest.approx(ref[name][k], abs=EXACT_TOL), name
