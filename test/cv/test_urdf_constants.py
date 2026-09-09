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
Pin every hard-coded copy of the head FK chain against sentry.urdf.xacro.

The root->body->head->head_pitch->camera constants are duplicated on
purpose across cv_head_aim_core, cv_target_emulator, shot_hit_harness and
both urdf/sentry.urdf.xacro copies -- see shot_hit_harness.py's module
docstring for why the harness re-derives the chain instead of importing
the emulator's. That is fine for the FK *algebra*; it is not fine for the
*numbers*, which had no cross-check at all. A drifted origin leaves every
other test green (test_cv_head_aim.py imports HEADPITCH_ORIGIN_YAW from
the module it is testing, so both sides of its round-trip move together)
and surfaces only as a collapsed shot-hit rate -- which is how -0.38885
cost a debugging cycle already, see sim/README.md's ## Notes.

So: parse the xacro and assert each copy against it. The emulator and
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

from sim.cv_head_aim_core import HEADPITCH_ORIGIN_YAW  # noqa: E402

WORKSPACE_SRC = os.path.dirname(SIM_DIR)
SIM_XACRO = os.path.join(SIM_DIR, 'urdf', 'sentry.urdf.xacro')
THORNBOTS_XACRO = os.path.join(
    WORKSPACE_SRC, 'thornbots_pkg', 'urdf', 'sentry.urdf.xacro')
EMULATOR = os.path.join(SIM_DIR, 'sim', 'cv_target_emulator.py')
HARNESS = os.path.join(os.path.dirname(__file__), 'shot_hit_harness.py')

CHAIN_JOINTS = ('fastened_2', 'headlink', 'headpitch', 'cameralink')
# The xacro writes pi as 3.14159; the Python copies use math.pi. That
# 2.7e-6 rad gap is rounding in the ONSHAPE export, not drift.
PI_TOL = 1e-5
EXACT_TOL = 1e-12


def _joint_origins(xacro_path):
    """Return {joint_name: (xyz, rpy, axis_or_None)} for the head chain."""
    root = ET.parse(xacro_path).getroot()
    out = {}
    for joint in root.iter('joint'):
        name = joint.get('name')
        if name not in CHAIN_JOINTS:
            continue
        origin = joint.find('origin')
        axis = joint.find('axis')
        out[name] = (
            tuple(float(v) for v in origin.get('xyz').split()),
            tuple(float(v) for v in origin.get('rpy').split()),
            tuple(float(v) for v in axis.get('xyz').split()) if axis is not None else None,
        )
    missing = set(CHAIN_JOINTS) - set(out)
    assert not missing, f'{xacro_path} no longer defines joints {sorted(missing)}'
    return out


def _module_constants(py_path, names):
    """
    Extract top-level constant assignments from a module without importing it.

    Values are evaluated against a namespace holding only `math`, so
    `math.pi` resolves and nothing else can. `_rotation_from_rpy(r, p, y)`
    yields its `(r, p, y)` arguments and `_transform(rot, trans)` yields
    whatever its `rot` argument yields -- the rpy triples are the part
    worth comparing against the xacro, not the assembled matrices.
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


def _assert_chain_matches_xacro(py_path):
    joints = _joint_origins(SIM_XACRO)
    consts = _module_constants(py_path, CHAIN_CONSTANTS)

    assert consts['_T_FASTENED_2'][2] == pytest.approx(
        joints['fastened_2'][1][2], abs=PI_TOL)

    assert consts['_HEADLINK_ORIGIN_R'][2] == pytest.approx(
        joints['headlink'][1][2], abs=PI_TOL)
    # The xacro's ~1e-17 x/y are export noise, not a real offset.
    assert consts['_HEADLINK_ORIGIN_T'] == pytest.approx(joints['headlink'][0], abs=1e-9)
    assert consts['_HEADLINK_AXIS'] == pytest.approx(joints['headlink'][2], abs=EXACT_TOL)

    assert consts['_HEADPITCH_ORIGIN_R'][2] == pytest.approx(
        joints['headpitch'][1][2], abs=EXACT_TOL)
    assert consts['_HEADPITCH_ORIGIN_T'] == pytest.approx(
        joints['headpitch'][0], abs=EXACT_TOL)
    assert consts['_HEADPITCH_AXIS'] == pytest.approx(joints['headpitch'][2], abs=EXACT_TOL)


def test_head_aim_core_yaw_matches_xacro():
    # The one constant test_cv_head_aim.py's round-trip cannot check,
    # because it imports it from the module under test.
    joints = _joint_origins(SIM_XACRO)
    assert HEADPITCH_ORIGIN_YAW == pytest.approx(joints['headpitch'][1][2], abs=EXACT_TOL)


def test_emulator_fk_constants_match_xacro():
    _assert_chain_matches_xacro(EMULATOR)


def test_harness_fk_constants_match_xacro():
    _assert_chain_matches_xacro(HARNESS)


def test_camera_link_is_still_identity():
    # All three Python copies stop the chain at head_pitch and treat the
    # camera frame as identical to it.
    xyz, rpy, _ = _joint_origins(SIM_XACRO)['cameralink']
    assert xyz == (0.0, 0.0, 0.0)
    assert rpy == (0.0, 0.0, 0.0)


def test_both_package_urdfs_agree_on_the_head_chain():
    # test_shot_hit.py scores against shot_hit_harness's FK (sim's xacro)
    # while the TF tree under test comes from thornbots_pkg's
    # auto.launch.py + robot_state_publisher (thornbots_pkg's xacro). If
    # those drift, the harness scores against a robot the stack isn't
    # driving, and neither side looks wrong on its own.
    if not os.path.exists(THORNBOTS_XACRO):
        pytest.skip('thornbots_pkg not checked out alongside sim')
    sim_joints = _joint_origins(SIM_XACRO)
    pkg_joints = _joint_origins(THORNBOTS_XACRO)
    for name in CHAIN_JOINTS:
        assert sim_joints[name][0] == pytest.approx(pkg_joints[name][0], abs=EXACT_TOL), name
        assert sim_joints[name][1] == pytest.approx(pkg_joints[name][1], abs=EXACT_TOL), name
