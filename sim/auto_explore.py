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
Grid-teleport mapping sweep for sim (sim-only).

Visits a fixed (x, y) grid in a snake pattern, teleporting the chassis
to each point and dwelling briefly so SLAM integrates a scan there --
no obstacle avoidance/reactive driving, it just jumps regardless of
what's there. "Teleport" is a `/world/<world>/set_pose` gz-transport
call, in process (a `gz service` CLI call costs ~0.3 s); each call is preceded by a model_only
WorldReset to zero joint state first. See README.md for why/how both
are safe here.
"""
import importlib
import math
import os
import re

from google.protobuf import text_format
import rclpy
from rclpy.node import Node

WORLD_NAME = 'ARCC_Field_2026'
ENTITY_NAME = 'sentry'  # robot_name arg default in sim/launch/sim.launch.py
Z = 0.03                # fixed spawn height, carried through every teleport

# Grid bounds, in world-frame meters. X tightened 0.5m on each side from
# the prior [-4, 4] to pull waypoints back off the walls in that axis.
GRID_X_MIN = -3.5
GRID_X_MAX = 3.5
GRID_Y_MIN = -5.0
GRID_Y_MAX = 5.0
GRID_SPACING = 0.5      # m between adjacent grid points

DWELL_SECONDS = 1.0      # s between teleports, so SLAM gets a settled scan
# at each pose before the next jump
SERVICE_TIMEOUT = 2.0    # s -- gz service call timeout


def build_grid():
    """
    Build the sweep waypoints in snake order (alternating x direction per row).

    Purely for a tidier sweep; teleporting makes travel distance
    irrelevant either way.
    """
    waypoints = []
    y = GRID_Y_MIN
    left_to_right = True
    while y <= GRID_Y_MAX + 1e-9:
        xs = []
        x = GRID_X_MIN
        while x <= GRID_X_MAX + 1e-9:
            xs.append(x)
            x += GRID_SPACING
        if not left_to_right:
            xs.reverse()
        waypoints.extend((x, y) for x in xs)
        left_to_right = not left_to_right
        y += GRID_SPACING
    return waypoints


_gz_node = None


def _gz_msg(type_name):
    """'gz.msgs.WorldControl' -> gz.msgs10.world_control_pb2.WorldControl."""
    name = type_name.rsplit('.', 1)[1]
    module = re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower() + '_pb2'
    return getattr(importlib.import_module(f'gz.msgs10.{module}'), name)


def _gz_call(service, req, rep_cls):
    """Call a world service with a gz.msgs request; the reply, or None on timeout."""
    global _gz_node
    from gz.transport13 import Node as GzNode  # gz-sim hosts only
    if _gz_node is None:
        # Without GZ_IP a Node resolves the hostname first, 20 s on a host
        # whose DNS stalls (2026-09-28): longer than any caller waits.
        os.environ.setdefault('GZ_IP', '127.0.0.1')
        _gz_node = GzNode()
    ok, rep = _gz_node.request(f'/world/{WORLD_NAME}/{service}', req, type(req), rep_cls,
                               int(SERVICE_TIMEOUT * 1000))
    return rep if ok else None


def _gz_request(service, req, rep_cls):
    """_gz_call for a gz.msgs.Boolean reply; True if gz replied true."""
    rep = _gz_call(service, req, rep_cls)
    return rep is not None and rep.data


def _gz_service(service, reqtype, reptype, req):
    """_gz_request with a text-format request, as `gz service --req` takes."""
    req_cls = _gz_msg(reqtype)
    return _gz_request(service, text_format.Parse(req, req_cls()), _gz_msg(reptype))


def spawn_model(name, sdf, x, y, z):
    """
    Spawn an SDF string as `name` at (x, y, z); False if gz refused.

    What `ros2 run ros_gz_sim create -string` does, without a process per
    model. The pose overrides the SDF's own <pose>, as create's -x/-y/-z do.
    """
    req = _gz_msg('gz.msgs.EntityFactory')(sdf=sdf, name=name, allow_renaming=False)
    req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
    return _gz_request('create', req, _gz_msg('gz.msgs.Boolean'))


def reset_joints():
    """
    Reset headlink/odowheel_x/odowheel_y to their SDF-declared zero positions/velocities.

    Uses a model_only WorldReset. Confirmed empirically that this does
    NOT touch root's own pose (it has no parent joint, so there's no
    "initial joint state" for it to reset to) -- only actual joints get
    reset.
    """
    return _gz_service(
        'control', 'gz.msgs.WorldControl', 'gz.msgs.Boolean',
        'reset: {model_only: true}',
    )


def teleport(x, y, z=Z, yaw=0.0):
    """
    Write x, y, z to gz world pose via UserCommands' set_pose service.

    Returns whether gz reported success; doesn't raise on a bad entity
    name. Orientation pinned to `yaw` (rad) each call. reset_joints() runs
    both before AND after -- see README.md for why the after-call is
    needed (a reaction-impulse artifact from root's position discontinuity).
    """
    reset_joints()
    ok = set_model_pose(ENTITY_NAME, x, y, z, yaw)
    reset_joints()
    return ok


def set_model_pose(name, x, y, z, yaw=0.0):
    """Set_pose one model to (x, y, z), turned yaw rad about z; no joint reset."""
    req = (
        f"name: '{name}', "
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}'
    )
    return _gz_service(
        'set_pose', 'gz.msgs.Pose', 'gz.msgs.Boolean', req,
    )


def model_names():
    """Names of every model in the world, or None if gz didn't answer."""
    _gz_msg('gz.msgs.Model')  # Scene's nested type; parsing fails unless loaded
    rep = _gz_call('scene/info', _gz_msg('gz.msgs.Empty')(), _gz_msg('gz.msgs.Scene'))
    return None if rep is None else [m.name for m in rep.model]


def remove_model(name):
    """Delete a model from the running world; False if gz refused or timed out."""
    return _gz_service(
        'remove', 'gz.msgs.Entity', 'gz.msgs.Boolean',
        f"name: '{name}', type: MODEL",
    )


class AutoExplore(Node):

    def __init__(self):
        super().__init__('auto_explore')
        self.waypoints = build_grid()
        self.index = 0
        self.done = False

        self.get_logger().info(
            f'grid sweep: {len(self.waypoints)} waypoints, '
            f'x=[{GRID_X_MIN},{GRID_X_MAX}] y=[{GRID_Y_MIN},{GRID_Y_MAX}] '
            f'spacing={GRID_SPACING}m'
        )
        self.timer = self.create_timer(DWELL_SECONDS, self.tick)
        self.tick()  # go to the first waypoint immediately instead of
        # waiting one full dwell period first

    def tick(self):
        if self.done:
            return
        x, y = self.waypoints[self.index]
        if teleport(x, y):
            self.get_logger().info(
                f'waypoint {self.index + 1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f})'
            )
        else:
            self.get_logger().warn(
                f'teleport to waypoint {self.index + 1} ({x:.2f}, {y:.2f}) failed'
            )
        self.index += 1
        if self.index >= len(self.waypoints):
            self.done = True
            self.timer.cancel()
            self.get_logger().info('grid sweep complete')


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
