"""
Grid-teleport mapping sweep for sim (sim-only). Visits a fixed (x, y)
grid in a snake pattern, teleporting the chassis to each point and
dwelling briefly so SLAM integrates a scan there -- no obstacle
avoidance/reactive driving, it just jumps regardless of what's there.
"Teleport" is a `/world/<world>/set_pose` gz-transport call via `ign
service`; each call is preceded by a model_only WorldReset to zero
joint state first. See README.md for why/how both are safe here.
"""
import subprocess

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
SERVICE_TIMEOUT = 2.0    # s -- ign service call timeout


def build_grid():
    """Snake order (alternating x direction per row) purely for a tidier
    sweep; teleporting makes travel distance irrelevant either way."""
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


def _ign_service(service, reqtype, reptype, req):
    try:
        result = subprocess.run(
            ['ign', 'service', '-s', f'/world/{WORLD_NAME}/{service}',
             '--reqtype', reqtype, '--reptype', reptype,
             '--timeout', str(int(SERVICE_TIMEOUT * 1000)),
             '--req', req],
            capture_output=True, text=True, timeout=SERVICE_TIMEOUT + 1.0,
        )
    except subprocess.TimeoutExpired:
        return False
    return 'true' in result.stdout


def reset_joints():
    """model_only WorldReset: snaps headlink/odowheel_x/odowheel_y back to
    their SDF-declared zero positions/velocities. Confirmed empirically
    that this does NOT touch root's own pose (it has no parent joint, so
    there's no "initial joint state" for it to reset to), only actual
    joints get reset."""
    return _ign_service(
        'control', 'ignition.msgs.WorldControl', 'ignition.msgs.Boolean',
        'reset: {model_only: true}',
    )


def teleport(x, y, z=Z):
    """True gz world-pose write via UserCommands' set_pose service; returns
    whether gz reported success, doesn't raise on a bad entity name.
    Orientation pinned to identity each call. reset_joints() runs both
    before AND after -- see README.md for why the after-call is needed
    (a reaction-impulse artifact from root's position discontinuity)."""
    reset_joints()
    req = (
        f"name: '{ENTITY_NAME}', "
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: 0, y: 0, z: 0, w: 1}}"
    )
    ok = _ign_service(
        'set_pose', 'ignition.msgs.Pose', 'ignition.msgs.Boolean', req,
    )
    reset_joints()
    return ok


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
