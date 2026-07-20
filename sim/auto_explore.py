"""
Grid-teleport mapping sweep for sim mapping runs. Sim-only: teleports the
chassis directly, no equivalent on real hardware.

Visits a fixed (x, y) grid in a snake pattern and teleports the chassis to
each point directly, one at a time, dwelling briefly at each so SLAM
(subscribed to /scan the same as ever) integrates a scan at that pose. No
obstacle avoidance, no reactive driving, no frontier bias: nothing here
steers around walls, it just jumps to the next grid point regardless of
what's there.

"Teleport" now means an actual gz-sim world-pose write via the
`/world/<world>/set_pose` gz-transport service (gz::sim::systems::
UserCommands, always loaded, see world/ARCC_Field_2026.sdf), called
directly through the `ign service` CLI since there's no ROS-side
equivalent to bridge and no gz-transport Python bindings in this image.
This only works because sentry.urdf.xacro's "root" link is a genuinely
free 6DOF body with no parent joint and no collision on any link: gz's
physics only honors a direct world-pose write on a link gz-physics'
FreeGroup API recognizes as free-floating (a jointed link silently
ignores it, confirmed empirically against an earlier version of the URDF
that drove root through a translation-only prismatic joint chain instead
specifically to prevent rotation), and having no collision means nothing
the chassis teleports through/into can ever generate contact forces that
would spin it up now that rotation is physically possible again.

The robot is holonomic and never turns during normal operation, but
unlike the old jointed design that made rotation structurally impossible,
nothing here enforces that any more (see sentry.urdf.xacro), so every
teleport call pins orientation to identity explicitly.
"""
import subprocess

import rclpy
from rclpy.node import Node

WORLD_NAME = 'ARCC_Field_2026'
ENTITY_NAME = 'sentry'  # robot_name arg default in sim/launch/sim.launch.py
Z = 0.03                # fixed spawn height, carried through every teleport

# Grid bounds, in world-frame meters. Widened 1m past the original
# conservative test-room-sized bounds in each direction to cover the whole
# field; teleporting past a wall is harmless now that root has no
# collision at all (see sentry.urdf.xacro), so oversizing this costs
# nothing but a few wasted waypoints.
GRID_X_MIN = -3.0
GRID_X_MAX = 3.0
GRID_Y_MIN = -3.0
GRID_Y_MAX = 3.0
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


def teleport(x, y, z=Z):
    """True gz world-pose write via UserCommands' set_pose service (see
    module docstring for why this now works). Returns whether gz reported
    success; doesn't raise on an unreachable/misnamed entity so a single
    bad call doesn't take the whole sweep down."""
    req = (
        f"name: '{ENTITY_NAME}', "
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: 0, y: 0, z: 0, w: 1}}"
    )
    try:
        result = subprocess.run(
            ['ign', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
             '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean',
             '--timeout', str(int(SERVICE_TIMEOUT * 1000)),
             '--req', req],
            capture_output=True, text=True, timeout=SERVICE_TIMEOUT + 1.0,
        )
    except subprocess.TimeoutExpired:
        return False
    return 'true' in result.stdout


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
