"""
Slowly sweeps the head back and forth while driving. The head (and the
lidar mounted on it) partially obstructs the lidar's own field of view at
whatever bearing the head currently sits at; a fixed head position means
that blind wedge stays in the same place relative to the robot forever.
Sweeping it means the blind wedge moves too, so across a few seconds of
driving the lidar (via robot_state_publisher's already-correct head->lidar
TF, tracked through the headlink joint) ends up covering the full circle
even though any single instant is still partially obstructed -- SLAM
integrates scans over time, so this fills in what a fixed head position
never would.

headlink's own joint limit is +-pi (see sentry.urdf.xacro), so this is a
sweep, not a continuous spin.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

PERIOD_SECONDS = 6.0    # full back-and-forth cycle time
AMPLITUDE = 2.5         # rad, safely inside the +-pi joint limit


class HeadSweep(Node):
    def __init__(self):
        super().__init__('head_sweep')
        self.pub = self.create_publisher(Float64, '/head_pan_cmd', 10)
        self.start = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        t = (self.get_clock().now() - self.start).nanoseconds / 1e9
        angle = AMPLITUDE * math.sin(2 * math.pi * t / PERIOD_SECONDS)
        self.pub.publish(Float64(data=angle))


def main(args=None):
    rclpy.init(args=args)
    node = HeadSweep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
