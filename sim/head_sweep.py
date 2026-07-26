"""
Slowly sweeps the head (and its mounted lidar) back and forth while
driving, to move the head's lidar self-occlusion blind wedge over time
so SLAM's scan integration ends up covering the full circle. headlink's
joint limit is +-pi (see sentry.urdf.xacro), so this sweeps rather than
spins continuously.
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
