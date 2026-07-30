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

DEFAULT_SWEEP_HZ = 1.0 / 6.0    # full back-and-forth cycle every 6s, as before
DEFAULT_AMPLITUDE = 2.5         # rad, safely inside the +-pi joint limit --
                                # the SLAM-blind-wedge use case wants full
                                # coverage, not frustum-keeping.


class HeadSweep(Node):
    def __init__(self):
        super().__init__('head_sweep')
        self.declare_parameter('sweep_hz', DEFAULT_SWEEP_HZ)
        self.declare_parameter('amplitude_rad', DEFAULT_AMPLITUDE)
        self.sweep_hz = float(self.get_parameter('sweep_hz').value)
        self.amplitude_rad = float(self.get_parameter('amplitude_rad').value)
        self.pub = self.create_publisher(Float64, '/head_pan_cmd', 10)
        self.start = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        t = (self.get_clock().now() - self.start).nanoseconds / 1e9
        angle = self.amplitude_rad * math.sin(2 * math.pi * t * self.sweep_hz)
        self.pub.publish(Float64(data=angle))


def main(args=None):
    rclpy.init(args=args)
    node = HeadSweep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
