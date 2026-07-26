"""
WASD keyboard teleop for the sim robot's holonomic /cmd_vel (sim-only;
real hardware drives via the DJI Type-C board, not ROS). Talks to the
VelocityControl gz plugin (sim/urdf/sentry.urdf.xacro) via the /cmd_vel
bridge in sim/launch/sim.launch.py.

  w/s forward/back  a/d strafe  q/e rotate  space stop  x quit
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BINDINGS = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
    ' ': (0, 0, 0),
}

LINEAR_SPEED = 0.3   # m/s
ANGULAR_SPEED = 0.5  # rad/s


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = Node('wasd_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    settings = termios.tcgetattr(sys.stdin)
    print(__doc__)
    try:
        while True:
            key = get_key(settings)
            if key == 'x' or key == '\x03':  # x or Ctrl-C
                break
            vx, vy, wz = BINDINGS.get(key, (0, 0, 0))
            twist = Twist()
            twist.linear.x = vx * LINEAR_SPEED
            twist.linear.y = vy * LINEAR_SPEED
            twist.angular.z = wz * ANGULAR_SPEED
            pub.publish(twist)
    finally:
        pub.publish(Twist())  # stop on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
