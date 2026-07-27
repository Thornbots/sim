"""
Simulated fast-moving-target ground truth: no gz entity, model, or plugin --
this node just integrates its own (x, y, z) state in a timer callback and
publishes nav_msgs/Odometry on /target/ground_truth_odom, the same way
pose_emulator.py stands in for real hardware without touching gz. See
README.md for the lateral-traverse path shape and dwell-count rationale.

Frame: header.frame_id is set to match /sim/raw_odom's ('odom' by default,
see sentry.urdf.xacro's OdometryPublisher plugin) since cv_target_emulator
assumes both ground-truth topics share one world frame with no TF lookup.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TargetDriver(Node):
    def __init__(self):
        super().__init__('target_driver')

        self.declare_parameter('target_speed', 2.0)
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('center_x', 3.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('half_width', 2.0)
        self.declare_parameter('target_z', 0.3)
        self.declare_parameter('frame_id', 'odom')

        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.half_width = self.get_parameter('half_width').value
        self.target_z = self.get_parameter('target_z').value
        self.frame_id = self.get_parameter('frame_id').value

        self.y = self.center_y
        self.direction = 1.0
        self._last_time = None

        self.pub = self.create_publisher(Odometry, '/target/ground_truth_odom', 10)

        rate_hz = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            f"target_driver ready: speed={self.get_parameter('target_speed').value:.2f} m/s, "
            f"path x={self.center_x:.2f} y=[{self.center_y - self.half_width:.2f}, "
            f"{self.center_y + self.half_width:.2f}] z={self.target_z:.2f}, "
            f"frame_id={self.frame_id}"
        )

    def on_timer(self):
        # Advance by elapsed sim-time delta (self.get_clock().now(), which
        # resolves to /clock under use_sim_time), never by the assumed
        # timer period -- otherwise a real-time-factor != 1.0 makes true
        # speed diverge from the target_speed param. First tick has no
        # prior sample to diff against, so it only seeds _last_time.
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            self._publish(0.0)
            return
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0:
            return

        speed = self.get_parameter('target_speed').value
        vy = self.direction * speed
        self.y += vy * dt
        # Reflect any overshoot past the bound back into range (rather than
        # clamping it away) so true average speed matches target_speed --
        # clamping silently loses up to speed*dt of travel per bounce.
        upper = self.center_y + self.half_width
        lower = self.center_y - self.half_width
        if self.y >= upper:
            self.y = upper - (self.y - upper)
            self.direction = -1.0
            vy = self.direction * speed
        elif self.y <= lower:
            self.y = lower + (lower - self.y)
            self.direction = 1.0
            vy = self.direction * speed

        self._publish(vy)

    def _publish(self, vy):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = 'target'
        msg.pose.pose.position.x = float(self.center_x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = float(self.target_z)
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.y = float(vy)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
