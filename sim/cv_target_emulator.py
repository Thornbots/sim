"""
Ground-truth-plus-noise CV emulator (no YOLOv8 render pipeline). Subscribes
/sim/raw_odom + /sim/raw_joint_states (camera FK, no TF) and
/target/ground_truth_odom; publishes roi_point (PointStamped) + roi
(Detection2D) -- exactly what point_to_cv_target.py subscribes to. Target
position is REP-103 relative to the camera (x=forward, y=left, z=up), NOT
optical -- point_to_cv_target.on_point expects that convention. Gates on
FOV (horizontal_fov=1.5184) + range (0.1-10.0m); publishes nothing outside
(track loss). Also publishes MarkerArray on target_markers (world frame:
green sphere = ground truth, yellow sphere = noisy detection, yellow
absent when out of frustum/dropped) for rviz visualization only -- not
consumed by point_to_cv_target. See README.md's ## Notes for the FK
chain, dwell-count guard, and REP-103-vs-optical rationale.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray


def _rotation_from_quaternion(x, y, z, w):
    """3x3 rotation matrix from a geometry_msgs Quaternion's components."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def _rotation_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _rotation_axis_angle(axis, angle):
    """Rodrigues' rotation formula about a unit axis."""
    ax = np.array(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([
        [0, -ax[2], ax[1]],
        [ax[2], 0, -ax[0]],
        [-ax[1], ax[0], 0],
    ])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _transform(rot, trans):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


# Fixed joint offsets from sentry.urdf.xacro (see README.md for the full
# chain rationale) -- root -> body -> head -> head_pitch -> camera.
_T_FASTENED_2 = _transform(_rotation_from_rpy(0, 0, math.pi), (0.0, 0.0, 0.0))
_HEADLINK_ORIGIN_R = _rotation_from_rpy(0, 0, math.pi)
_HEADLINK_ORIGIN_T = (0.0, 0.0, 0.252215)
_HEADLINK_AXIS = (0.0, 0.0, -1.0)
_HEADPITCH_ORIGIN_R = _rotation_from_rpy(0, 0, -0.38885)
_HEADPITCH_ORIGIN_T = (0.1, 0.0, 0.1218)
_HEADPITCH_AXIS = (0.0, 1.0, 0.0)
# cameralink is identity -- omitted, camera frame == head_pitch frame.


class CvTargetEmulator(Node):
    def __init__(self):
        super().__init__('cv_target_emulator')

        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('horizontal_fov', 1.5184)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('range_near', 0.1)
        self.declare_parameter('range_far', 10.0)
        self.declare_parameter('noise_pos_stddev', 0.03)
        self.declare_parameter('dropout_probability', 0.0)
        self.declare_parameter('publish_latency_s', 0.0)
        self.declare_parameter('yaw_joint_name', 'headlink')
        self.declare_parameter('pitch_joint_name', 'headpitch')

        hfov = self.get_parameter('horizontal_fov').value
        aspect = (self.get_parameter('image_height').value
                  / self.get_parameter('image_width').value)
        self.vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * aspect)
        self.hfov = hfov

        self._root_pos = None
        self._root_rot = None
        self._root_frame_id = None
        self._head_yaw = 0.0
        self._head_pitch = 0.0
        self._target_pos = None
        self._target_frame_id = None
        self._pending = []  # [(publish_ros_time, PointStamped, Detection2D)]

        # In-frustum dwell tracking (per README.md's dwell-count guard --
        # the test script counts these via roi_point arrival timing; this
        # log line is the human-readable equivalent).
        self._dwell_count = 0

        self.roi_point_pub = self.create_publisher(PointStamped, 'roi_point', 10)
        self.roi_pub = self.create_publisher(Detection2D, 'roi', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'target_markers', 10)

        self.create_subscription(Odometry, '/sim/raw_odom', self.on_root_odom, 10)
        self.create_subscription(JointState, '/sim/raw_joint_states', self.on_joint_states, 10)
        self.create_subscription(Odometry, '/target/ground_truth_odom', self.on_target_odom, 10)

        rate_hz = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            f"cv_target_emulator ready: hfov={self.hfov:.3f} vfov={self.vfov:.3f} "
            f"range=[{self.get_parameter('range_near').value:.2f}, "
            f"{self.get_parameter('range_far').value:.2f}]"
        )

    def on_root_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._root_pos = np.array([p.x, p.y, p.z])
        self._root_rot = _rotation_from_quaternion(q.x, q.y, q.z, q.w)
        self._root_frame_id = msg.header.frame_id

    def on_joint_states(self, msg):
        yaw_name = self.get_parameter('yaw_joint_name').value
        pitch_name = self.get_parameter('pitch_joint_name').value
        if yaw_name in msg.name:
            self._head_yaw = msg.position[msg.name.index(yaw_name)]
        if pitch_name in msg.name:
            self._head_pitch = msg.position[msg.name.index(pitch_name)]

    def on_target_odom(self, msg):
        p = msg.pose.pose.position
        self._target_pos = np.array([p.x, p.y, p.z])
        self._target_frame_id = msg.header.frame_id

    def _camera_pose(self):
        """World position + rotation of the camera link via the fixed FK
        chain (root -> fastened_2 -> headlink(yaw) -> headpitch(pitch) ->
        cameralink), no TF lookup. See README.md for the full derivation."""
        t_root = _transform(self._root_rot, self._root_pos)
        t_body = t_root @ _T_FASTENED_2
        t_headlink = _transform(
            _HEADLINK_ORIGIN_R @ _rotation_axis_angle(_HEADLINK_AXIS, self._head_yaw),
            _HEADLINK_ORIGIN_T)
        t_head = t_body @ t_headlink
        t_headpitch = _transform(
            _HEADPITCH_ORIGIN_R @ _rotation_axis_angle(_HEADPITCH_AXIS, self._head_pitch),
            _HEADPITCH_ORIGIN_T)
        t_camera = t_head @ t_headpitch
        return t_camera[:3, 3], t_camera[:3, :3]

    def on_timer(self):
        self._flush_pending()

        if self._root_pos is None or self._target_pos is None:
            return
        if self._root_frame_id and self._target_frame_id \
                and self._root_frame_id != self._target_frame_id:
            self.get_logger().warn(
                f"/sim/raw_odom frame_id='{self._root_frame_id}' != "
                f"/target/ground_truth_odom frame_id='{self._target_frame_id}' "
                f"-- FK assumes a shared world frame.", throttle_duration_sec=5.0)

        cam_pos, cam_rot = self._camera_pose()
        # REP-103 (fwd, left, up) relative to the camera -- see module
        # docstring for why this convention, not optical.
        rel_world = self._target_pos - cam_pos
        rel_cam = cam_rot.T @ rel_world
        fwd, left, up = float(rel_cam[0]), float(rel_cam[1]), float(rel_cam[2])

        near = self.get_parameter('range_near').value
        far = self.get_parameter('range_far').value
        in_range = near <= fwd <= far
        bearing_h = math.atan2(left, fwd) if fwd > 0 else math.pi
        bearing_v = math.atan2(up, fwd) if fwd > 0 else math.pi
        in_fov = (abs(bearing_h) <= self.hfov / 2.0
                  and abs(bearing_v) <= self.vfov / 2.0)

        world_frame = self._target_frame_id or self._root_frame_id or 'odom'
        if not (in_range and in_fov):
            if self._dwell_count > 0:
                self.get_logger().info(
                    f"target left frustum after {self._dwell_count} consecutive samples")
            self._dwell_count = 0
            self._publish_markers(world_frame, detected_world=None)
            return

        if np.random.uniform() < self.get_parameter('dropout_probability').value:
            self._dwell_count = 0  # dropout breaks the dwell run too
            self._publish_markers(world_frame, detected_world=None)
            return

        stddev = self.get_parameter('noise_pos_stddev').value
        fwd_n = fwd + np.random.normal(0.0, stddev)
        left_n = left + np.random.normal(0.0, stddev)
        up_n = up + np.random.normal(0.0, stddev)
        self._dwell_count += 1

        # World-frame position of the noisy detection, purely for the rviz
        # markers below -- the actual roi_point payload (fwd_n/left_n/up_n)
        # stays camera-relative REP-103, unaffected by this.
        detected_world = cam_pos + cam_rot @ np.array([fwd_n, left_n, up_n])
        self._publish_markers(world_frame, detected_world)

        point = PointStamped()
        point.point.x = fwd_n
        point.point.y = left_n
        point.point.z = up_n

        detection = Detection2D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'target'
        hyp.hypothesis.score = 1.0
        detection.results.append(hyp)

        latency_s = self.get_parameter('publish_latency_s').value
        publish_at = self.get_clock().now() + rclpy.duration.Duration(seconds=latency_s)
        self._pending.append((publish_at, point, detection))

    def _publish_markers(self, frame_id, detected_world):
        """rviz visualization only -- ground-truth (green) always shown,
        noisy-detected (yellow) shown only while actually in-frustum and
        not dropped, so losing track is visible as the yellow sphere
        disappearing rather than freezing in place."""
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        gt = Marker()
        gt.header.frame_id = frame_id
        gt.header.stamp = now
        gt.ns = 'cv_target'
        gt.id = 0
        gt.type = Marker.SPHERE
        gt.action = Marker.ADD
        gt.pose.position.x, gt.pose.position.y, gt.pose.position.z = self._target_pos.tolist()
        gt.pose.orientation.w = 1.0
        gt.scale.x = gt.scale.y = gt.scale.z = 0.2
        gt.color.g = 1.0
        gt.color.a = 0.6
        markers.markers.append(gt)

        det = Marker()
        det.header.frame_id = frame_id
        det.header.stamp = now
        det.ns = 'cv_target'
        det.id = 1
        det.type = Marker.SPHERE
        if detected_world is None:
            det.action = Marker.DELETE
        else:
            det.action = Marker.ADD
            det.pose.position.x, det.pose.position.y, det.pose.position.z = detected_world.tolist()
            det.pose.orientation.w = 1.0
            det.scale.x = det.scale.y = det.scale.z = 0.15
            det.color.r = 1.0
            det.color.g = 1.0
            det.color.a = 0.9
        markers.markers.append(det)

        self.marker_pub.publish(markers)

    def _flush_pending(self):
        now = self.get_clock().now()
        still_pending = []
        for publish_at, point, detection in self._pending:
            if now >= publish_at:
                # Stamp from this node's own clock at actual publish time
                # (sim time under use_sim_time) -- point_to_cv_target.on_point
                # derives dt straight from this header, so any other stamp
                # source (forwarded ground truth, wall time) breaks it.
                stamp = self.get_clock().now().to_msg()
                point.header.stamp = stamp
                point.header.frame_id = 'camera'
                detection.header.stamp = stamp
                detection.header.frame_id = 'camera'
                self.roi_point_pub.publish(point)
                self.roi_pub.publish(detection)
            else:
                still_pending.append((publish_at, point, detection))
        self._pending = still_pending


def main(args=None):
    rclpy.init(args=args)
    node = CvTargetEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
