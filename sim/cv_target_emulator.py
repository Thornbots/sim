"""
Ground-truth-plus-noise CV emulator (no YOLOv8 render pipeline) for a
4-armor-panel target *robot*, not a single point. Subscribes
/sim/raw_odom + /sim/raw_joint_states (camera FK, no TF) and
/target/ground_truth_odom (chassis center + yaw, see target_driver.py);
derives the 4 panel poses from a fixed layout (panel_radius, spaced 90
degrees apart) rotated by the chassis yaw. A panel only "presents" (is
detectable) when its outward normal points toward the camera within
panel_view_half_angle, matching a real armor panel's LED/retroreflector
being visible only from roughly in front of it -- so as the chassis spins
(target_driver's spin_hz), which panel presents keeps changing, same as
ARCC_2026_SENTRY_CONTEXT.md's "Opponent robot characteristics" section
describes. Among presenting panels also inside the camera's FOV/range, the
most head-on one is published on cv/panel_detection (PanelDetection) --
exactly what point_to_cv_target.py subscribes to. ALL qualifying panels
(not just the most head-on) are also published on cv/panel_detections
(PanelDetectionArray) -- the real roi_depth_node's output shape, kept
alongside the single-panel topic for the transition (see README.md).
Corners are the panel's
true PANEL_SIZE square (ground truth, not depth-approximated like the
real roi_depth_node), built from its outward normal so they carry the
same S122 cant. Target position is REP-103 relative to the camera
(x=forward, y=left, z=up), NOT optical -- point_to_cv_target.on_panel
expects that convention. Publishes
nothing when no panel qualifies (track loss). Also publishes MarkerArray on
target_markers (world frame: green sphere = chassis center, small cyan
boxes = all 4 panels, yellow sphere = the currently-selected noisy
detection, yellow absent when nothing qualifies) for rviz visualization
only -- not consumed by point_to_cv_target. See README.md's ## Notes for
the FK chain, dwell-count guard, and REP-103-vs-optical rationale.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Point32
from dji_serial_bridge.msg import PanelDetection, PanelDetectionArray
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


def _quat_from_axes(x_axis, y_axis, z_axis):
    """Quaternion (x, y, z, w) for the rotation whose local +X/+Y/+Z map to
    the given orthonormal world-frame axes -- used to orient panel/
    detection boxes so their thin (normal) axis actually points along
    panel_normal's real tilt (both azimuth AND the S122 cant), not just
    yaw, which a flush atan2(normal.y, normal.x) discards."""
    r = np.column_stack([x_axis, y_axis, z_axis])
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


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

# 4 armor panels spaced 90 degrees apart around the chassis center (front,
# left, back, right), matching a standard RoboMaster-class robot's layout
# per ARCC_2026_SENTRY_CONTEXT.md's "Opponent robot characteristics". Not
# an authoritative extracted spec (this repo's rulebook notes don't have
# exact construction dimensions yet -- see ARCC_2026_SENTRY_CONTEXT.md's
# "Not yet extracted" list) -- panel_radius_x/y below approximate a public
# RoboMaster Standard-class footprint (~600mm front-back, ~480mm
# left-right) rather than a single square layout.
_PANEL_OFFSETS_RAD = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
_PANEL_NAMES = ('front', 'left', 'back', 'right')
_PANEL_USES_RADIUS_X = (True, False, True, False)  # front/back vs left/right
# Small Armor Module (Standard-class, most ARCC opponents) approximated as
# a flat 0.1m x 0.1m square. NOT sourced from ARCC_2026_SENTRY_CONTEXT.md --
# that doc gives mounting angle/height/offsets but no panel face dimensions
# (checked 2026-07-29). This is the pass/fail line for run_shot_hit_tests.py's
# DEFAULT_HIT_RADIUS = PANEL_SIZE/2, so treat any hit-rate number as
# calibrated on an approximation, not a confirmed spec, until a real
# dimension is found.
PANEL_SIZE = 0.1
# S122 (ARCC_2026_SENTRY_CONTEXT.md "Mounting angle"): panel outward normal
# makes a 75-degree angle with straight-up, i.e. canted ~15 degrees off
# pure-horizontal (90 degrees would be flush-vertical) -- not the z=0
# flush-vertical assumption used before this was confirmed.
PANEL_NORMAL_ANGLE_FROM_UP = math.radians(75.0)


class CvTargetEmulator(Node):
    def __init__(self):
        super().__init__('cv_target_emulator')

        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('horizontal_fov', 1.5184)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('range_near', 0.1)
        self.declare_parameter('range_far', 10.0)
        # Restored to non-zero (plan Phase 6) so run_shot_hit_tests.py's
        # hit-rate is transferable rather than a noiseless-sim artifact.
        # publish_latency_s=0.06 is a placeholder pending a real measurement
        # from point_to_cv_target's LatencyStat (plan verification item 9) --
        # the plan's own estimate is pipeline latency ~50-100ms.
        self.declare_parameter('noise_pos_stddev', 0.03)
        self.declare_parameter('dropout_probability', 0.1)
        self.declare_parameter('publish_latency_s', 0.06)
        self.declare_parameter('yaw_joint_name', 'headlink')
        self.declare_parameter('pitch_joint_name', 'headpitch')
        self.declare_parameter('panel_radius_x', 0.30)  # front/back, ~600mm chassis length / 2
        self.declare_parameter('panel_radius_y', 0.24)  # left/right, ~480mm chassis width / 2
        self.declare_parameter('panel_view_half_angle', math.radians(75.0))

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
        self._target_rot = None
        self._target_frame_id = None
        self._pending = []  # [dict(publish_at, sample_stamp, single, array)]

        # In-frustum dwell tracking (per README.md's dwell-count guard --
        # the test script counts these via panel_detection arrival timing;
        # this log line is the human-readable equivalent).
        self._dwell_count = 0

        self.panel_pub = self.create_publisher(PanelDetection, 'cv/panel_detection', 10)
        self.panel_array_pub = self.create_publisher(
            PanelDetectionArray, 'cv/panel_detections', 10)
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
        q = msg.pose.pose.orientation
        self._target_pos = np.array([p.x, p.y, p.z])
        self._target_rot = _rotation_from_quaternion(q.x, q.y, q.z, q.w)
        self._target_frame_id = msg.header.frame_id

    def _panel_poses(self):
        """World (position, outward_normal_unit_vector, right_dir, up_dir)
        for each of the 4 armor panels, chassis yaw applied via the
        target's own rotation matrix -- see module docstring for the panel
        layout.

        Position offset stays in the horizontal chassis plane (radius_x/y
        place the panel's center on the correct side face), but the
        outward normal is canted per S122 -- PANEL_NORMAL_ANGLE_FROM_UP
        from straight-up, not flush-horizontal (z=0). right_dir/up_dir are
        an orthonormal in-plane basis (built from the normal) used to place
        the panel's 4 corners -- up_dir inherits the same cant as the
        normal, matching a real rigid panel."""
        radius_x = self.get_parameter('panel_radius_x').value
        radius_y = self.get_parameter('panel_radius_y').value
        world_up = np.array([0.0, 0.0, 1.0])
        poses = []
        for offset, use_x in zip(_PANEL_OFFSETS_RAD, _PANEL_USES_RADIUS_X):
            radius = radius_x if use_x else radius_y
            horiz_dir = np.array([math.cos(offset), math.sin(offset), 0.0])
            world_horiz = self._target_rot @ horiz_dir
            panel_pos = self._target_pos + radius * world_horiz

            local_normal = np.array([
                math.sin(PANEL_NORMAL_ANGLE_FROM_UP) * math.cos(offset),
                math.sin(PANEL_NORMAL_ANGLE_FROM_UP) * math.sin(offset),
                math.cos(PANEL_NORMAL_ANGLE_FROM_UP),
            ])
            world_normal = self._target_rot @ local_normal

            right_dir = np.cross(world_up, world_normal)
            right_dir /= np.linalg.norm(right_dir)
            up_dir = np.cross(world_normal, right_dir)

            poses.append((panel_pos, world_normal, right_dir, up_dir))
        return poses

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

    def _make_detection(self, cand, cam_pos, cam_rot):
        """Build one noisy PanelDetection from a qualifying candidate tuple,
        or None if this draw dropped out. Same noise/corner construction as
        the single-best path used before this was split out for the array."""
        _, fwd, left, up, panel_pos, panel_normal, right_dir, up_dir, panel_idx = cand
        if np.random.uniform() < self.get_parameter('dropout_probability').value:
            return None

        stddev = self.get_parameter('noise_pos_stddev').value
        fwd_n = fwd + np.random.normal(0.0, stddev)
        left_n = left + np.random.normal(0.0, stddev)
        up_n = up + np.random.normal(0.0, stddev)

        half = PANEL_SIZE / 2.0
        noise = np.array([fwd_n - fwd, left_n - left, up_n - up])
        corners_world = [
            panel_pos - half * right_dir + half * up_dir,  # TL
            panel_pos + half * right_dir + half * up_dir,  # TR
            panel_pos + half * right_dir - half * up_dir,  # BR
            panel_pos - half * right_dir - half * up_dir,  # BL
        ]

        def to_point32(world_pt):
            rel_cam = cam_rot.T @ (world_pt - cam_pos) + noise
            return Point32(x=float(rel_cam[0]), y=float(rel_cam[1]), z=float(rel_cam[2]))

        detection = PanelDetection()
        detection.corners = [to_point32(c) for c in corners_world]
        detection.center = Point32(x=fwd_n, y=left_n, z=up_n)
        detection.depth_m = fwd_n
        detection.confidence = 1.0
        detection.class_id = panel_idx
        return detection

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
        panels = self._panel_poses()
        world_frame = self._target_frame_id or self._root_frame_id or 'odom'

        near = self.get_parameter('range_near').value
        far = self.get_parameter('range_far').value
        max_view_angle = self.get_parameter('panel_view_half_angle').value

        # Every panel that qualifies (in range, in FOV, presenting within
        # max_view_angle) -- not just the most head-on one, so the array
        # output below carries all simultaneously-visible panels the way
        # the real roi_depth_node would from one YOLO frame.
        # panel_idx (0-3, matching _PANEL_OFFSETS_RAD's front/left/back/right
        # order) stands in for class_id so a spinning target's visible panel
        # actually changes id as it rotates -- target_tracker.py's
        # SpinDetector keys entirely off class_id changes, so without this
        # every detection would report class_id=0 forever and spin
        # detection would be silently untestable against this emulator (a
        # real 8-class team+plate-digit id isn't needed for that, just a
        # value that changes with which panel is visible).
        qualifying = []  # [(view_angle, fwd, left, up, panel_pos, panel_normal, right_dir, up_dir, panel_idx)]
        for panel_idx, (panel_pos, panel_normal, right_dir, up_dir) in enumerate(panels):
            # REP-103 (fwd, left, up) relative to the camera -- see module
            # docstring for why this convention, not optical.
            rel_world = panel_pos - cam_pos
            rel_cam = cam_rot.T @ rel_world
            fwd, left, up = float(rel_cam[0]), float(rel_cam[1]), float(rel_cam[2])

            in_range = near <= fwd <= far
            bearing_h = math.atan2(left, fwd) if fwd > 0 else math.pi
            bearing_v = math.atan2(up, fwd) if fwd > 0 else math.pi
            in_fov = (abs(bearing_h) <= self.hfov / 2.0
                      and abs(bearing_v) <= self.vfov / 2.0)
            if not (in_range and in_fov):
                continue

            # A panel only "presents" within max_view_angle of head-on --
            # real armor-panel LEDs/retroreflectors aren't visible edge-on.
            to_camera = -rel_world / (np.linalg.norm(rel_world) + 1e-9)
            view_angle = math.acos(np.clip(np.dot(panel_normal, to_camera), -1.0, 1.0))
            if view_angle > max_view_angle:
                continue

            qualifying.append((view_angle, fwd, left, up, panel_pos, panel_normal,
                               right_dir, up_dir, panel_idx))

        if not qualifying:
            if self._dwell_count > 0:
                self.get_logger().info(
                    f"no panel presenting after {self._dwell_count} consecutive samples")
            self._dwell_count = 0
            self._publish_markers(world_frame, panels, cam_pos, cam_rot, detected_world=None)
            return

        best = min(qualifying, key=lambda c: c[0])
        single = self._make_detection(best, cam_pos, cam_rot)

        if single is None:
            self._dwell_count = 0  # dropout breaks the dwell run too
            self._publish_markers(world_frame, panels, cam_pos, cam_rot, detected_world=None)
        else:
            self._dwell_count += 1
            _, _, _, _, panel_pos, panel_normal, right_dir, up_dir, _ = best
            # World-frame position of the noisy detection, purely for the
            # rviz markers below -- the actual panel_detection payload stays
            # camera-relative REP-103, unaffected by this.
            detected_world = cam_pos + cam_rot @ np.array(
                [single.center.x, single.center.y, single.center.z])
            self._publish_markers(
                world_frame, panels, cam_pos, cam_rot, detected_world,
                panel_normal, right_dir, up_dir)

        # Independent noise/dropout draw per candidate -- each panel is an
        # independent detection, same as the real pipeline treating each
        # YOLO box separately.
        array_detections = [d for d in
                             (self._make_detection(c, cam_pos, cam_rot) for c in qualifying)
                             if d is not None]

        # Stamp at sample time (now), not flush time -- see _flush_pending.
        # publish_latency_s is purely the delivery delay from here on.
        sample_stamp = self.get_clock().now().to_msg()
        latency_s = self.get_parameter('publish_latency_s').value
        publish_at = self.get_clock().now() + rclpy.duration.Duration(seconds=latency_s)
        self._pending.append({
            'publish_at': publish_at,
            'sample_stamp': sample_stamp,
            'single': single,
            'array': array_detections,
        })

    def _publish_markers(self, frame_id, panels, cam_pos, cam_rot, detected_world,
                          detected_normal=None, detected_right=None, detected_up=None):
        """rviz visualization only -- chassis center (green) always shown,
        all 4 panels (dim cyan boxes) always shown so spin is visible even
        when nothing currently presents, noisy-detected (yellow) shown only
        while a panel actually qualified and wasn't dropped, so losing
        track is visible as the yellow box disappearing rather than
        freezing in place -- drawn as a PANEL_SIZE box (like the ground
        truth panels), not a point, since a detection is a panel-sized
        region, not a single point in space. A white arrow from the camera
        along its current aim direction (cam_rot's local +Z) shows where
        the head is looking, regardless of whether anything currently
        presents."""
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        aim = Marker()
        aim.header.frame_id = frame_id
        aim.header.stamp = now
        aim.ns = 'cv_target_aim'
        aim.id = 0
        aim.type = Marker.ARROW
        aim.action = Marker.ADD
        # Camera-local +X is forward, not +Z -- see the REP-103 conversion
        # above (rel_cam[0] is called 'fwd').
        aim_dir = cam_rot @ np.array([1.0, 0.0, 0.0])
        start = cam_pos
        end = cam_pos + aim_dir * self.get_parameter('range_far').value
        aim.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                      Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
        aim.scale.x = 0.03  # shaft diameter
        aim.scale.y = 0.06  # head diameter
        aim.scale.z = 0.0   # head length, 0 = auto
        aim.color.r = aim.color.g = aim.color.b = 1.0
        aim.color.a = 0.5
        markers.markers.append(aim)

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
        gt.color.b = 1.0
        gt.color.a = 0.6
        markers.markers.append(gt)

        for i, (panel_pos, panel_normal, right_dir, up_dir) in enumerate(panels):
            panel = Marker()
            panel.header.frame_id = frame_id
            panel.header.stamp = now
            panel.ns = 'cv_target_panels'
            panel.id = i
            panel.type = Marker.CUBE
            panel.action = Marker.ADD
            panel.pose.position.x, panel.pose.position.y, panel.pose.position.z = panel_pos.tolist()
            qx, qy, qz, qw = _quat_from_axes(panel_normal, right_dir, up_dir)
            panel.pose.orientation.x = qx
            panel.pose.orientation.y = qy
            panel.pose.orientation.z = qz
            panel.pose.orientation.w = qw
            panel.scale.x = 0.02
            panel.scale.y = PANEL_SIZE
            panel.scale.z = PANEL_SIZE
            panel.color.b = 1.0
            panel.color.g = 1.0
            panel.color.a = 0.5
            markers.markers.append(panel)

        det = Marker()
        det.header.frame_id = frame_id
        det.header.stamp = now
        det.ns = 'cv_target'
        det.id = 1
        det.type = Marker.CUBE
        if detected_world is None:
            det.action = Marker.DELETE
        else:
            det.action = Marker.ADD
            det.pose.position.x, det.pose.position.y, det.pose.position.z = detected_world.tolist()
            qx, qy, qz, qw = _quat_from_axes(detected_normal, detected_right, detected_up)
            det.pose.orientation.x = qx
            det.pose.orientation.y = qy
            det.pose.orientation.z = qz
            det.pose.orientation.w = qw
            det.scale.x = 0.02
            det.scale.y = det.scale.z = PANEL_SIZE
            det.color.r = 1.0
            det.color.g = 1.0
            det.color.a = 0.9
        markers.markers.append(det)

        self.marker_pub.publish(markers)

    def _flush_pending(self):
        now = self.get_clock().now()
        still_pending = []
        for item in self._pending:
            if now >= item['publish_at']:
                stamp = item['sample_stamp']  # sample time, not flush time
                if item['single'] is not None:
                    item['single'].header.stamp = stamp
                    item['single'].header.frame_id = 'camera'
                    self.panel_pub.publish(item['single'])

                array_msg = PanelDetectionArray()
                array_msg.header.stamp = stamp
                array_msg.header.frame_id = 'camera'
                for d in item['array']:
                    d.header.stamp = stamp
                    d.header.frame_id = 'camera'
                array_msg.detections = item['array']
                self.panel_array_pub.publish(array_msg)
            else:
                still_pending.append(item)
        self._pending = still_pending


def main(args=None):
    rclpy.init(args=args)
    node = CvTargetEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
