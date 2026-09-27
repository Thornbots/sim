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
C2 estimation-bench machinery: scores target_tracker's TargetState against truth.

estimation.launch.py runs bench_world (the clock, the phantom target, our
chassis and head, and detections off our head's camera; C++, no gz) and the
real Part 2 (target_selector, target_tracker). point_to_cv_target and
bench_world's head controller keep the head on the target; nothing fires. Each TargetState is
compared with bench_world's truth at its own header.stamp, so a wrong stamp
scores as error. Importable only -- test_estimation.py holds the assertions.
see ../../README.md for design rationale
"""
import json
import math
import os

from dji_serial_bridge.msg import TargetState
from estimation_metrics import METRICS, state_errors, summarize_case
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from shot_hit_harness import (
    interpolate, LaunchTree, PANEL_RADIUS_X, PANEL_RADIUS_Y, SimTimeNode, TARGET_PATHS,
)
from sim.cv_head_aim_core import solve_head_angles
from std_msgs.msg import Float64, Header

DEFAULT_DURATION = 30.0  # sim seconds scored per case, after SETTLE_S
SETTLE_S = 3.0  # sim seconds after the track restarts before steady-state scoring
# Detections off this long at each case start, past target_tracker's
# track_max_gap_s (0.5), so every case starts a fresh track. The sampler
# points the head at the true target meanwhile: the head controller holds with no
# target, and a head left at the last case's path end can miss the next.
RESET_S = 1.0
HEADPITCH_LIMIT = 0.6  # rad, cv_head_aim's clamp
TRUTH_HISTORY_S = 2.0
DEFAULT_LOG_DIR = '/tmp/estimation_test_logs'
# --blackout: blackout_s of no detections every period, shorter than
# track_max_gap_s, so the tracker coasts and must pick the target up again.
BLACKOUT = {'blackout_period_s': 2.0, 'blackout_s': 0.3}
# --shooter-speed drives our chassis along y within this of the origin.
SHOOTER_HALF_WIDTH = 1.0
SHOOTER_CMD_PERIOD_S = 0.05
# Steady-state p95 limits per cell, metric -> m, rad, m/s or rad/s: the worst
# of five runs (2026-09-26, Humble, bench_world paced on the tracker's input)
# x 1.25, floored at 0.01, printed by tools/estimation_limits.py. A cell not
# here only reports (CV_SPLIT_PLAN.md 2.0). Comments list each run's p95.
LIMITS = {
    'flat-speed0.5-lateral-shooter0': {
        'facing_panel_m': 0.1195,  # 0.0888, 0.0914, 0.0956, 0.0775, 0.0806
        'panel_m': 0.1194,  # 0.0929, 0.0938, 0.0955, 0.0855, 0.0869
        'center_m': 0.1161,  # 0.0903, 0.0914, 0.0929, 0.0827, 0.0835
        'center_along_m': 0.0544,  # 0.0394, 0.0350, 0.0358, 0.0435, 0.0389
        'center_across_m': 0.1130,  # 0.0886, 0.0888, 0.0904, 0.0774, 0.0780
        'velocity_m_s': 0.7495,  # 0.5798, 0.5865, 0.5996, 0.5417, 0.5540
        'yaw_rad': 0.1950,  # 0.1413, 0.1478, 0.1560, 0.1431, 0.1514
        'yaw_rate_rad_s': 0.3974,  # 0.3099, 0.3122, 0.3179, 0.2900, 0.3135
        'radius_m': 0.0446,  # 0.0317, 0.0311, 0.0357, 0.0249, 0.0278
        'z_offset_m': 0.0100,  # 0.0019, 0.0020, 0.0016, 0.0015, 0.0015
    },
    'flat-speed1-lateral-shooter0': {
        'facing_panel_m': 0.1077,  # 0.0762, 0.0862, 0.0858, 0.0846, 0.0833
        'panel_m': 0.1196,  # 0.0868, 0.0925, 0.0957, 0.0956, 0.0907
        'center_m': 0.1167,  # 0.0851, 0.0909, 0.0934, 0.0933, 0.0869
        'center_along_m': 0.0788,  # 0.0543, 0.0630, 0.0552, 0.0566, 0.0517
        'center_across_m': 0.1010,  # 0.0724, 0.0741, 0.0772, 0.0808, 0.0744
        'velocity_m_s': 0.9714,  # 0.7326, 0.7647, 0.7594, 0.7683, 0.7771
        'yaw_rad': 0.1964,  # 0.1291, 0.1266, 0.1467, 0.1571, 0.1406
        'yaw_rate_rad_s': 0.3986,  # 0.2884, 0.2677, 0.3098, 0.3189, 0.2923
        'radius_m': 0.0248,  # 0.0175, 0.0193, 0.0196, 0.0198, 0.0190
        'z_offset_m': 0.0100,  # 0.0016, 0.0014, 0.0021, 0.0020, 0.0017
    },
    'flat-speed2-lateral-shooter0': {
        'facing_panel_m': 0.1509,  # 0.1087, 0.1207, 0.1112, 0.1109, 0.1123
        'panel_m': 0.1535,  # 0.1121, 0.1228, 0.1148, 0.1161, 0.1182
        'center_m': 0.1522,  # 0.1110, 0.1218, 0.1143, 0.1155, 0.1175
        'center_along_m': 0.1279,  # 0.0933, 0.1023, 0.1005, 0.0943, 0.0977
        'center_across_m': 0.1050,  # 0.0786, 0.0840, 0.0770, 0.0830, 0.0772
        'velocity_m_s': 1.2836,  # 0.9845, 1.0260, 0.9837, 1.0269, 0.9899
        'yaw_rad': 0.1894,  # 0.1263, 0.1375, 0.1373, 0.1428, 0.1515
        'yaw_rate_rad_s': 0.3830,  # 0.2621, 0.2896, 0.2918, 0.2724, 0.3064
        'radius_m': 0.0302,  # 0.0194, 0.0242, 0.0204, 0.0185, 0.0191
        'z_offset_m': 0.0100,  # 0.0017, 0.0019, 0.0018, 0.0016, 0.0019
    },
    'flat-speed4-lateral-shooter0': {
        'facing_panel_m': 0.3016,  # 0.1407, 0.1559, 0.2413, 0.1376, 0.1304
        'panel_m': 0.2968,  # 0.1351, 0.1499, 0.2374, 0.1379, 0.1279
        'center_m': 0.2824,  # 0.1315, 0.1425, 0.2259, 0.1325, 0.1234
        'center_along_m': 0.1806,  # 0.1022, 0.0990, 0.1445, 0.0974, 0.0893
        'center_across_m': 0.2459,  # 0.1137, 0.1282, 0.1967, 0.1146, 0.1029
        'velocity_m_s': 2.0889,  # 1.0784, 1.0670, 1.6711, 1.0940, 1.0757
        'yaw_rad': 0.3009,  # 0.1627, 0.1604, 0.2407, 0.1480, 0.1452
        'yaw_rate_rad_s': 0.7537,  # 0.3727, 0.3621, 0.6030, 0.3635, 0.3493
        'radius_m': 0.1313,  # 0.0531, 0.0631, 0.1050, 0.0439, 0.0441
        'z_offset_m': 0.0100,  # 0.0024, 0.0029, 0.0037, 0.0021, 0.0020
    },
    'flat-stationary-lateral-shooter0': {
        'facing_panel_m': 0.0101,  # 0.0081, 0.0078, 0.0073, 0.0073, 0.0073
        'panel_m': 0.0663,  # 0.0530, 0.0474, 0.0398, 0.0452, 0.0397
        'center_m': 0.0594,  # 0.0475, 0.0425, 0.0333, 0.0383, 0.0334
        'center_along_m': 0.0574,  # 0.0459, 0.0425, 0.0320, 0.0358, 0.0330
        'center_across_m': 0.0100,  # 0.0031, 0.0028, 0.0037, 0.0031, 0.0031
        'velocity_m_s': 0.0100,  # 0.0000, 0.0000, 0.0000, 0.0000, 0.0000
        'yaw_rad': 0.0100,  # 0.0047, 0.0044, 0.0047, 0.0049, 0.0047
        'yaw_rate_rad_s': 0.0100,  # 0.0000, 0.0000, 0.0000, 0.0000, 0.0000
        'radius_m': 0.0545,  # 0.0436, 0.0398, 0.0317, 0.0366, 0.0325
        'z_offset_m': 0.0164,  # 0.0119, 0.0011, 0.0088, 0.0131, 0.0052
    },
    'staggered-speed0.5-lateral-shooter0': {
        'facing_panel_m': 0.1140,  # 0.0902, 0.0834, 0.0912, 0.0800, 0.0887
        'panel_m': 0.1247,  # 0.0943, 0.0890, 0.0940, 0.0887, 0.0998
        'center_m': 0.1205,  # 0.0924, 0.0854, 0.0901, 0.0847, 0.0964
        'center_along_m': 0.0478,  # 0.0359, 0.0382, 0.0362, 0.0358, 0.0350
        'center_across_m': 0.1136,  # 0.0870, 0.0785, 0.0854, 0.0796, 0.0909
        'velocity_m_s': 0.7554,  # 0.5945, 0.5404, 0.5837, 0.5432, 0.6043
        'yaw_rad': 0.2047,  # 0.1395, 0.1638, 0.1570, 0.1596, 0.1505
        'yaw_rate_rad_s': 0.4171,  # 0.3106, 0.3188, 0.3168, 0.3337, 0.3278
        'radius_m': 0.0361,  # 0.0285, 0.0275, 0.0289, 0.0266, 0.0276
        'z_offset_m': 0.0100,  # 0.0031, 0.0028, 0.0026, 0.0034, 0.0035
    },
    'staggered-speed1-lateral-shooter0': {
        'facing_panel_m': 0.1179,  # 0.0943, 0.0876, 0.0908, 0.0870, 0.0853
        'panel_m': 0.1254,  # 0.0984, 0.0968, 0.1003, 0.0900, 0.0924
        'center_m': 0.1208,  # 0.0963, 0.0942, 0.0966, 0.0882, 0.0904
        'center_along_m': 0.0805,  # 0.0593, 0.0572, 0.0644, 0.0562, 0.0572
        'center_across_m': 0.1016,  # 0.0778, 0.0788, 0.0813, 0.0756, 0.0763
        'velocity_m_s': 1.0187,  # 0.8091, 0.8150, 0.7764, 0.7514, 0.7480
        'yaw_rad': 0.1946,  # 0.1347, 0.1337, 0.1413, 0.1557, 0.1347
        'yaw_rate_rad_s': 0.4074,  # 0.2864, 0.2867, 0.3004, 0.3259, 0.3165
        'radius_m': 0.0244,  # 0.0195, 0.0188, 0.0184, 0.0187, 0.0194
        'z_offset_m': 0.0100,  # 0.0027, 0.0022, 0.0027, 0.0030, 0.0026
    },
    'staggered-speed2-lateral-shooter0': {
        'facing_panel_m': 0.1476,  # 0.1181, 0.1154, 0.1107, 0.1134, 0.1167
        'panel_m': 0.1507,  # 0.1197, 0.1206, 0.1124, 0.1164, 0.1170
        'center_m': 0.1487,  # 0.1190, 0.1169, 0.1108, 0.1148, 0.1162
        'center_along_m': 0.1232,  # 0.0986, 0.0982, 0.0949, 0.0954, 0.0939
        'center_across_m': 0.1071,  # 0.0857, 0.0804, 0.0775, 0.0802, 0.0784
        'velocity_m_s': 1.2671,  # 1.0131, 1.0122, 1.0137, 0.9949, 0.9986
        'yaw_rad': 0.1932,  # 0.1496, 0.1546, 0.1423, 0.1440, 0.1519
        'yaw_rate_rad_s': 0.4108,  # 0.3010, 0.3177, 0.2850, 0.3045, 0.3286
        'radius_m': 0.0252,  # 0.0202, 0.0171, 0.0163, 0.0184, 0.0193
        'z_offset_m': 0.0100,  # 0.0030, 0.0026, 0.0024, 0.0030, 0.0022
    },
    'staggered-speed4-lateral-shooter0': {
        'facing_panel_m': 0.3427,  # 0.1570, 0.1936, 0.1293, 0.1648, 0.2742
        'panel_m': 0.3514,  # 0.1481, 0.1740, 0.1234, 0.1634, 0.2811
        'center_m': 0.3335,  # 0.1438, 0.1657, 0.1210, 0.1546, 0.2668
        'center_along_m': 0.2251,  # 0.1064, 0.1162, 0.0957, 0.1017, 0.1801
        'center_across_m': 0.2609,  # 0.1207, 0.1438, 0.0961, 0.1393, 0.2087
        'velocity_m_s': 2.5575,  # 1.1762, 1.2538, 1.0489, 1.2148, 2.0460
        'yaw_rad': 0.3299,  # 0.1617, 0.1761, 0.1338, 0.1764, 0.2639
        'yaw_rate_rad_s': 1.0948,  # 0.3691, 0.4387, 0.3230, 0.4163, 0.8758
        'radius_m': 0.1486,  # 0.0573, 0.0817, 0.0318, 0.0617, 0.1189
        'z_offset_m': 0.0274,  # 0.0029, 0.0028, 0.0019, 0.0029, 0.0219
    },
    'staggered-stationary-lateral-shooter0': {
        'facing_panel_m': 0.0240,  # 0.0165, 0.0080, 0.0192, 0.0175, 0.0083
        'panel_m': 0.1536,  # 0.0854, 0.1229, 0.0469, 0.0936, 0.0970
        'center_m': 0.1321,  # 0.0794, 0.1057, 0.0439, 0.0837, 0.0824
        'center_along_m': 0.0465,  # 0.0125, 0.0245, 0.0086, 0.0115, 0.0372
        'center_across_m': 0.1231,  # 0.0793, 0.0985, 0.0439, 0.0829, 0.0583
        'velocity_m_s': 0.0100,  # 0.0000, 0.0000, 0.0000, 0.0000, 0.0000
        'yaw_rad': 0.4135,  # 0.2385, 0.3308, 0.1210, 0.2699, 0.1900
        'yaw_rate_rad_s': 0.0100,  # 0.0000, 0.0000, 0.0000, 0.0000, 0.0000
        'radius_m': 0.0750,  # 0.0600, 0.0358, 0.0390, 0.0600, 0.0407
        'z_offset_m': 0.0583,  # 0.0017, 0.0327, 0.0018, 0.0014, 0.0466
    },
}


class EstimationSampler(SimTimeNode):
    """Collects TargetStates and scores each once truth covers its stamp."""

    def __init__(self, stagger, shooter_speed=0.0):
        super().__init__('estimation_sampler')
        self.stagger = stagger
        self.shooter_speed = shooter_speed
        self._shooter_dir = 1.0
        self._last_cmd_s = None
        self._truth = []  # [(stamp_s, center, velocity, yaw unwrapped, yaw_rate)]
        self._root_xy = (0.0, 0.0)  # ours, from /sim/raw_odom
        self._root_pose = (np.zeros(3), 0.0)  # position, yaw
        self.aim_head = False  # command the head at truth; run_case sets it for RESET_S
        self._pending = []  # [(stamp_s, arrival_s, TargetState)]
        self.records = []  # one dict per scored state
        self.create_subscription(Odometry, '/target/ground_truth_odom', self._on_truth, 50)
        self.create_subscription(TargetState, '/cv/target_state', self._on_state, 50)
        self.create_subscription(Odometry, '/sim/raw_odom', self._on_root_odom, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_pub = self.create_publisher(Float64, '/head_pan_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/head_pitch_cmd', 10)
        # The truth stamp scored up to, so sim_clock's paced mode (rate 0)
        # doesn't run ahead of this scorer.
        self.progress_pub = self.create_publisher(Header, '/bench/progress', 10)

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_truth(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        stamp = self._stamp_s(msg.header.stamp)
        yaw = 2.0 * math.atan2(q.z, q.w)  # bench_world publishes yaw-only
        if self._truth:
            prev = self._truth[-1][3]
            yaw = prev + math.atan2(math.sin(yaw - prev), math.cos(yaw - prev))
        # The truth twist is world-frame, despite child_frame_id.
        self._truth.append((stamp, np.array([p.x, p.y, p.z]),
                            np.array([v.linear.x, v.linear.y, v.linear.z]),
                            yaw, v.angular.z))
        self._truth = [h for h in self._truth if stamp - h[0] <= TRUTH_HISTORY_S]
        self._resolve(stamp)
        self.progress_pub.publish(Header(stamp=msg.header.stamp))
        if self.aim_head:
            self._aim_head_at(self._truth[-1][1])

    def _aim_head_at(self, center):
        root, yaw = self._root_pose
        d = center - root
        c, s = math.cos(yaw), math.sin(yaw)
        head_yaw, head_pitch = solve_head_angles((c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]))
        self.pan_pub.publish(Float64(data=head_yaw))
        self.pitch_pub.publish(Float64(data=max(-HEADPITCH_LIMIT,
                                                min(HEADPITCH_LIMIT, head_pitch))))

    def _on_state(self, msg):
        self._pending.append((self._stamp_s(msg.header.stamp), self._now_s(), msg))

    def _resolve(self, truth_s):
        keep = []
        for stamp, arrival, msg in self._pending:
            if stamp > truth_s:
                keep.append((stamp, arrival, msg))
                continue
            if stamp < self._truth[0][0]:
                continue  # older than the truth kept; can't score it
            center, vel, yaw, yaw_rate = interpolate(self._truth, stamp)
            rec = state_errors(msg, (center, vel, yaw, yaw_rate), self.stagger,
                               (PANEL_RADIUS_X, PANEL_RADIUS_Y), self._root_xy)
            rec.update({'t': round(stamp, 4), 'valid': bool(msg.valid),
                        'age_on_arrival_s': round(arrival - stamp, 4),
                        'track_id': int(msg.robot_track_id)})
            self.records.append(rec)
        self._pending = keep

    def _on_root_odom(self, msg):
        """Keep our position; bounce our chassis along y at shooter_speed, x held at 0."""
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._root_xy = (p.x, p.y)
        self._root_pose = (np.array([p.x, p.y, p.z]),
                           math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        if self.shooter_speed <= 0.0:
            return
        now_s = self._stamp_s(msg.header.stamp)
        if self._last_cmd_s is not None and now_s - self._last_cmd_s < SHOOTER_CMD_PERIOD_S:
            return
        self._last_cmd_s = now_s
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if y >= SHOOTER_HALF_WIDTH:
            self._shooter_dir = -1.0
        elif y <= -SHOOTER_HALF_WIDTH:
            self._shooter_dir = 1.0
        cmd = Twist()
        cmd.linear.x = max(-self.shooter_speed, min(self.shooter_speed, -2.0 * x))
        cmd.linear.y = self._shooter_dir * self.shooter_speed
        self.cmd_vel_pub.publish(cmd)

    def stop_shooter(self):
        if self.shooter_speed > 0.0:
            self.cmd_vel_pub.publish(Twist())


class EstimationStack:
    """
    The C2 stack, launched once and reused for every case.

    launch/estimation.launch.py defines it; with external=True it is already
    running (it started this pytest). Between cases only bench_world's target
    motion, panel stagger and masking change.
    """

    def __init__(self, headless, log_dir, external=False, extra_args=()):
        self.launch = None
        if not external:
            self.launch = LaunchTree(
                'stack',
                ['ros2', 'launch', 'sim', 'estimation.launch.py', 'run_tests:=false',
                 f'headless:={str(headless).lower()}', *extra_args],
                os.path.join(log_dir, 'stack.log'))
        self.states_path = os.path.join(log_dir, 'estimation_states.jsonl')
        self.summary_path = os.path.join(log_dir, 'estimation.jsonl')
        self.node = rclpy.create_node(
            'estimation_stack',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._client = self.node.create_client(SetParameters, '/bench_world/set_parameters')

    def start(self):
        for path in (self.states_path, self.summary_path):
            open(path, 'w').close()
        if self.launch is not None:
            self.launch.start()
        probe = EstimationSampler(stagger=0.0)
        try:
            probe.wait_until(lambda: bool(probe._truth), timeout=120.0,
                             description='/target/ground_truth_odom publishing')
            ready = ['bench_world', 'target_selector', 'target_tracker', 'point_to_cv_target']
            probe.wait_until(lambda: probe.nodes_up(*ready), timeout=60.0,
                             description=f'{", ".join(ready)} nodes up')
        finally:
            probe.destroy_node()

    def set_params(self, **params):
        """Set bench_world's target and detection parameters."""
        client = self._client
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/bench_world/set_parameters not available')
        msgs = []
        for k, v in params.items():
            kind = Parameter.Type.BOOL if isinstance(v, bool) else Parameter.Type.DOUBLE
            msgs.append(Parameter(k, kind, v if isinstance(v, bool) else float(v))
                        .to_parameter_msg())
        future = client.call_async(SetParameters.Request(parameters=msgs))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not all(r.successful for r in result.results):
            raise RuntimeError(f'bench_world rejected {params}')

    def stop(self):
        if self.launch is not None:
            self.launch.stop()
        self.node.destroy_node()


def run_case(stack, cell, speed, spin_hz, duration, stagger=0.0, path='lateral',
             blackout=False, shooter_speed=0.0):
    """
    Restart the track on (speed, spin_hz, stagger, path), then score every state.

    Detections go off for RESET_S so the tracker drops the old track, while
    the head turns to the target, then come back; states are scored from
    there for SETTLE_S + duration.
    """
    stack.set_params(target_speed=speed, spin_hz=spin_hz, **TARGET_PATHS[path])
    stack.set_params(panel_stagger_m=stagger, detections_enabled=False,
                     **(BLACKOUT if blackout else {'blackout_period_s': 0.0}))
    sampler = EstimationSampler(stagger, shooter_speed)
    try:
        sampler.aim_head = True
        sampler.spin_for(RESET_S)
        sampler.aim_head = False
        sampler._pending, sampler.records = [], []
        stack.set_params(detections_enabled=True)
        start_s = sampler._now_s()
        sampler.spin_for(SETTLE_S + duration)
    finally:
        sampler.stop_shooter()
        records = sampler.records
        sampler.destroy_node()
    summary = {'cell': cell, **summarize_case(records, start_s, SETTLE_S)}
    with open(stack.states_path, 'a') as f:
        for rec in records:
            f.write(json.dumps({'cell': cell, **rec}) + '\n')
    with open(stack.summary_path, 'a') as f:
        f.write(json.dumps(summary) + '\n')
    return summary


def print_summary(label, summary):
    print(f'{label}: {summary["states"]} states, '
          f'{summary.get("valid_fraction", 0):.0%} valid, '
          f'converged in {summary.get("converge_s")} s, '
          f'age on arrival {summary.get("age_on_arrival_s")} s')
    for key in METRICS:
        stat = summary.get(key)
        if stat:
            print(f'    {key:16s} mean {stat["mean"]:.4f}  p95 {stat["p95"]:.4f}')
