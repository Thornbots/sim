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
"""On-demand image pipeline integration and repeatability test."""
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


def test_image_estimation(tmp_path):
    """Each rendered frame has a result and the scored run repeats under load."""
    model = os.environ.get('IMAGE_BENCH_MODEL', '')
    if not model or not Path(model).is_file():
        pytest.skip('set IMAGE_BENCH_MODEL to an existing ONNX model')
    import rclpy
    from dji_serial_bridge.msg import ArmorKeypointsArray, PanelDetectionArray
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node('image_estimation_observer')
    frames, results, depths, keypoints = [], [], [], []

    def stamp(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    node.create_subscription(Image, '/color/image_raw',
                             lambda msg: frames.append(stamp(msg)), 1000)
    node.create_subscription(PanelDetectionArray, '/cv/panel_detections',
                             lambda msg: results.append((stamp(msg), bool(msg.detections))), 1000)
    node.create_subscription(Image, '/depth/image_rect_raw',
                             lambda msg: depths.append(stamp(msg)), 1000)
    node.create_subscription(ArmorKeypointsArray, '/cv/armor_keypoints',
                             lambda msg: keypoints.append(stamp(msg)), 1000)
    stress = None
    try:
        outputs = []
        failures = []
        for run in range(2):
            frames.clear()
            results.clear()
            depths.clear()
            keypoints.clear()
            directory = tmp_path / str(run)
            directory.mkdir()
            stress = subprocess.Popen(['python3', '-c', 'while True: pass'])
            command = ['ros2', 'launch', 'sim', 'estimation.launch.py', 'cv_source:=images',
                       'model_onnx:=' + model, 'headless:=true', 'panel_layout:=flat',
                       'only_stationary:=true', 'duration:=2', 'seed:=17',
                       'pytest_args:=--gtest_filter=*flat_stationary_lateral*',
                       'log_dir:=' + str(directory)]
            with (directory / 'launch.log').open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                try:
                    deadline = time.monotonic() + 1200
                    while process.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                    assert process.poll() is not None, 'image estimation launch timed out'
                    drain_deadline = time.monotonic() + 10
                    last_change = time.monotonic()
                    counts = None
                    while time.monotonic() < drain_deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                        new_counts = (len(frames), len(results), len(depths), len(keypoints))
                        if new_counts != counts:
                            counts, last_change = new_counts, time.monotonic()
                        elif time.monotonic() - last_change >= 1:
                            break
                    print(f'observed {len(frames)} images, {len(depths)} depths, '
                          f'{len(keypoints)} keypoint arrays and {len(results)} panel arrays, '
                          f'{sum(nonempty for _, nonempty in results)} nonempty')
                    if process.returncode != 0:
                        failures.append(f'run {run}: launch exited {process.returncode}')
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        process.wait(timeout=30)
            checked = subprocess.run(['ros2', 'run', 'sim', 'check_bench_log',
                                      str(directory / 'launch.log')], check=False)
            if checked.returncode:
                failures.append(f'run {run}: log checker failed')
            if not frames or len(frames) != len(set(frames)):
                failures.append(f'run {run}: missing or duplicate image stamps')
            if len(results) != len(frames) or {s for s, _ in results} != set(frames):
                failures.append(f'run {run}: image and result stamps differ')
            rate = sum(nonempty for _, nonempty in results) / len(results) if results else 0
            print(f'run {run}: {len(results)} frames, {rate:.1%} nonempty')
            if rate <= 0.5:
                failures.append(f'run {run}: detection rate {rate:.1%}')
            outputs.append((directory / 'estimation.jsonl').read_bytes())
            print(f'run {run}: sha256 {hashlib.sha256(outputs[-1]).hexdigest()}')
            if stress:
                stress.terminate()
                stress.wait()
                stress = None
        if outputs[0] != outputs[1]:
            failures.append('scorer output differs between runs')
        assert not failures, '; '.join(failures)
    finally:
        if stress:
            stress.terminate()
            stress.wait()
        node.destroy_node()
        rclpy.shutdown()
