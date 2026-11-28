#!/usr/bin/env python3
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
"""CPU YOLO26 inference for the image estimation bench."""
import ctypes
import os
import sys

import cv2
import numpy as np


def letterbox(image):
    """Match the centred, zero-padded RGB 640-square encoder input."""
    height, width = image.shape[:2]
    scale = min(640 / width, 640 / height)
    # ROI depth documents encoder truncation and centred padding:
    # Realsense_ROI_Depth_Rectifier/src/roi_depth_node.cpp, constructor.
    w, h = int(width * scale), int(height * scale)
    resized = cv2.resize(image, (w, h), interpolation=cv2.INTER_LINEAR)
    padded = np.zeros((640, 640, 3), dtype=np.uint8)
    x, y = (640 - w) // 2, (640 - h) // 2
    padded[y:y + h, x:x + w] = resized
    return np.ascontiguousarray(padded.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0


def decode_output(library, tensor, threshold):
    """Decode output0 using the same C ABI as the robot decoder."""
    if tensor.shape != (1, 300, 18) or tensor.dtype != np.float32:
        raise ValueError(f'invalid output0: {tensor.shape}, {tensor.dtype}')
    decode = library.yolo26_decode_pose
    ptr = ctypes.POINTER(ctypes.c_float)
    decode.restype = ctypes.c_int64
    decode.argtypes = [ptr, ctypes.c_size_t, ctypes.c_int64, ctypes.c_int64,
                       ctypes.c_float, ptr, ctypes.c_size_t, ctypes.c_char_p, ctypes.c_size_t]
    tensor = np.ascontiguousarray(tensor)
    out = np.empty((300, 18), dtype=np.float32)
    error = ctypes.create_string_buffer(512)
    count = decode(tensor.ctypes.data_as(ptr), tensor.size, 300, 18, threshold,
                   out.ctypes.data_as(ptr), 300, error, len(error))
    if count < 0:
        raise ValueError(error.value.decode())
    return out[:count]


def make_node(ort):
    """Import ROS only when running the node, so helpers work on the host."""
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    from cv_bridge import CvBridge
    from dji_serial_bridge.msg import ArmorKeypoints, ArmorKeypointsArray
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class Inference(Node):
        """Publish one decoded array for each image, including empty arrays."""

        def __init__(self):
            super().__init__('onnx_armor_inference')
            model = self.declare_parameter('model_onnx', '').value
            threshold = self.declare_parameter('confidence_threshold', 0.25).value
            self.threshold = float(threshold)
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            self.session = ort.InferenceSession(
                model, sess_options=options, providers=['CPUExecutionProvider'])
            inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
            if (len(inputs) != 1 or inputs[0].name != 'images' or
                    inputs[0].shape != [1, 3, 640, 640]):
                raise ValueError('model must have input images [1,3,640,640]')
            if (len(outputs) != 1 or outputs[0].name != 'output0' or
                    outputs[0].shape != [1, 300, 18]):
                raise ValueError('model must have output0 [1,300,18]')
            try:
                path = os.path.join(get_package_prefix('realsense_yolov8_nitros_bridge'), 'lib',
                                    'librealsense_yolov8_nitros_bridge_yolo26_c.so')
                self.library = ctypes.CDLL(path)
            except (PackageNotFoundError, OSError) as exc:
                raise RuntimeError(
                    'B1 decoder missing; run sim/scripts/image_bench_deps.sh') from exc
            self.bridge = CvBridge()
            self.pub = self.create_publisher(ArmorKeypointsArray, '/cv/armor_keypoints', 10)
            self.sub = self.create_subscription(Image, '/color/image_raw', self.image,
                                                10)

        def image(self, msg):
            """Decode using the shared robot decoder and preserve capture time."""
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            tensor = self.session.run(['output0'], {'images': letterbox(image)})[0]
            rows = decode_output(self.library, tensor, self.threshold)
            result = ArmorKeypointsArray()
            result.header = msg.header
            for row in rows:
                det = ArmorKeypoints()
                det.header = msg.header
                det.center_x, det.center_y, det.size_x, det.size_y = map(float, row[:4])
                det.confidence, det.class_id = float(row[4]), int(row[5])
                det.has_keypoints = True
                for point, values in zip(det.keypoints, row[6:].reshape(4, 3)):
                    point.x, point.y, point.z = map(float, values)
                result.detections.append(det)
            self.pub.publish(result)

    return Inference()


def main():
    """Run inference until shutdown."""
    try:
        import onnxruntime as ort
    except ImportError as exc:
        print('ONNX Runtime missing; run sim/scripts/image_bench_deps.sh', file=sys.stderr)
        raise SystemExit(1) from exc
    import rclpy
    rclpy.init()
    node = None
    try:
        node = make_node(ort)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
