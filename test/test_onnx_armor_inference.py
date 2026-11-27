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
"""Host checks for the CPU encoder and shared decoder."""
import ctypes
import importlib.util
import os
import shutil
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT.parent / 'realsense-yolov8-nitros-bridge'
MODEL = Path(os.environ.get('ONNX_ARMOR_TEST_MODEL',
                            '/Users/blaise/ros2_ws/scratch/cv-aiming-synthetic/model-training/'
                            'runs/pose/yolo26n_pose_realsense/v1/weights/best.onnx'))
spec = importlib.util.spec_from_file_location('inference', ROOT / 'scripts/onnx_armor_inference.py')
inference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inference)


def reference(image):
    """Bilinear half-pixel resize followed by centred integer padding."""
    height, width = image.shape[:2]
    scale = min(640 / width, 640 / height)
    w, h = int(width * scale), int(height * scale)
    x = np.clip((np.arange(w) + 0.5) * width / w - 0.5, 0, width - 1)
    y = np.clip((np.arange(h) + 0.5) * height / h - 0.5, 0, height - 1)
    x0, y0 = x.astype(int), y.astype(int)
    x1, y1 = np.minimum(x0 + 1, width - 1), np.minimum(y0 + 1, height - 1)
    dx, dy = (x - x0)[None, :, None], (y - y0)[:, None, None]
    src = image.astype(float)
    top = src[y0[:, None], x0] * (1 - dx) + src[y0[:, None], x1] * dx
    bottom = src[y1[:, None], x0] * (1 - dx) + src[y1[:, None], x1] * dx
    resized = np.rint(top * (1 - dy) + bottom * dy).clip(0, 255)
    canvas = np.zeros((640, 640, 3))
    ox, oy = (640 - w) // 2, (640 - h) // 2
    canvas[oy:oy + h, ox:ox + w] = resized
    return canvas.transpose(2, 0, 1)[None] / 255


class LetterboxTest(unittest.TestCase):
    def test_reference(self):
        rng = np.random.default_rng(123)
        for h, w in [(480, 640), (377, 613)]:
            with self.subTest(height=h, width=w):
                image = rng.integers(0, 256, (h, w, 3), dtype=np.uint8)
                result = inference.letterbox(image)
                self.assertEqual(result.shape, (1, 3, 640, 640))
                self.assertEqual(result.dtype, np.float32)
                np.testing.assert_allclose(result, reference(image), atol=1.01 / 255)

    def test_real_model_and_decoder(self):
        if not MODEL.is_file() or not (BRIDGE / 'src/yolo26_c.cpp').is_file():
            self.skipTest('real model or decoder source unavailable')
        if not shutil.which('c++'):
            self.skipTest('C++ compiler unavailable')
        try:
            import onnxruntime as ort
        except ImportError:
            self.skipTest('onnxruntime unavailable')
        with tempfile.TemporaryDirectory() as temp:
            lib = Path(temp) / 'decoder.dylib'
            subprocess.run(['c++', '-std=c++17', '-shared', '-fPIC',
                            '-I' + str(BRIDGE / 'include'),
                            str(BRIDGE / 'src/yolo26_decoder.cpp'),
                            str(BRIDGE / 'src/yolo26_c.cpp'), '-o', str(lib)], check=True)
            session = ort.InferenceSession(str(MODEL), providers=['CPUExecutionProvider'])
            self.assertEqual((session.get_inputs()[0].name, session.get_inputs()[0].shape),
                             ('images', [1, 3, 640, 640]))
            self.assertEqual((session.get_outputs()[0].name, session.get_outputs()[0].shape),
                             ('output0', [1, 300, 18]))
            tensor = session.run(['output0'], {'images': np.zeros((1, 3, 640, 640),
                                                                  np.float32)})[0]
            decoded = inference.decode_output(ctypes.CDLL(str(lib)), tensor, 0.0)
            self.assertGreater(len(decoded), 0)
            row = tensor[0, 0]
            np.testing.assert_allclose(decoded[0, :4],
                                       [(row[0] + row[2]) / 2, (row[1] + row[3]) / 2,
                                        row[2] - row[0], row[3] - row[1]], atol=1e-4)
            np.testing.assert_allclose(decoded[0, 4:], row[4:], atol=1e-5)
            self.assertEqual(len(inference.decode_output(ctypes.CDLL(str(lib)), tensor, 1.1)), 0)
            with self.assertRaises(ValueError):
                inference.decode_output(ctypes.CDLL(str(lib)), tensor[:, :, :6], 0.25)


if __name__ == '__main__':
    unittest.main()
