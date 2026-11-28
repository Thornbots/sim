#!/usr/bin/env python3
# Copyright 2026 Thornbots
# Licensed under the Apache License, Version 2.0.
"""Score panel_sweep's RGB stream with the bench encoder and robot decoder."""
import argparse
import ctypes
import subprocess
import time

import numpy as np
import onnxruntime as ort

from onnx_armor_inference import decode_output, letterbox


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('renderer')
    parser.add_argument('model')
    parser.add_argument('decoder')
    parser.add_argument('--hfov', default='1.204277')
    parser.add_argument('--threshold', type=float, default=0.1)
    args = parser.parse_args()
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(args.model, sess_options=options,
                                   providers=['CPUExecutionProvider'])
    decoder = ctypes.CDLL(args.decoder)
    start = time.monotonic()
    frames = subprocess.check_output([args.renderer, args.hfov])
    render_seconds = time.monotonic() - start
    size = 640 * 480 * 3
    if len(frames) != 84 * size:
        raise ValueError('renderer emitted an unexpected number of bytes')
    start = time.monotonic()
    for face in ('sticker', 'dotmatrix'):
        print(face, 'yaw: -60 -40 -20 0 20 40 60')
        for distance in range(1, 7):
            row = []
            for yaw in range(7):
                index = ((0 if face == 'sticker' else 1) * 6 + distance - 1) * 7 + yaw
                image = np.frombuffer(frames, np.uint8, size, index * size).reshape(480, 640, 3)
                output = session.run(['output0'], {'images': letterbox(image)})[0]
                row.append(int(len(decode_output(decoder, output, args.threshold)) > 0))
            print(f'{distance}m:', *row)
    print(f'render {render_seconds / 84:.3f} s/frame; '
          f'encode+inference+decode {(time.monotonic() - start) / 84:.3f} s/frame')


if __name__ == '__main__':
    main()
