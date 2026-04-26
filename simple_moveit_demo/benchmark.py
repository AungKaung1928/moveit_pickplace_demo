#!/usr/bin/env python3
"""
YOLOv8n ONNX latency benchmark — simulates what TensorRT profiling shows on Jetson.

Usage:
    python3 benchmark.py [path/to/yolov8n.onnx]
    # or after colcon build:
    ros2 run simple_moveit_demo benchmark

Run export_model.py first to generate the ONNX file.
On Jetson with TensorRT, expect 5–10× lower latency than CPU numbers here.
"""

import sys
import time
import numpy as np

N_WARMUP  = 10
N_BENCH   = 100
INPUT_H   = 480
INPUT_W   = 640


def main():
    try:
        import onnxruntime as ort
    except ImportError:
        print('onnxruntime not installed — run: pip install onnxruntime')
        sys.exit(1)

    onnx_path = sys.argv[1] if len(sys.argv) > 1 else 'yolov8n.onnx'

    import os
    if not os.path.exists(onnx_path):
        print(f'ONNX model not found at {onnx_path}. Run export_model.py first.')
        sys.exit(1)

    sess = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
    input_name  = sess.get_inputs()[0].name
    input_shape = (1, 3, INPUT_H, INPUT_W)
    dummy = np.random.rand(*input_shape).astype(np.float32)

    print(f'Benchmarking {onnx_path}  input {INPUT_W}×{INPUT_H}')
    print(f'Warming up ({N_WARMUP} iterations)...')
    for _ in range(N_WARMUP):
        sess.run(None, {input_name: dummy})

    print(f'Benchmarking ({N_BENCH} iterations)...')
    times_ms = []
    for _ in range(N_BENCH):
        t0 = time.perf_counter()
        sess.run(None, {input_name: dummy})
        times_ms.append((time.perf_counter() - t0) * 1000.0)

    t = np.array(times_ms)
    print()
    print(f'  mean : {t.mean():.1f} ms')
    print(f'  std  : {t.std():.1f} ms')
    print(f'  min  : {t.min():.1f} ms')
    print(f'  max  : {t.max():.1f} ms')
    print(f'  FPS  : {1000.0 / t.mean():.1f}')
    print()
    print('  Note: CPU-only (onnxruntime). Jetson Orin with TensorRT FP16')
    print('  typically achieves 5–10× lower latency than these numbers.')


if __name__ == '__main__':
    main()
