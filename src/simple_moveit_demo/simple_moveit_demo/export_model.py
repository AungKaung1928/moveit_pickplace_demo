#!/usr/bin/env python3
"""
Export YOLOv8n to ONNX with fixed 640×480 input and verify inference.

Usage:
    python3 export_model.py
    # or after colcon build:
    ros2 run simple_moveit_demo export_model
"""

import os
import sys
import numpy as np


def main():
    try:
        from ultralytics import YOLO
    except ImportError:
        print('ultralytics not installed — run: pip install ultralytics')
        sys.exit(1)

    try:
        import onnxruntime as ort
    except ImportError:
        print('onnxruntime not installed — run: pip install onnxruntime')
        sys.exit(1)

    print('Loading YOLOv8n...')
    model = YOLO('yolov8n.pt')

    print('Exporting to ONNX (input 640×480)...')
    # imgsz = [height, width] for ultralytics export
    export_path = model.export(format='onnx', imgsz=[480, 640], simplify=True)

    size_mb = os.path.getsize(export_path) / (1024 * 1024)
    print(f'Exported → {export_path}  ({size_mb:.1f} MB)')

    print('Verifying ONNX model...')
    sess = ort.InferenceSession(export_path,
                                providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0]
    dummy = np.random.rand(1, 3, 480, 640).astype(np.float32)
    out = sess.run(None, {inp.name: dummy})
    print(f'Inference OK — input {inp.shape}  output {out[0].shape}')
    print('Run benchmark.py next to measure latency.')


if __name__ == '__main__':
    main()
