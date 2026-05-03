#!/usr/bin/env python3
"""
YOLOv8n-based object detector — replaces the synthetic-data MLP.

Trained on COCO (real images); expect low recall on synthetic top-down shapes.
See README → Domain Gap Analysis for the three gap factors.
"""

import numpy as np

try:
    from ultralytics import YOLO as _YOLO
    _ULTRALYTICS_OK = True
except ImportError:
    _ULTRALYTICS_OK = False


class YOLODetector:
    """
    Wraps YOLOv8n for image-level object detection.

    Input:  BGR numpy array  (H × W × 3)
    Output: list of {'class': str, 'confidence': float, 'bbox': [x1, y1, x2, y2]}
    """

    def __init__(self, weights: str = 'yolov8n.pt'):
        if _ULTRALYTICS_OK:
            self._model = _YOLO(weights)
            self._ready = True
        else:
            self._model = None
            self._ready = False

    @property
    def ready(self) -> bool:
        return self._ready

    def detect(self, image: np.ndarray) -> list:
        if not self._ready:
            return []
        results = self._model(image, verbose=False)
        out = []
        for r in results:
            for box in r.boxes:
                out.append({
                    'class': self._model.names[int(box.cls)],
                    'confidence': float(box.conf),
                    'bbox': box.xyxy[0].tolist(),   # [x1, y1, x2, y2]
                })
        return out


def geometric_classify(circularity: float, aspect_ratio: float,
                        solidity: float) -> tuple:
    """Heuristic shape label when YOLO has no detections (domain gap fallback)."""
    if circularity > 0.85 and solidity > 0.92:
        return 'cylinder', 0.72
    if 0.65 < aspect_ratio < 1.45 and solidity > 0.88:
        return 'cube', 0.72
    return 'irregular', 0.72
