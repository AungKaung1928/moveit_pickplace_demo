#!/usr/bin/env python3

import numpy as np
import threading

try:
    import torch
    import torch.nn as nn
    _TORCH_OK = True
except ImportError:
    _TORCH_OK = False

CLASSES = ['cube', 'cylinder', 'irregular']


class _Net(nn.Module if _TORCH_OK else object):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(5, 32), nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(32, 16), nn.ReLU(),
            nn.Linear(16, 3),
        )

    def forward(self, x):
        return self.net(x)


class ShapeClassifier:
    """
    Lightweight PyTorch shape classifier trained at startup on synthetic data.

    Input features (5):
        area_norm   – contour area / image area          [0, 1]
        aspect_ratio – bounding-rect w/h (raw value)     [0.1 – 5]
        circularity  – 4π·area / perimeter²              [0, 1]
        solidity     – area / convex-hull area            [0, 1]
        extent       – area / bounding-rect area          [0, 1]

    Output: (class_name: str, confidence: float)
    """

    def __init__(self):
        self._model = _Net() if _TORCH_OK else None
        self._trained = False
        self._lock = threading.Lock()

        if _TORCH_OK:
            t = threading.Thread(target=self._train, daemon=True)
            t.start()

    # ------------------------------------------------------------------
    def _synthetic_dataset(self, n: int = 2500):
        rng = np.random.default_rng(42)
        X, y = [], []
        for _ in range(n):
            cls = int(rng.integers(0, 3))
            if cls == 0:           # cube – nearly square, medium circularity
                area       = float(rng.uniform(0.04, 0.35))
                aspect     = float(rng.normal(1.0, 0.18))
                circularity= float(rng.normal(0.785, 0.055))
                solidity   = float(rng.normal(0.95, 0.03))
                extent     = float(rng.normal(0.88, 0.05))
            elif cls == 1:         # cylinder – very round in top-view
                area       = float(rng.uniform(0.04, 0.35))
                aspect     = float(rng.normal(1.0, 0.10))
                circularity= float(rng.normal(0.94, 0.03))
                solidity   = float(rng.normal(0.97, 0.02))
                extent     = float(rng.normal(0.785, 0.04))
            else:                  # irregular – wide variance
                area       = float(rng.uniform(0.01, 0.25))
                aspect     = float(rng.uniform(0.4, 3.0))
                circularity= float(rng.uniform(0.15, 0.62))
                solidity   = float(rng.normal(0.72, 0.12))
                extent     = float(rng.uniform(0.30, 0.70))

            feat = [
                np.clip(area, 0.0, 1.0),
                np.clip(aspect / 4.0, 0.0, 1.0),   # normalise to [0,1]
                np.clip(circularity, 0.0, 1.0),
                np.clip(solidity, 0.0, 1.0),
                np.clip(extent, 0.0, 1.0),
            ]
            X.append(feat)
            y.append(cls)

        return (np.array(X, dtype=np.float32),
                np.array(y, dtype=np.int64))

    def _train(self):
        X, y = self._synthetic_dataset()
        Xt = torch.from_numpy(X)
        yt = torch.from_numpy(y)

        opt = torch.optim.Adam(self._model.parameters(), lr=1e-3)
        loss_fn = nn.CrossEntropyLoss()

        self._model.train()
        for _ in range(120):
            perm = torch.randperm(len(Xt))
            for i in range(0, len(Xt), 64):
                idx = perm[i:i + 64]
                opt.zero_grad()
                loss_fn(self._model(Xt[idx]), yt[idx]).backward()
                opt.step()

        self._model.eval()
        with self._lock:
            self._trained = True

    # ------------------------------------------------------------------
    def classify(self, area_norm: float, aspect_ratio: float,
                 circularity: float, solidity: float, extent: float):
        """Return (class_name, confidence)."""
        if _TORCH_OK:
            with self._lock:
                trained = self._trained

            if trained:
                feat = torch.tensor(
                    [[area_norm,
                      aspect_ratio / 4.0,
                      circularity,
                      solidity,
                      extent]],
                    dtype=torch.float32)
                with torch.no_grad():
                    probs = torch.softmax(self._model(feat), dim=1)[0]
                idx = int(probs.argmax())
                return CLASSES[idx], float(probs[idx])

        # Heuristic fallback (also used when torch unavailable)
        if circularity > 0.85 and solidity > 0.92:
            return 'cylinder', 0.72
        if 0.65 < aspect_ratio < 1.45 and solidity > 0.88:
            return 'cube', 0.72
        return 'irregular', 0.72
