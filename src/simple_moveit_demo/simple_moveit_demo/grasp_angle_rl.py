#!/usr/bin/env python3
import os
import torch
import torch.nn as nn
import torch.optim as optim

_CKPT = os.path.expanduser('~/.ros/grasp_angle_policy_v2.pt')
_X_NORM = (0.25, 0.60)
_Y_NORM = (-0.40, 0.40)


class _Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(2, 16), nn.ReLU(),
            nn.Linear(16, 2),
        )

    def forward(self, x: torch.Tensor):
        out = self.layers(x)
        return out[0] * 90.0, out[1].clamp(-3.0, 0.0)


class GraspAnglePolicy:
    """REINFORCE policy: (x, y) → grasp_angle_deg. Checkpoint persists to ~/.ros/."""

    def __init__(self, lr: float = 3e-3):
        self._net = _Net()
        self._opt = optim.Adam(self._net.parameters(), lr=lr)
        self._log_prob: torch.Tensor | None = None
        self._load()

    def select_angle(self, obj_x: float, obj_y: float) -> float:
        feat = self._encode(obj_x, obj_y)
        mean, log_std = self._net(feat)
        dist = torch.distributions.Normal(mean, log_std.exp())
        angle = dist.sample()
        self._log_prob = dist.log_prob(angle)
        return float(angle.clamp(-90.0, 90.0).detach())

    def update(self, reward: float) -> None:
        if self._log_prob is None:
            return
        loss = -self._log_prob * reward
        self._opt.zero_grad()
        loss.backward()
        self._opt.step()
        self._log_prob = None
        self._save()

    def _encode(self, x: float, y: float) -> torch.Tensor:
        nx = (x - _X_NORM[0]) / (_X_NORM[1] - _X_NORM[0])
        ny = (y - _Y_NORM[0]) / (_Y_NORM[1] - _Y_NORM[0])
        return torch.tensor([nx, ny], dtype=torch.float32)

    def _save(self) -> None:
        os.makedirs(os.path.dirname(_CKPT), exist_ok=True)
        torch.save(self._net.state_dict(), _CKPT)

    def _load(self) -> None:
        if os.path.exists(_CKPT):
            try:
                self._net.load_state_dict(torch.load(_CKPT, weights_only=True))
            except Exception:
                pass
