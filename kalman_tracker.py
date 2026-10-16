"""
Day 5: Kalman Filter & Probabilistic Uncertainty Representation
Day 7: Multi-Object Tracking, Trajectory Forecasting & Time-To-Collision (TTC)
"""

import math
from typing import Dict, List, Optional, Tuple
import numpy as np


class KalmanTrack2D:
    """
    Linear Constant-Velocity Kalman Filter in 2D BEV Space:
      State vector: x = [X, Y, Vx, Vy]^T
      State transition matrix F(dt):
        [1, 0, dt,  0]
        [0, 1,  0, dt]
        [0, 0,  1,  0]
        [0, 0,  0,  1]
      Measurement matrix H:
        [1, 0, 0, 0]
        [0, 1, 0, 0]
    """

    def __init__(self, obj_id: int, label: str, init_x: float, init_y: float):
        self.obj_id = obj_id
        self.label = label
        self.age = 0
        self.missed_frames = 0

        # State vector [X, Y, Vx, Vy]^T
        self.x = np.array([[init_x], [init_y], [-2.5], [0.0]], dtype=np.float64)

        # State covariance matrix P (4x4)
        self.P = np.diag([0.25, 0.25, 2.0, 2.0]).astype(np.float64)

        # Measurement matrix H (2x4)
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ], dtype=np.float64)

        # Base Measurement Noise Covariance R (2x2) in meters^2
        self.R_base = np.diag([0.12, 0.08]).astype(np.float64)

        # History of estimated positions for trail rendering
        self.history: List[Tuple[float, float]] = [(init_x, init_y)]

    def predict(self, dt: float):
        """Kalman Filter Prediction Step: x_{k|k-1} = F x_{k-1}, P_{k|k-1} = F P F^T + Q."""
        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Continuous white-noise acceleration process noise Q
        q_pos = 0.15 * dt
        q_vel = 1.2 * dt
        Q = np.diag([q_pos, q_pos, q_vel, q_vel]).astype(np.float64)

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.age += 1
        self.missed_frames += 1

        self.history.append((float(self.x[0, 0]), float(self.x[1, 0])))
        if len(self.history) > 35:
            self.history.pop(0)

    def update(self, meas_x: float, meas_y: float):
        """Kalman Filter Measurement Update Step with distance-adaptive measurement noise R."""
        z = np.array([[meas_x], [meas_y]], dtype=np.float64)

        # Farther objects in monocular BEV have higher longitudinal uncertainty
        dist_scale = max(1.0, (meas_x / 10.0) ** 2)
        R = self.R_base * dist_scale

        # Innovation y = z - H x
        y = z - (self.H @ self.x)
        # Innovation covariance S = H P H^T + R
        S = self.H @ self.P @ self.H.T + R
        # Optimal Kalman Gain K = P H^T S^{-1}
        K = self.P @ self.H.T @ np.linalg.inv(S)

        I = np.eye(4, dtype=np.float64)
        self.x = self.x + (K @ y)
        self.P = (I - K @ self.H) @ self.P
        self.missed_frames = 0
        self.history[-1] = (float(self.x[0, 0]), float(self.x[1, 0]))

    @property
    def pos(self) -> Tuple[float, float]:
        return (float(self.x[0, 0]), float(self.x[1, 0]))

    @property
    def vel(self) -> Tuple[float, float]:
        return (float(self.x[2, 0]), float(self.x[3, 0]))

    def get_uncertainty_ellipse(self, chi2_val: float = 5.991) -> Tuple[float, float, float]:
        """
        Computes 95% confidence Uncertainty Ellipse from 2x2 position covariance P_xy:
        Returns (semi_major_m, semi_minor_m, angle_deg).
        """
        P_xy = self.P[0:2, 0:2]
        eigvals, eigvecs = np.linalg.eigh(P_xy)
        eigvals = np.maximum(eigvals, 1e-4)

        # Sort largest eigenvalue first
        order = np.argsort(eigvals)[::-1]
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]

        a = math.sqrt(chi2_val * eigvals[0])
        b = math.sqrt(chi2_val * eigvals[1])
        angle_deg = math.degrees(math.atan2(eigvecs[1, 0], eigvecs[0, 0]))
        return (a, b, angle_deg)

    def forecast_trajectory(self, horizon_sec: float = 2.0, num_steps: int = 8) -> List[Tuple[float, float]]:
        """Forecasts future BEV positions over horizon_sec using estimated constant velocity."""
        px, py = self.pos
        vx, vy = self.vel
        pts = []
        for t in np.linspace(0.2, horizon_sec, num_steps):
            pts.append((px + vx * float(t), py + vy * float(t)))
        return pts

    def compute_ttc(self, ego_half_width: float = 1.65) -> Optional[float]:
        """
        Computes Time-To-Collision (TTC in seconds) with the VinBus front safety zone.
        Checks both current longitudinal closing speed and whether the forecasted lateral position
        intersects the bus collision corridor [-ego_half_width, +ego_half_width].
        """
        px, py = self.pos
        vx, vy = self.vel

        # Object must be in front of bumper (px > 0) and approaching (vx < -0.2 m/s)
        if px <= 0.3 or vx >= -0.25:
            return None

        ttc_x = (px - 0.5) / (-vx)
        if ttc_x < 0.0 or ttc_x > 8.0:
            return None

        # Predicted lateral position Y when object reaches the front bumper X = 0.5m
        pred_y_at_impact = py + vy * ttc_x

        # Include positional uncertainty margin from P_yy
        std_y = math.sqrt(max(self.P[1, 1], 0.01))
        if abs(pred_y_at_impact) <= (ego_half_width + 0.5 * std_y) or abs(py) <= ego_half_width:
            return float(ttc_x)

        return None


class MultiObjectKalmanTracker:
    """Manages multiple KalmanTrack2D instances across frames (including during full occlusion)."""

    def __init__(self, max_missed_frames: int = 45):
        self.tracks: Dict[int, KalmanTrack2D] = {}
        self.max_missed_frames = max_missed_frames

    def step(self, dt: float, measurements: Dict[int, Tuple[str, float, float]]) -> Dict[int, KalmanTrack2D]:
        """
        1. Predicts all existing tracks forward by dt.
        2. Updates tracks that have a valid BEV measurement this frame.
        3. Keeps occluded tracks alive via Kalman prediction up to max_missed_frames (~1.5s)!
        """
        # 1. Predict all existing tracks
        for trk in self.tracks.values():
            trk.predict(dt)

        # 2. Update with current measurements or initialize new track
        for obj_id, (label, mx, my) in measurements.items():
            if obj_id in self.tracks:
                self.tracks[obj_id].update(mx, my)
            else:
                self.tracks[obj_id] = KalmanTrack2D(obj_id, label, mx, my)

        # 3. Prune expired tracks
        stale_ids = [
            tid for tid, trk in self.tracks.items()
            if trk.missed_frames > self.max_missed_frames or trk.pos[0] < 0.2 or trk.pos[0] > 35.0
        ]
        for tid in stale_ids:
            del self.tracks[tid]

        return self.tracks

    def reset(self):
        self.tracks.clear()
