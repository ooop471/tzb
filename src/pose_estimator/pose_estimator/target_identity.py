"""Keep detection selection on the mission's world-frame target before registration."""
import math
import numpy as np


class LockedTargetIdentity:
    def __init__(self, radius=0.45):
        self.radius = radius
        self.active = False
        self.point = None
        self.frame = 'odom_init'

    def update(self, state):
        self.active = bool(state.get('target_position_locked', False))
        self.frame = state.get('locked_target_frame', 'odom_init')
        try:
            p = np.asarray(state.get('locked_target_world'), dtype=float)
            self.point = p if p.shape == (3,) and np.isfinite(p).all() else None
        except (TypeError, ValueError):
            self.point = None

    def expected(self, buffer, frame):
        if not self.active or self.point is None:
            return None
        if frame == self.frame:
            return self.point.copy()
        try:
            from rclpy.time import Time
            t = buffer.lookup_transform(frame, self.frame, Time()).transform
            q = t.rotation
            v = np.array([q.x, q.y, q.z], dtype=float)
            norm = math.sqrt(float(v @ v) + q.w*q.w)
            if not math.isfinite(norm) or norm < 1e-6:
                return None
            v = v / norm
            w = q.w / norm
            return self.point + 2*w*np.cross(v, self.point) + 2*np.cross(v, np.cross(v, self.point)) + np.array([t.translation.x, t.translation.y, t.translation.z])
        except Exception:
            return None

    def distance(self, candidate, expected):
        if not self.active:
            return 0.0
        if expected is None:
            return None
        p = np.asarray(candidate, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            return None
        d = float(np.linalg.norm(p - expected))
        return d if d <= self.radius else None
