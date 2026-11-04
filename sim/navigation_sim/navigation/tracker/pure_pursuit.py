"""Pure Pursuit 路径跟踪：路径点 → (v, ω)。航向误差过大时原地转向。"""

import math
from dataclasses import dataclass


@dataclass
class TrackerParams:
    lookahead_m: float = 0.45
    v_max: float = 0.35          # m/s
    w_max: float = 1.8           # rad/s
    goal_tol_m: float = 0.12
    rotate_in_place_rad: float = 1.0
    slow_down_m: float = 0.5     # 距终点多远开始减速


class PurePursuit:
    def __init__(self, params=None):
        self.p = params or TrackerParams()
        self.path = []
        self.idx = 0

    def set_path(self, pts):
        self.path = list(pts)
        self.idx = 0

    def compute(self, x, y, th):
        """返回 (v, w, arrived)。"""
        if not self.path:
            return 0.0, 0.0, False
        gx, gy = self.path[-1]
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal < self.p.goal_tol_m:
            return 0.0, 0.0, True

        # 前进最近点索引（只向前，避免回头）
        best_i, best_d = self.idx, math.inf
        for i in range(self.idx, min(len(self.path), self.idx + 20)):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < best_d:
                best_i, best_d = i, d
        self.idx = best_i

        # 预瞄点：沿路径第一个距离 ≥ lookahead 的点
        tx, ty = gx, gy
        for i in range(self.idx, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= self.p.lookahead_m:
                tx, ty = px, py
                break

        alpha = (math.atan2(ty - y, tx - x) - th + math.pi) % (2 * math.pi) - math.pi
        if abs(alpha) > self.p.rotate_in_place_rad:
            return 0.0, math.copysign(self.p.w_max * 0.6, alpha), False

        ld = max(math.hypot(tx - x, ty - y), 1e-3)
        curvature = 2 * math.sin(alpha) / ld
        v = self.p.v_max * min(1.0, dist_goal / self.p.slow_down_m + 0.2)
        v *= max(0.3, math.cos(alpha))
        w = v * curvature
        if abs(w) > self.p.w_max:
            v *= self.p.w_max / abs(w)
            w = math.copysign(self.p.w_max, w)
        return v, w, False
