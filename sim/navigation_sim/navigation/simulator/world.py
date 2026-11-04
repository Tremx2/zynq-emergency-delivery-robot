"""2D 栅格世界：静态建筑、动态封路（真实环境中存在，机器人靠近才发现）、风险区。"""

import math
from dataclasses import dataclass


@dataclass
class RiskZone:
    cx: float          # m
    cy: float
    radius: float      # m，风险从中心 1.0 线性衰减到边缘 0
    level: float = 1.0
    name: str = "risk"

    def value_at(self, x, y):
        d = math.hypot(x - self.cx, y - self.cy)
        return 0.0 if d >= self.radius else self.level * (1 - d / self.radius)


class World:
    def __init__(self, width, height, res, static_blocked, start, start_th, goals, risk_zones=(),
                 events=(), name="world"):
        self.w, self.h, self.res = width, height, res
        self.static_blocked = set(static_blocked)
        self.closures = set()             # 真实环境中的临时阻挡（真值）
        self.risk_zones = list(risk_zones)
        self.start, self.start_th = start, start_th
        self.goals = dict(goals)          # 名称 -> cell
        self.events = sorted(events, key=lambda e: e[0])  # (t, kind, payload)
        self.name = name

    def in_bounds(self, c):
        return 0 <= c[0] < self.w and 0 <= c[1] < self.h

    def is_blocked(self, c):
        return not self.in_bounds(c) or c in self.static_blocked or c in self.closures

    def cell_center(self, c):
        return ((c[0] + 0.5) * self.res, (c[1] + 0.5) * self.res)

    def to_cell(self, x, y):
        return (int(math.floor(x / self.res)), int(math.floor(y / self.res)))

    def risk(self, c):
        x, y = self.cell_center(c)
        return min(1.0, sum(z.value_at(x, y) for z in self.risk_zones))

    def add_closure(self, cells):
        self.closures |= {c for c in cells if self.in_bounds(c) and c not in self.static_blocked}

    def remove_closure(self, cells):
        self.closures -= set(cells)
