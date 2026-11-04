"""机器人“已知”代价地图：已知障碍（静态 + 已发现封路）按机器人尺寸膨胀，叠加风险代价。

cell 代价 = 1 + risk_weight × risk（0..1）；不可通行为 INF。
"""

import math

INF = math.inf


class CostMap:
    def __init__(self, world, inflate_cells=1, risk_weight=8.0):
        self.world = world
        self.w, self.h = world.w, world.h
        self.inflate = inflate_cells
        self.risk_weight = risk_weight
        self.known_closures = set()
        self.occupied = set()      # 未膨胀的已知障碍，用于对角切角判断
        self.cost = {}
        self.rebuild()

    def rebuild(self):
        """重算全图，返回代价发生变化的 cell 集合（供 D* Lite 增量更新）。"""
        known = self.world.static_blocked | self.known_closures
        self.occupied = known
        inflated = set()
        r = self.inflate
        for (x, y) in known:
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    inflated.add((x + dx, y + dy))
        new = {}
        for x in range(self.w):
            for y in range(self.h):
                c = (x, y)
                if c in inflated:
                    new[c] = INF
                else:
                    new[c] = 1.0 + self.risk_weight * self.world.risk(c)
        changed = {c for c in new if self.cost.get(c) != new[c]}
        self.cost = new
        return changed

    def __call__(self, c):
        return self.cost.get(c, INF)

    def is_lethal(self, c):
        return self(c) == INF
