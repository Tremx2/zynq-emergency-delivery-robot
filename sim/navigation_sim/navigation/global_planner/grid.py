"""8 邻接栅格图的公共定义。边 u→v 的代价 = 几何长度 × cost(v)；对角移动不允许切过真实障碍的角
（膨胀层已提供安全距离，若按膨胀层判断切角，贴墙起点会被困住）。"""

import math

INF = math.inf
SQRT2 = math.sqrt(2)
MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


def neighbors(costmap, u):
    """返回 (v, edge_cost)，包含 INF 边。"""
    x, y = u
    for dx, dy, d in MOVES:
        v = (x + dx, y + dy)
        if not (0 <= v[0] < costmap.w and 0 <= v[1] < costmap.h):
            continue
        c = costmap(v)
        if dx and dy and ((x + dx, y) in costmap.occupied or (x, y + dy) in costmap.occupied):
            c = INF
        yield v, d * c


def octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2) * min(dx, dy)


def path_cost(costmap, path):
    total = 0.0
    for u, v in zip(path, path[1:]):
        for n, c in neighbors(costmap, u):
            if n == v:
                total += c
                break
        else:
            return INF
    return total
