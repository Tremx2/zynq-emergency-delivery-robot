"""内置场景。地图用字符画描述：'#' 建筑/墙体，'.' 可通行，'S' 起点（物资仓库），'A'..'Z' 投送点。
字符画第一行是地图最上方（y 最大）。"""

import math

from .world import RiskZone, World

DISASTER_AREA = """
########################################
#S.....#...........#...................#
#......#...........#...........B.......#
#......#....####...#....######.........#
#...........####........######.........#
#...........####........######.........#
#...........####........######.........#
#......#...............................#
#......#.......................#########
#......######....#######.......#.......#
#................#.....#...........C...#
#................#.....#...............#
#................#.....#...............#
####.....###.....#.....#.......#.......#
#........###.....#.....#########.......#
#........###...........................#
#..............................A.......#
#......#####......######...............#
#......................................#
########################################
"""


def _parse(art, up):
    """字符画 → 栅格。每个字符放大为 up×up 个 cell。"""
    rows = art.strip("\n").splitlines()
    h = len(rows)
    w = max(len(r) for r in rows)
    blocked, start, goals = set(), None, {}
    for j, row in enumerate(rows):
        y = h - 1 - j
        for x in range(w):
            ch = row[x] if x < len(row) else "#"
            if ch == "#":
                blocked |= {(x * up + i, y * up + k) for i in range(up) for k in range(up)}
            elif ch == "S":
                start = (x, y)
            elif ch.isalpha():
                goals[ch] = (x, y)
    return w * up, h * up, blocked, start, goals


def _rect(world, x0, y0, x1, y1):
    """米制矩形 [x0,x1)×[y0,y1) 覆盖的 cell。"""
    r = world.res
    return [(i, j) for i in range(int(x0 / r), int(x1 / r)) for j in range(int(y0 / r), int(y1 / r))]


def disaster_area():
    """10 m × 5 m 灾区（字符 0.25 m，栅格 0.125 m）：两处风险区 + 一次定时封路事件。"""
    up, res = 2, 0.125
    w, h, blocked, start, goals = _parse(DISASTER_AREA, up)

    def pick(c):
        cands = [(c[0] * up + i, c[1] * up + k) for i in range(up) for k in range(up)]
        return max(cands, key=lambda s: min(max(abs(s[0] - b[0]), abs(s[1] - b[1])) for b in blocked))

    zones = [
        RiskZone(cx=5.2, cy=3.3, radius=1.0, level=1.0, name="火情"),
        RiskZone(cx=3.2, cy=1.0, radius=0.9, level=0.8, name="塌方"),
    ]
    world = World(w, h, res, blocked, pick(start), -math.pi / 2,
                  {k: pick(v) for k, v in goals.items()}, zones, name="disaster_area")
    # 第 6 s 时通往 A/C 的中部通道被倒塌物封住（机器人进入感知半径才能发现）
    world.events = [(6.0, "close", _rect(world, 3.0, 2.25, 4.25, 2.75))]
    return world


SCENARIOS = {"disaster_area": disaster_area}
