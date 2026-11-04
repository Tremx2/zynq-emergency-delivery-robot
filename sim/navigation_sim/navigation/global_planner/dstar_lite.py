"""D* Lite（Koenig & Likhachev 2002，优化版）：从目标反向搜索，地图变化时增量修复。

边代价依赖目标端 cell 与对角切角的两个正交 cell，因此 cell c 变化时，
受影响的边的起点都在 c 及其 8 邻域内——对这些顶点重新计算 rhs 即可。
"""

import heapq

from .grid import INF, neighbors, octile

EPS = 1e-6  # 浮点累加误差：键值“相等”的节点也必须展开，否则会残留过期 g 值


class DStarLitePlanner:
    name = "D* Lite"

    def __init__(self, costmap):
        self.costmap = costmap
        self.goal = None
        self.expansions = 0
        self.total_expansions = 0
        self.fallbacks = 0         # 路径提取失败后整图重算的次数（正常应为 0）
        self._reset()

    def _reset(self):
        self.g, self.rhs = {}, {}
        self.U = []
        self.in_queue = {}
        self.km = 0.0
        self.s_last = None
        self.s_start = None
        self.pending = set()

    def set_goal(self, goal):
        self._reset()
        self.goal = goal
        if goal is not None:
            self.rhs[goal] = 0.0
            self._push(goal)

    # ---- 基本操作 ----
    def _key(self, s):
        m = min(self.g.get(s, INF), self.rhs.get(s, INF))
        return (m + octile(self.s_start, s) + self.km, m) if self.s_start else (m, m)

    def _push(self, s):
        k = self._key(s)
        self.in_queue[s] = k
        heapq.heappush(self.U, (k, s))

    def _update_vertex(self, u):
        if u != self.goal:
            best = INF
            for v, c in neighbors(self.costmap, u):
                if c != INF:
                    best = min(best, c + self.g.get(v, INF))
            self.rhs[u] = best
        self.in_queue.pop(u, None)  # 惰性删除：堆中旧条目在弹出时跳过
        if self.g.get(u, INF) != self.rhs.get(u, INF):
            self._push(u)

    def _top(self):
        while self.U:
            k, s = self.U[0]
            if self.in_queue.get(s) == k:
                return k, s
            heapq.heappop(self.U)
        return (INF, INF), None

    def _compute(self):
        start = self.s_start
        while True:
            k_old, u = self._top()
            if u is None:
                break
            ks = self._key(start)
            if not (k_old[0] < ks[0] + EPS or self.rhs.get(start, INF) != self.g.get(start, INF)):
                break
            self.expansions += 1
            k_new = self._key(u)
            if k_old < k_new:
                self._push(u)
                continue
            heapq.heappop(self.U)
            del self.in_queue[u]
            gu, ru = self.g.get(u, INF), self.rhs.get(u, INF)
            if gu > ru:
                self.g[u] = ru
                for p, _ in neighbors(self.costmap, u):
                    self._update_vertex(p)
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for p, _ in neighbors(self.costmap, u):
                    self._update_vertex(p)

    # ---- 对外接口 ----
    def notify_changed(self, cells):
        """登记代价变化的 cell，在下一次 plan(start) 时用当前起点修复。"""
        self.pending |= set(cells)

    def _apply_pending(self):
        if not self.pending:
            return
        self.km += octile(self.s_last, self.s_start)
        self.s_last = self.s_start
        touched = set()
        for (x, y) in self.pending:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    c = (x + dx, y + dy)
                    if 0 <= c[0] < self.costmap.w and 0 <= c[1] < self.costmap.h:
                        touched.add(c)
        self.pending = set()
        for c in touched:
            self._update_vertex(c)

    def plan(self, start):
        if self.goal is None or self.costmap(self.goal) == INF:
            return None
        self.s_start = start
        if self.s_last is None:
            self.s_last = start
        self.expansions = 0
        self._apply_pending()
        self._compute()
        path = self._extract(start)
        if path is False:
            # 防御：增量状态异常时退化为从头搜索，并计数以便测试发现
            self.fallbacks += 1
            self.set_goal(self.goal)
            self.s_start = self.s_last = start
            self._compute()
            path = self._extract(start) or None
        self.total_expansions += self.expansions
        return path

    def _extract(self, start):
        """沿 argmin(c + g) 贪心回溯。不可达返回 None，提取异常返回 False。"""
        if self.g.get(start, INF) == INF:
            return None
        path = [start]
        seen = {start}
        while path[-1] != self.goal:
            u = path[-1]
            best, best_v = INF, None
            for v, c in neighbors(self.costmap, u):
                val = c + self.g.get(v, INF)
                if val < best:
                    best, best_v = val, v
            if best_v is None or best == INF or best_v in seen:
                return False
            path.append(best_v)
            seen.add(best_v)
        return path
