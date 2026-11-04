"""A*：每次从头规划（基线 / 对照组）。"""

import heapq
import itertools

from .grid import INF, neighbors, octile


class AStarPlanner:
    name = "A*"

    def __init__(self, costmap):
        self.costmap = costmap
        self.goal = None
        self.expansions = 0        # 最近一次规划扩展节点数
        self.total_expansions = 0

    def set_goal(self, goal):
        self.goal = goal

    def notify_changed(self, cells):
        pass  # 无增量状态

    def plan(self, start):
        goal = self.goal
        if goal is None or self.costmap(goal) == INF:
            return None
        g = {start: 0.0}
        parent = {}
        tie = itertools.count()
        openq = [(octile(start, goal), next(tie), start)]
        closed = set()
        self.expansions = 0
        while openq:
            _, _, u = heapq.heappop(openq)
            if u in closed:
                continue
            closed.add(u)
            self.expansions += 1
            if u == goal:
                break
            for v, c in neighbors(self.costmap, u):
                if c == INF or v in closed:
                    continue
                ng = g[u] + c
                if ng < g.get(v, INF):
                    g[v] = ng
                    parent[v] = u
                    heapq.heappush(openq, (ng + octile(v, goal), next(tie), v))
        self.total_expansions += self.expansions
        if goal not in g:
            return None
        path = [goal]
        while path[-1] != start:
            path.append(parent[path[-1]])
        return path[::-1]
