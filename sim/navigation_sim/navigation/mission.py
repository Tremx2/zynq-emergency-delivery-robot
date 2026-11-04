"""Mission Manager / HIL Bridge：感知更新 → 规划 → 跟踪 → Chassis.set_velocity → 读回遥测。

导航层只通过 Chassis API 与底盘交互；仿真与真实板卡的区别只在 make_* 工厂里。
"""

import csv
import math
import time

from pynq_sw.drivers import regmap as R
from pynq_sw.drivers.chassis import Chassis, ChassisParams, Odometry
from pynq_sw.drivers.sim_pl import SimPL

from .global_planner.astar import AStarPlanner
from .global_planner.dstar_lite import DStarLitePlanner
from .risk.costmap import CostMap
from .simulator.scenarios import SCENARIOS
from .simulator.world import RiskZone
from .tracker.pure_pursuit import PurePursuit

PLANNERS = {"astar": AStarPlanner, "dstar": DStarLitePlanner}

IDLE, MOVING, BLOCKED, ARRIVED, SAFETY_STOP = "IDLE", "MOVING", "BLOCKED", "ARRIVED", "SAFETY_STOP"


class Mission:
    NAV_DT = 0.05   # 导航/心跳周期 20 Hz

    def __init__(self, world, chassis, advance, planner="dstar", risk_weight=8.0, sense_radius_m=1.5,
                 enable_events=True):
        self.world = world
        self.chassis = chassis
        self.advance = advance
        self.sense_radius = sense_radius_m
        self.enable_events = enable_events
        self.costmap = CostMap(world, inflate_cells=1, risk_weight=risk_weight)
        self.planner_kind = planner
        self.planner = PLANNERS[planner](self.costmap)
        self.tracker = PurePursuit()
        self.odom = Odometry(chassis.p)
        self.heartbeat_enabled = True
        self.map_version = 0       # 代价地图每次变化 +1，供界面判断是否重绘
        self.reset()

    # ---------------- 生命周期 ----------------
    def reset(self):
        tel = self.chassis.read_telemetry()
        x, y = self.world.cell_center(self.world.start)
        self.odom.reset(x, y, self.world.start_th, tel.enc)
        self.t = 0.0
        self.state = IDLE
        self.goal = None
        self.goal_name = None
        self.path = None
        self.need_replan = False
        self.trail = [(x, y)]
        self.cmd = (0.0, 0.0)
        self.tel = tel
        self.rows = []
        self.event_idx = 0
        self.stats = dict(replans=0, safety_stop_events=0, collisions=0, risk_exposure=0.0,
                          arrive_time=None, blocked_time=0.0, plan_ms=[])
        self._prev_cell_blocked = False
        self._prev_safe = False
        self.chassis.stop()
        self.chassis.clear_fault()
        self.chassis.enable()

    def set_goal(self, goal):
        if isinstance(goal, str):
            self.goal_name, goal = goal, self.world.goals[goal]
        else:
            self.goal_name = None
        if self.costmap.is_lethal(goal):
            return False
        self.goal = goal
        self.planner.set_goal(goal)
        self.path = None
        self.need_replan = True
        self.state = MOVING
        self.stats["arrive_time"] = None
        return True

    def set_planner(self, kind):
        self.planner_kind = kind
        self.planner = PLANNERS[kind](self.costmap)
        if self.goal is not None:
            self.planner.set_goal(self.goal)
            self.need_replan = True

    def set_risk_weight(self, wgt):
        self.costmap.risk_weight = wgt
        self._map_changed()

    # ---------------- 环境编辑（真值） ----------------
    def add_closure(self, cells):
        self.world.add_closure(cells)

    def remove_closure(self, cells):
        self.world.remove_closure(cells)

    def add_risk_zone(self, zone):
        self.world.risk_zones.append(zone)
        self._map_changed()   # 风险区由指挥中心广播，立即已知

    def remove_risk_zone_near(self, x, y):
        for z in self.world.risk_zones:
            if math.hypot(z.cx - x, z.cy - y) <= z.radius:
                self.world.risk_zones.remove(z)
                self._map_changed()
                return True
        return False

    def _map_changed(self):
        changed = self.costmap.rebuild()
        if changed:
            self.map_version += 1
            self.planner.notify_changed(changed)
            if self.goal is not None and self.state != ARRIVED:
                self.need_replan = True

    # ---------------- 感知 ----------------
    def robot_cell(self):
        return self.world.to_cell(self.odom.x, self.odom.y)

    def _sense(self):
        r2 = self.sense_radius ** 2

        def near(c):
            cx, cy = self.world.cell_center(c)
            return (cx - self.odom.x) ** 2 + (cy - self.odom.y) ** 2 <= r2

        known = self.costmap.known_closures
        found = {c for c in self.world.closures - known if near(c)}
        cleared = {c for c in known - self.world.closures if near(c)}
        if found or cleared:
            known |= found
            known -= cleared
            self._map_changed()

    def _process_events(self):
        while self.enable_events and self.event_idx < len(self.world.events) \
                and self.world.events[self.event_idx][0] <= self.t:
            _, kind, payload = self.world.events[self.event_idx]
            self.event_idx += 1
            if kind == "close":
                self.add_closure(payload)
            elif kind == "open":
                self.remove_closure(payload)
            elif kind == "risk":
                self.add_risk_zone(payload if isinstance(payload, RiskZone) else RiskZone(*payload))

    # ---------------- 主循环一步 ----------------
    def step(self):
        dt = self.NAV_DT
        self._process_events()
        tel = self.chassis.read_telemetry()
        self.tel = tel
        px, py = self.odom.x, self.odom.y
        self.odom.update(tel.enc)
        ds = math.hypot(self.odom.x - px, self.odom.y - py)
        if ds > 1e-4:
            self.trail.append((self.odom.x, self.odom.y))
        if self.heartbeat_enabled:
            self.chassis.heartbeat()

        self._sense()
        cell = self.robot_cell()

        v = w = 0.0
        if self.goal is not None and self.state != ARRIVED:
            if self.need_replan or self.path is None:
                t0 = time.perf_counter()
                path = self.planner.plan(cell)
                self.stats["plan_ms"].append((time.perf_counter() - t0) * 1000)
                if self.path is not None and path is not None:
                    self.stats["replans"] += 1
                self.need_replan = False
                self.path = path
                if path:
                    self.tracker.set_path([self.world.cell_center(c) for c in path])
            if self.path is None:
                self.state = BLOCKED            # L2：无路可走则原地停车，等待环境变化
                self.need_replan = True
                self.stats["blocked_time"] += dt
            else:
                v, w, arrived = self.tracker.compute(self.odom.x, self.odom.y, self.odom.th)
                if arrived:
                    self.state = ARRIVED
                    self.stats["arrive_time"] = self.t
                    v = w = 0.0
                else:
                    self.state = MOVING

        # 安全域独立于导航：即使导航仍在发速度命令，PL 处于非 RUN 态时也不会执行
        safe = tel.state == R.SAFE_RUN
        if not safe and self._prev_safe:   # 只统计 RUN → 非 RUN 的跳变
            self.stats["safety_stop_events"] += 1
        self._prev_safe = safe
        self.cmd = (v, w)
        self.chassis.set_velocity(v, w)

        blocked = self.world.is_blocked(cell)
        if blocked and not self._prev_cell_blocked:
            self.stats["collisions"] += 1
        self._prev_cell_blocked = blocked
        self.stats["risk_exposure"] += self.world.risk(cell) * ds

        self._log(tel)
        self.advance(dt)
        self.t += dt

    def display_state(self):
        if self.tel.state != R.SAFE_RUN:
            return f"{SAFETY_STOP}({self.tel.state_name})"
        return self.state

    # ---------------- 日志与指标 ----------------
    def _log(self, tel):
        row = dict(t=round(self.t, 3), tick=tel.tick, x=round(self.odom.x, 4), y=round(self.odom.y, 4),
                   th=round(self.odom.th, 4), v_cmd=round(self.cmd[0], 4), w_cmd=round(self.cmd[1], 4),
                   mission=self.state, safety_state=tel.state, fault_bits=tel.faults)
        for i, n in enumerate(R.WHEEL_NAMES):
            row[f"target_{n}"] = tel.target[i]
            row[f"speed_{n}"] = tel.speed[i]
            row[f"pid_out_{n}"] = tel.pid_out[i]
        self.rows.append(row)

    def write_csv(self, path):
        if not self.rows:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            wr.writeheader()
            wr.writerows(self.rows)

    def summary(self):
        s = self.stats
        pm = sorted(s["plan_ms"])
        return dict(
            scenario=self.world.name, planner=self.planner.name, goal=self.goal_name or self.goal,
            risk_weight=self.costmap.risk_weight, arrived=s["arrive_time"] is not None,
            arrive_time_s=None if s["arrive_time"] is None else round(s["arrive_time"], 2),
            path_length_m=round(self.odom.distance, 3), replans=s["replans"],
            safety_stop_events=s["safety_stop_events"], collisions=s["collisions"],
            risk_exposure=round(s["risk_exposure"], 3), blocked_time_s=round(s["blocked_time"], 2),
            planner_total_expansions=self.planner.total_expansions,
            plan_ms_p50=round(pm[len(pm) // 2], 3) if pm else None,
            plan_ms_max=round(pm[-1], 3) if pm else None,
        )


def make_sim_system(scenario="disaster_area", planner="dstar", risk_weight=8.0, seed=0, **kw):
    """纯仿真：世界 + SimPL + Chassis + Mission。"""
    world = SCENARIOS[scenario]()
    pl = SimPL(seed=seed)
    chassis = Chassis(pl, ChassisParams())
    mission = Mission(world, chassis, pl.advance, planner=planner, risk_weight=risk_weight, **kw)
    return world, pl, chassis, mission


def make_pynq_system(base_addr, scenario="disaster_area", planner="dstar", risk_weight=8.0, **kw):
    """HIL：在 PYNQ 上运行，导航仍在仿真地图里，底盘是真实 PL（尚未有比特流，未验证）。"""
    from pynq_sw.drivers.chassis import PynqMMIO
    world = SCENARIOS[scenario]()
    chassis = Chassis(PynqMMIO(base_addr), ChassisParams())
    mission = Mission(world, chassis, time.sleep, planner=planner, risk_weight=risk_weight, **kw)
    return world, None, chassis, mission
