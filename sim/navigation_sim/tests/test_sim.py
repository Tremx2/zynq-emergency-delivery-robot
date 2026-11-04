"""仿真回归测试：python -m unittest discover -s tests -v（在仓库根目录运行）"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from navigation.global_planner.astar import AStarPlanner
from navigation.global_planner.dstar_lite import DStarLitePlanner
from navigation.global_planner.grid import INF, path_cost
from navigation.mission import ARRIVED, make_sim_system
from navigation.risk.costmap import CostMap
from navigation.run_headless import run
from navigation.simulator.scenarios import disaster_area
from navigation.simulator.world import RiskZone, World
from pynq_sw.drivers import regmap as R
from pynq_sw.drivers.chassis import Chassis
from pynq_sw.drivers.sim_pl import SimPL


def run_for(pl, chassis, seconds, heartbeat=True):
    for _ in range(int(seconds * 100)):
        if heartbeat:
            chassis.heartbeat()
        pl.advance(0.01)


class TestPLModel(unittest.TestCase):
    def setUp(self):
        self.pl = SimPL(seed=1)
        self.ch = Chassis(self.pl)

    def test_reset_is_safe(self):
        tel = self.ch.read_telemetry()
        self.assertEqual(tel.state, R.SAFE_IDLE)
        self.assertEqual(tel.pid_out, [0, 0, 0, 0])
        self.ch.set_wheel_targets([3000] * 4)       # 未使能时下发目标也不应输出
        run_for(self.pl, self.ch, 0.5)
        self.assertEqual(self.ch.read_telemetry().pid_out, [0, 0, 0, 0])

    def test_four_wheels_track_different_targets(self):
        targets = [1500, -2500, 4000, -1000]        # Demo A：四轮不同目标
        self.ch.enable()
        self.ch.set_wheel_targets(targets)
        run_for(self.pl, self.ch, 1.5)
        speeds = []
        for _ in range(50):                        # 0.5 s 平均，消除量化抖动
            run_for(self.pl, self.ch, 0.01)
            speeds.append(self.ch.read_telemetry().speed)
        for i, t in enumerate(targets):
            avg = sum(s[i] for s in speeds) / len(speeds)
            self.assertLess(abs(avg - t), 0.03 * abs(t) + 30, f"wheel {i}: {avg} vs {t}")

    def test_watchdog_trips_and_latches(self):
        self.ch.configure(wdt_timeout_ms=200)
        self.ch.enable()
        self.ch.set_wheel_targets([3000] * 4)
        run_for(self.pl, self.ch, 1.0)
        self.ch.heartbeat()
        tick0 = self.ch.read_telemetry().tick
        while self.ch.read_telemetry().state == R.SAFE_RUN:
            self.pl.advance(0.01)                  # 不再喂狗，导航仍保持旧速度命令
        tel = self.ch.read_telemetry()
        self.assertEqual(tel.state, R.SAFE_ESTOP)
        self.assertTrue(tel.faults & R.FAULT_WDT)
        self.assertEqual(tel.tick - tick0, 20)     # 200 ms = 20 个 control_tick
        self.assertEqual(tel.pid_out, [0, 0, 0, 0])
        run_for(self.pl, self.ch, 0.5)             # 恢复心跳也不会自动解除（故障锁存）
        self.assertEqual(self.ch.read_telemetry().state, R.SAFE_ESTOP)
        self.ch.clear_fault()
        run_for(self.pl, self.ch, 0.05)
        self.assertEqual(self.ch.read_telemetry().state, R.SAFE_RUN)

    def test_clear_fault_rejected_without_heartbeat(self):
        self.ch.enable()
        run_for(self.pl, self.ch, 0.5, heartbeat=False)
        self.pl.write(R.REG_CTRL, R.CTRL_MOTOR_EN | R.CTRL_CLEAR_FAULT)   # 不喂狗直接清
        self.pl.advance(0.01)
        self.assertEqual(self.ch.read_telemetry().state, R.SAFE_ESTOP)

    def test_sw_estop(self):
        self.ch.enable()
        self.ch.set_wheel_targets([3000] * 4)
        run_for(self.pl, self.ch, 0.5)
        self.ch.estop()
        run_for(self.pl, self.ch, 0.02)
        tel = self.ch.read_telemetry()
        self.assertEqual(tel.state, R.SAFE_ESTOP)
        self.assertTrue(tel.faults & R.FAULT_SW_ESTOP)

    def test_set_velocity_kinematics(self):
        self.ch.set_velocity(0.3, 0.0)
        t = self.ch.read_telemetry().target
        self.assertTrue(t[0] == t[1] == t[2] == t[3] > 0)
        self.ch.set_velocity(0.0, 1.0)              # 逆时针原地转：左轮负、右轮正
        t = self.ch.read_telemetry().target
        self.assertTrue(t[R.WHEEL_FL] < 0 < t[R.WHEEL_FR] and t[R.WHEEL_RL] < 0 < t[R.WHEEL_RR])
        self.ch.set_velocity(5.0, 5.0)              # 超限按比例缩放
        t = self.ch.read_telemetry().target
        self.assertLessEqual(max(abs(v) for v in t), self.ch.mps_to_cps(self.ch.p.max_wheel_speed_mps) + 1)


class TestPlanners(unittest.TestCase):
    def test_dstar_matches_astar_under_random_changes(self):
        world = disaster_area()
        cm = CostMap(world)
        rng = random.Random(7)
        free = [c for c in cm.cost if not cm.is_lethal(c)]
        for trial in range(5):
            start, goal = rng.sample(free, 2)
            d = DStarLitePlanner(cm)
            d.set_goal(goal)
            a = AStarPlanner(cm)
            a.set_goal(goal)
            for _ in range(8):
                pd, pa = d.plan(start), a.plan(start)
                self.assertEqual(pa is None, pd is None)
                if pa:
                    self.assertAlmostEqual(path_cost(cm, pd), path_cost(cm, pa), places=6)
                    start = pd[min(len(pd) - 1, rng.randint(1, 15))]    # 机器人前进几步
                # 随机封路 / 解封 / 风险区
                c = rng.choice(free)
                cells = [(c[0] + i, c[1] + j) for i in range(3) for j in range(3)]
                if rng.random() < 0.7:
                    cm.known_closures |= set(cells) - {start, goal}
                else:
                    world.risk_zones.append(RiskZone(c[0] * world.res, c[1] * world.res, 0.6))
                d.notify_changed(cm.rebuild())
            cm.known_closures.clear()
            world.risk_zones[2:] = []
            cm.rebuild()
            self.assertEqual(d.fallbacks, 0)

    def test_risk_weight_avoids_risk(self):
        # 空旷 6 m × 2 m 场地，直线中点放一个风险区
        world = World(48, 16, 0.125, set(), (2, 8), 0.0, {"G": (45, 8)},
                      [RiskZone(3.0, 1.0, 0.7)])
        start, goal = world.start, world.goals["G"]
        exposure = {}
        for wgt in (0.0, 8.0):
            cm = CostMap(world, risk_weight=wgt)
            a = AStarPlanner(cm)
            a.set_goal(goal)
            exposure[wgt] = sum(world.risk(c) for c in a.plan(start))
        self.assertLess(exposure[8.0], exposure[0.0])


class TestMission(unittest.TestCase):
    def test_all_goals_reached_with_closure(self):
        for goal in "ABC":
            for planner in ("dstar", "astar"):
                m, s = run(goal, planner)
                self.assertTrue(s["arrived"], f"{goal}/{planner}: {s}")
                self.assertEqual(s["collisions"], 0)

    def test_closure_forces_replan(self):
        _, with_evt = run("A", "dstar", events=True)
        _, no_evt = run("A", "dstar", events=False)
        self.assertGreater(with_evt["path_length_m"], no_evt["path_length_m"] + 0.5)

    def test_watchdog_stops_robot_while_nav_still_commands(self):
        _, _, ch, m = make_sim_system(enable_events=False)
        m.set_goal("B")
        while m.t < 5:
            m.step()
        m.heartbeat_enabled = False
        x0 = None
        for _ in range(40):                         # 2 s
            m.step()
            if m.tel.state == R.SAFE_ESTOP and x0 is None:
                x0 = (m.odom.x, m.odom.y)
        self.assertIsNotNone(x0)
        self.assertGreater(abs(m.cmd[0]) + abs(m.cmd[1]), 0)     # 导航仍在发命令
        self.assertTrue(all(abs(s) < 50 for s in m.tel.speed))    # 但轮子已停
        self.assertLess(math.hypot(m.odom.x - x0[0], m.odom.y - x0[1]), 0.15)


if __name__ == "__main__":
    unittest.main()
