"""2D 仿真 / HIL 可视化界面（tkinter，无第三方依赖）。

运行：python -m navigation.gui
  左键：按当前模式 设目标 / 加封路 / 加风险区；右键：移除封路或风险区。
"""

import json
import math
import os
import time
import tkinter as tk
from tkinter import ttk

from pynq_sw.drivers import regmap as R

from .mission import ARRIVED, BLOCKED, make_sim_system
from .simulator.world import RiskZone

PX = 11                     # 每个 cell 的像素
CHART_H = 170
CHART_SECONDS = 6.0
WHEEL_COLORS = ("#1f77b4", "#d62728", "#2ca02c", "#9467bd")
FONT = ("Microsoft YaHei", 9)
FONT_B = ("Microsoft YaHei", 10, "bold")
CLOSURE_BRUSH = 4           # 封路笔刷边长（cell）
RISK_RADIUS = 0.8           # 新增风险区半径（m）

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _blend(c1, c2, t):
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    return "#%02x%02x%02x" % tuple(round(x + (y - x) * t) for x, y in zip(a, b))


class App:
    def __init__(self, root):
        self.root = root
        root.title("PYNQ-Z2 应急物资投送机器人 · 2D 仿真 / HIL")
        self.running = False
        self.speed = tk.IntVar(value=2)
        self.mode = tk.StringVar(value="goal")
        self.planner_kind = tk.StringVar(value="dstar")
        self.risk_weight = tk.DoubleVar(value=8.0)
        self.hb = tk.BooleanVar(value=True)
        self.events_on = tk.BooleanVar(value=True)
        self._build_ui()
        self.reset()
        self._tick()

    # ---------------- 布局 ----------------
    def _build_ui(self):
        left = tk.Frame(self.root)
        left.pack(side=tk.LEFT, padx=6, pady=6)
        right = tk.Frame(self.root)
        right.pack(side=tk.LEFT, fill=tk.Y, padx=6, pady=6)

        self.canvas = tk.Canvas(left, bg="white", highlightthickness=1, highlightbackground="#999")
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self._on_left)
        self.canvas.bind("<Button-3>", self._on_right)
        tk.Label(left, text="四轮目标(虚线) / 实测(实线) 轮速 counts/s —— PL control_tick 100 Hz",
                 font=FONT).pack(anchor="w")
        self.chart = tk.Canvas(left, height=CHART_H, bg="white", highlightthickness=1,
                               highlightbackground="#999")
        self.chart.pack(fill=tk.X)

        def section(text):
            tk.Label(right, text=text, font=FONT_B, anchor="w").pack(fill=tk.X, pady=(8, 2))

        section("运行")
        row = tk.Frame(right)
        row.pack(fill=tk.X)
        self.btn_run = tk.Button(row, text="▶ 开始", width=8, command=self.toggle_run)
        self.btn_run.pack(side=tk.LEFT)
        tk.Button(row, text="重置", width=6, command=self.reset).pack(side=tk.LEFT, padx=2)
        tk.Button(row, text="保存日志", width=8, command=self.save_log).pack(side=tk.LEFT)
        row = tk.Frame(right)
        row.pack(fill=tk.X)
        tk.Label(row, text="倍速", font=FONT).pack(side=tk.LEFT)
        tk.Scale(row, from_=1, to=8, orient=tk.HORIZONTAL, variable=self.speed, length=150).pack(side=tk.LEFT)
        tk.Checkbutton(right, text="启用场景定时事件（6 s 封路）", variable=self.events_on, font=FONT,
                       command=lambda: setattr(self.mission, "enable_events", self.events_on.get())
                       ).pack(anchor="w")

        section("任务目标")
        row = tk.Frame(right)
        row.pack(fill=tk.X)
        self.goal_row = row

        section("鼠标左键模式（右键删除）")
        for val, txt in (("goal", "设目标点"), ("close", "加封路（真实环境，靠近才发现）"),
                         ("risk", "加风险区（指挥中心广播，立即已知）")):
            tk.Radiobutton(right, text=txt, variable=self.mode, value=val, font=FONT).pack(anchor="w")

        section("规划器")
        row = tk.Frame(right)
        row.pack(fill=tk.X)
        for val, txt in (("dstar", "D* Lite（增量）"), ("astar", "A*（每次重算）")):
            tk.Radiobutton(row, text=txt, variable=self.planner_kind, value=val, font=FONT,
                           command=lambda: self.mission.set_planner(self.planner_kind.get())).pack(side=tk.LEFT)
        row = tk.Frame(right)
        row.pack(fill=tk.X)
        tk.Label(row, text="风险权重", font=FONT).pack(side=tk.LEFT)
        tk.Scale(row, from_=0, to=30, resolution=1, orient=tk.HORIZONTAL, variable=self.risk_weight,
                 length=150, command=lambda _v: self.mission.set_risk_weight(self.risk_weight.get())
                 ).pack(side=tk.LEFT)

        section("安全域（PL 独立于导航）")
        tk.Checkbutton(right, text="PS 发送 Heartbeat（取消 = 模拟上层程序卡死）", variable=self.hb,
                       font=FONT, command=lambda: setattr(self.mission, "heartbeat_enabled", self.hb.get())
                       ).pack(anchor="w")
        row = tk.Frame(right)
        row.pack(fill=tk.X)
        tk.Button(row, text="软件急停", width=10, bg="#f4c7c3",
                  command=lambda: self.chassis.estop()).pack(side=tk.LEFT)
        tk.Button(row, text="清除故障并使能", width=14,
                  command=self._clear_fault).pack(side=tk.LEFT, padx=4)
        self.lbl_safety = tk.Label(right, font=("Consolas", 12, "bold"), anchor="w")
        self.lbl_safety.pack(fill=tk.X, pady=(4, 0))

        section("遥测（经 Chassis API 读寄存器）")
        self.txt = tk.Label(right, font=("Consolas", 9), justify=tk.LEFT, anchor="nw")
        self.txt.pack(fill=tk.X)
        self.lbl_hint = tk.Label(right, font=FONT, fg="#b00", anchor="w", justify=tk.LEFT, wraplength=330)
        self.lbl_hint.pack(fill=tk.X, pady=4)

    # ---------------- 系统构建 ----------------
    def reset(self):
        self.world, self.pl, self.chassis, self.mission = make_sim_system(
            planner=self.planner_kind.get(), risk_weight=self.risk_weight.get(),
            enable_events=self.events_on.get())
        self.mission.heartbeat_enabled = self.hb.get()
        w, h = self.world.w, self.world.h
        self.canvas.config(width=w * PX, height=h * PX)
        self.chart.config(width=w * PX)
        for c in self.goal_row.winfo_children():
            c.destroy()
        for name in sorted(self.world.goals):
            tk.Button(self.goal_row, text=f"投送点 {name}", width=8,
                      command=lambda n=name: self._set_goal(n)).pack(side=tk.LEFT, padx=1)
        self._drawn_map = None
        self._drawn_truth = None
        self.history = []
        self.canvas.delete("all")
        self._draw_static()
        self._redraw()

    def toggle_run(self):
        self.running = not self.running
        self.btn_run.config(text="⏸ 暂停" if self.running else "▶ 开始")

    def _set_goal(self, goal):
        if not self.mission.set_goal(goal):
            self.lbl_hint.config(text="目标点处于障碍/膨胀区内，无法设置")
            return
        if not self.running:
            self.toggle_run()

    def _clear_fault(self):
        self.hb.set(True)
        self.mission.heartbeat_enabled = True
        self.chassis.clear_fault()
        self.chassis.enable()

    def save_log(self):
        out = os.path.join(ROOT, "metrics", "navigation")
        os.makedirs(out, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.mission.write_csv(os.path.join(out, f"gui_{stamp}.csv"))
        with open(os.path.join(out, f"gui_{stamp}_summary.json"), "w", encoding="utf-8") as f:
            json.dump(self.mission.summary(), f, ensure_ascii=False, indent=2)
        self.lbl_hint.config(text=f"已保存到 metrics/navigation/gui_{stamp}*", fg="#060")

    # ---------------- 鼠标 ----------------
    def _event_cell(self, ev):
        return ev.x // PX, self.world.h - 1 - ev.y // PX

    def _event_xy(self, ev):
        return ev.x / PX * self.world.res, (self.world.h * PX - ev.y) / PX * self.world.res

    def _brush(self, c):
        o = CLOSURE_BRUSH // 2
        return [(c[0] + i - o, c[1] + j - o) for i in range(CLOSURE_BRUSH) for j in range(CLOSURE_BRUSH)]

    def _on_left(self, ev):
        c = self._event_cell(ev)
        mode = self.mode.get()
        if mode == "goal":
            self._set_goal(c)
        elif mode == "close":
            self.mission.add_closure(self._brush(c))
        else:
            x, y = self._event_xy(ev)
            self.mission.add_risk_zone(RiskZone(x, y, RISK_RADIUS, 1.0, name="新增风险"))
        self._redraw()

    def _on_right(self, ev):
        c = self._event_cell(ev)
        if self.mode.get() == "risk":
            self.mission.remove_risk_zone_near(*self._event_xy(ev))
        else:
            self.mission.remove_closure(self._brush(c))
        self._redraw()

    # ---------------- 主循环 ----------------
    def _tick(self):
        t0 = time.perf_counter()
        if self.running:
            for _ in range(self.speed.get()):
                self.mission.step()
                tel = self.mission.tel
                self.history.append((self.mission.t, tel.target[:], tel.speed[:]))
            cutoff = self.mission.t - CHART_SECONDS
            while self.history and self.history[0][0] < cutoff:
                self.history.pop(0)
            if self.mission.state == ARRIVED and self.mission.tel.state == R.SAFE_RUN \
                    and all(s == 0 for s in self.mission.tel.speed):
                self.toggle_run()
        self._redraw()
        spent = (time.perf_counter() - t0) * 1000
        self.root.after(max(1, int(self.mission.NAV_DT * 1000 - spent)), self._tick)

    # ---------------- 绘制 ----------------
    def _rect(self, c, **kw):
        x, y = c[0] * PX, (self.world.h - 1 - c[1]) * PX
        return self.canvas.create_rectangle(x, y, x + PX, y + PX, width=0, **kw)

    def _pt(self, x, y):
        return x / self.world.res * PX, (self.world.h - y / self.world.res) * PX

    def _draw_static(self):
        for c in self.world.static_blocked:
            self._rect(c, fill="#555", tags="static")
        for name, g in self.world.goals.items():
            x, y = self._pt(*self.world.cell_center(g))
            self.canvas.create_rectangle(x - 7, y - 7, x + 7, y + 7, outline="#0a0", width=2, tags="static")
            self.canvas.create_text(x, y - 14, text=f"投送点{name}", font=FONT_B, fill="#060", tags="static")
        x, y = self._pt(*self.world.cell_center(self.world.start))
        self.canvas.create_text(x + 26, y, text="仓库", font=FONT_B, fill="#333", tags="static")

    def _draw_costmap(self):
        cv = self.canvas
        cv.delete("cost")
        cm, wd = self.mission.costmap, self.world
        for x in range(wd.w):
            for y in range(wd.h):
                c = (x, y)
                if c in wd.static_blocked:
                    continue
                if c in cm.known_closures:
                    self._rect(c, fill="#222", tags="cost")
                elif cm.is_lethal(c):
                    self._rect(c, fill="#d9d9d9", tags="cost")
                else:
                    r = wd.risk(c)
                    if r > 0:
                        self._rect(c, fill=_blend("#fff4e0", "#ff3b1f", r), tags="cost")
        for z in wd.risk_zones:
            x, y = self._pt(z.cx, z.cy)
            cv.create_text(x, y, text=z.name, font=FONT_B, fill="#900", tags="cost")
        cv.tag_lower("cost")

    def _draw_truth(self):
        """尚未被机器人发现的真实封路：虚线框（上帝视角）。"""
        cv = self.canvas
        cv.delete("truth")
        for c in self.world.closures - self.mission.costmap.known_closures:
            x, y = c[0] * PX, (self.world.h - 1 - c[1]) * PX
            cv.create_rectangle(x + 1, y + 1, x + PX - 1, y + PX - 1, outline="#444", dash=(2, 2), tags="truth")

    def _redraw(self):
        m = self.mission
        if self._drawn_map != m.map_version:
            self._draw_costmap()
            self._drawn_map = m.map_version
        truth_key = (len(self.world.closures), len(m.costmap.known_closures), hash(frozenset(self.world.closures)))
        if self._drawn_truth != truth_key:
            self._draw_truth()
            self._drawn_truth = truth_key

        cv = self.canvas
        cv.delete("dyn")
        # 感知半径
        rx, ry = self._pt(m.odom.x, m.odom.y)
        rr = m.sense_radius / self.world.res * PX
        cv.create_oval(rx - rr, ry - rr, rx + rr, ry + rr, outline="#9cf", dash=(3, 3), tags="dyn")
        # 规划路径
        if m.path and m.state != ARRIVED:
            pts = [p for c in m.path for p in self._pt(*self.world.cell_center(c))]
            if len(pts) >= 4:
                cv.create_line(*pts, fill="#1a73e8", width=2, tags="dyn")
        if m.goal is not None:
            gx, gy = self._pt(*self.world.cell_center(m.goal))
            cv.create_oval(gx - 5, gy - 5, gx + 5, gy + 5, fill="#1a73e8", outline="", tags="dyn")
        # 实际轨迹
        trail = m.trail[::max(1, len(m.trail) // 400)] + [m.trail[-1]]
        if len(trail) >= 2:
            cv.create_line(*[p for xy in trail for p in self._pt(*xy)], fill="#0a8f3c", width=2, tags="dyn")
        # 机器人
        th = m.odom.th
        L = 0.16 / self.world.res * PX
        pts = []
        for a, r in ((0, L), (2.5, L * 0.75), (-2.5, L * 0.75)):
            pts += [rx + r * math.cos(th + a), ry - r * math.sin(th + a)]
        safe = m.tel.state == R.SAFE_RUN
        cv.create_polygon(*pts, fill="#1a73e8" if safe else "#e53935", outline="black", tags="dyn")

        self._draw_chart()
        self._update_text()

    def _draw_chart(self):
        ch = self.chart
        ch.delete("all")
        W = int(ch.cget("width"))
        H = CHART_H
        if not self.history:
            return
        vmax = max(1000, max(abs(v) for _, tg, sp in self.history for v in tg + sp)) * 1.1
        t_end = self.history[-1][0]

        def xy(t, v):
            return W - (t_end - t) / CHART_SECONDS * W, H / 2 - v / vmax * (H / 2 - 6)

        ch.create_line(0, H / 2, W, H / 2, fill="#ccc")
        ch.create_text(4, 8, text=f"+{vmax:.0f}", anchor="w", font=("Consolas", 8), fill="#888")
        ch.create_text(4, H - 8, text=f"-{vmax:.0f}", anchor="w", font=("Consolas", 8), fill="#888")
        for i in range(4):
            tgt = [p for t, tg, _ in self.history for p in xy(t, tg[i])]
            spd = [p for t, _, sp in self.history for p in xy(t, sp[i])]
            if len(spd) >= 4:
                ch.create_line(*tgt, fill=WHEEL_COLORS[i], dash=(4, 3))
                ch.create_line(*spd, fill=WHEEL_COLORS[i], width=2)
            ch.create_text(60 + 50 * i, 8, text=R.WHEEL_NAMES[i], fill=WHEEL_COLORS[i], font=FONT_B)

    def _update_text(self):
        m, tel = self.mission, self.mission.tel
        safe = tel.state == R.SAFE_RUN
        faults = "|".join(R.fault_names(tel.faults)) or "无"
        self.lbl_safety.config(text=f"PL: {tel.state_name}   故障: {faults}",
                               fg="#060" if safe else "#c00")
        lines = [f"仿真时间 {m.t:6.2f} s   control_tick {tel.tick}",
                 f"任务状态 {m.display_state()}   目标 {m.goal_name or m.goal}",
                 f"位姿 x={m.odom.x:5.2f} y={m.odom.y:5.2f} θ={math.degrees(m.odom.th):6.1f}°",
                 f"导航命令 v={m.cmd[0]:+.2f} m/s  ω={m.cmd[1]:+.2f} rad/s",
                 "",
                 "轮   target   speed   pid_out    enc"]
        for i, n in enumerate(R.WHEEL_NAMES):
            lines.append(f"{n}  {tel.target[i]:+7d} {tel.speed[i]:+7d}  {tel.pid_out[i]:+7d} {tel.enc[i]:+8d}")
        s = m.stats
        lines += ["",
                  f"{m.planner.name}: 重规划 {s['replans']} 次  累计扩展 {m.planner.total_expansions}",
                  f"行驶 {m.odom.distance:5.2f} m  风险暴露 {s['risk_exposure']:.3f}",
                  f"安全停车 {s['safety_stop_events']} 次  碰撞 {s['collisions']} 次",
                  f"到达用时 {s['arrive_time']:.1f} s" if s["arrive_time"] is not None else "到达用时 -"]
        self.txt.config(text="\n".join(lines))

        if not safe and abs(m.cmd[0]) + abs(m.cmd[1]) > 1e-3:
            self.lbl_hint.config(text="导航层仍在下发速度命令，但 PL 安全域已关闭 Motor Gate，拒绝执行。",
                                 fg="#b00")
        elif m.state == BLOCKED:
            self.lbl_hint.config(text="当前已知地图中无可达路径：原地停车，等待环境变化。", fg="#b00")
        elif self.lbl_hint.cget("fg") == "#b00":
            self.lbl_hint.config(text="")


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
