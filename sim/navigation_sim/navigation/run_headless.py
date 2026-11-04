"""无界面批量运行，输出 CSV 遥测 + summary JSON 到 metrics/navigation/。

例：
  python -m navigation.run_headless --goal A                 # 含 6 s 定时封路
  python -m navigation.run_headless --goal A --no-events     # 无封路
  python -m navigation.run_headless --goal C --planner astar
  python -m navigation.run_headless --goal A --wdt-demo 10   # 第 10 s 停发 heartbeat 1 s
"""

import argparse
import json
import os

from .mission import ARRIVED, make_sim_system

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(goal="A", planner="dstar", risk_weight=8.0, events=True, wdt_demo=None, max_time=120.0, seed=0):
    world, pl, chassis, m = make_sim_system(planner=planner, risk_weight=risk_weight, seed=seed,
                                            enable_events=events)
    m.set_goal(goal)
    wdt = {}
    while m.state != ARRIVED and m.t < max_time:
        if wdt_demo is not None:
            if wdt_demo <= m.t < wdt_demo + 1.0:
                m.heartbeat_enabled = False
                wdt.setdefault("hb_stop_t", m.t)
            elif m.t >= wdt_demo + 1.0 and "recover_t" not in wdt:
                m.heartbeat_enabled = True
                chassis.clear_fault()
                chassis.enable()
                wdt["recover_t"] = m.t
            if "hb_stop_t" in wdt and "estop_t" not in wdt and m.tel.state != 1:
                wdt["estop_t"] = m.t
            if "estop_t" in wdt and "stopped_t" not in wdt and all(abs(s) < 50 for s in m.tel.speed):
                wdt["stopped_t"] = m.t
        m.step()
    s = m.summary()
    if wdt:
        s["wdt_demo"] = {k: round(v, 3) for k, v in wdt.items()}
        if "estop_t" in wdt:
            s["wdt_demo"]["hb_loss_to_estop_s"] = round(wdt["estop_t"] - wdt["hb_stop_t"], 3)
    return m, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", default="A")
    ap.add_argument("--planner", choices=["dstar", "astar"], default="dstar")
    ap.add_argument("--risk-weight", type=float, default=8.0)
    ap.add_argument("--no-events", action="store_true")
    ap.add_argument("--wdt-demo", type=float, default=None, help="在该时刻停发 heartbeat 1 s")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=os.path.join(ROOT, "metrics", "navigation"))
    a = ap.parse_args()

    m, s = run(a.goal, a.planner, a.risk_weight, not a.no_events, a.wdt_demo, seed=a.seed)
    os.makedirs(a.out, exist_ok=True)
    tag = f"{a.goal}_{a.planner}_rw{a.risk_weight:g}" + ("" if not a.no_events else "_noevt") \
          + ("" if a.wdt_demo is None else "_wdt")
    m.write_csv(os.path.join(a.out, f"run_{tag}.csv"))
    with open(os.path.join(a.out, f"run_{tag}_summary.json"), "w", encoding="utf-8") as f:
        json.dump(s, f, ensure_ascii=False, indent=2)
    print(json.dumps(s, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
