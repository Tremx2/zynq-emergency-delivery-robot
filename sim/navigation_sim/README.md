# PYNQ-Z2 应急物资投送机器人 —— 2D 仿真 / HIL 框架

ToF 暂未建模（`FAULT_TOF` / `SAFE_WARNING` 已在寄存器中预留）。

纯 Python 标准库（界面用 tkinter），无需安装任何依赖。

## 运行

```bash
cd sim/navigation_sim
python -m navigation.gui                              # 可视化界面
python -m navigation.run_headless --goal A            # 无界面，含 6 s 定时封路
python -m navigation.run_headless --goal A --wdt-demo 10   # 第 10 s 停发 heartbeat
python -m unittest discover -s tests -v               # 回归测试
```

界面操作：点“投送点 A/B/C”或左键设目标；切换左键模式可加封路（真实环境，进入 1.5 m 感知半径才被发现）或风险区（立即已知）；右键删除。
取消“PS 发送 Heartbeat”即 Demo D；“软件急停”/“清除故障并使能”用于安全域演示。

## 分层

```
navigation/                 ← 数字环境 / 上层（只调用 Chassis API）
  simulator/  world.py scenarios.py     栅格世界、封路真值、风险区、定时事件
  risk/       costmap.py                已知障碍膨胀 + 风险代价（Risk Map, L4）
  global_planner/ astar.py dstar_lite.py  A*（L1）/ D* Lite 增量重规划（L3）
  tracker/    pure_pursuit.py           路径 → (v, ω)
  mission.py                            Mission Manager / HIL Bridge，日志与指标
  gui.py  run_headless.py
pynq_sw/drivers/            ← PS 侧
  regmap.py   寄存器映射（RTL / 仿真 / 驱动共用，见 docs/register_map.md）
  chassis.py  Chassis API + 差速运动学 + 编码器里程计 + PynqMMIO 后端
  sim_pl.py   PL 行为级模型：4×(编码器→测速→Q16.16 PID→PWM)，Safety FSM，Watchdog，电机物理
```

> 目录用 `pynq_sw/` 而不是文档里的 `pynq/`：板上若把仓库根加入 `sys.path`，同名目录会遮蔽官方 `pynq` 包。

HIL 切换点只有一个：`navigation/mission.py` 的 `make_pynq_system(base_addr)` 用 `PynqMMIO` 替换 `SimPL`，导航代码不变。**该路径尚无比特流，未经验证。**

## 已验证（仿真内，非真实硬件数据）

| 项目 | 结果 |
|---|---|
| 四轮不同目标速度跟踪 | 1.5 s 后 0.5 s 平均误差 < 3%（含 ±7% 电机个体差异） |
| Watchdog | 最后一次 heartbeat 后恰好 20 个 control_tick（200 ms）进入 E_STOP，故障锁存，需 heartbeat 新鲜才能清除 |
| 导航仍在发命令时 PL 拒绝执行 | 测试覆盖 |
| A/B/C 三点 × A*/D* Lite，含封路 | 全部到达，0 碰撞 |
| D* Lite vs A* | 随机封路/风险区变化下路径代价一致；去 A 的任务累计扩展 1766 vs 5052 |

模型参数（轮径、CPR、空载转速、时间常数、PID 增益、等效轮距）都是占位值，接真实底盘后需要实测替换；所有性能结论以真实测试为准。

## 指标输出

`metrics/navigation/run_*.csv`：每 50 ms 一行，含位姿、v/ω 命令、safety_state、fault_bits、四轮 target/speed/pid_out（与文档 4.4 的信号集一致）。
`*_summary.json`：到达与否/用时、路径长度、重规划次数、安全停车次数、碰撞、风险暴露、规划耗时。
