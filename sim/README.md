# sim/ — 仿真与验证

## navigation_sim/
这是 2D 导航仿真和硬件在环（HIL）框架，只用 Python 标准库，界面基于 tkinter，主要内容：
- 栅格灾区场景；
- 风险代价地图；
- A* / D* Lite 增量重规划；
- Pure Pursuit 路径跟踪；
- 任务状态机；
- PL 行为级模型 `SimPL`，模拟编码器、测速、PID、PWM、安全状态机和看门狗。

```bash
cd sim/navigation_sim
python -m navigation.gui                                 # 可视化
python -m navigation.run_headless --goal A               # 无界面，含第 6 s 动态封路
python -m navigation.run_headless --goal A --wdt-demo 10 # 第 10 s 停发心跳，演示看门狗
python -m unittest discover -s tests -v                  # 11 项回归测试
```
运行结果写入 `navigation_sim/metrics/`（该目录已加入 .gitignore）。参考结果放在 `data/navigation_sim/`。

> 注意：`SimPL` 用的是早期寄存器草案（200 ms 看门狗、Q16.16 PID）和占位电机参数，与实际比特流不一致。仿真结果只用来验证算法。

## RTL 验证
- 自定义 IP 的 RTL 已经在实际比特流上完成验证，测试方法和数据见 `board/tests/` 与 `data/`。
- `src/ip_repo/robot_motor_axi_1_0/example_designs/bfm_design/` 是 Vivado 自动生成的 AXI BFM 测试平台模板。
- 目前还没有独立的 RTL 单元测试平台，这一项列为后续工作。
