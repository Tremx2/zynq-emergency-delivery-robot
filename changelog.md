\# Changelog



本文件记录 Zynq FPGA Autonomous Emergency Delivery Robot 的重要版本变化。



\---



\## \[Unreleased]

### 2026-11-04 仓库重组与新增测试

- 按竞赛推荐结构重组目录：`src/`、`sim/`、`build/`、`board/`、`data/`、`report/`。原 `pynq/` 拆到 `src/ps/` 和 `board/tests/`，原 `fpga/` 并入 `src/`，原 `docs/` 并入 `report/`。
- 用当前上板版本更新了以下文件：
  - `src/bd/robot_system.bd`；
  - `src/constraints/robot_system.xdc`（含 I2C 和 tof_xshut）；
  - `board/overlay/robot_system_wrapper.bit` 和 `.hwh`（与板上一致）。
- 删除过时的 `robot_system_bd.tcl` 和 `recreate_project_legacy.tcl`（含旧的 D: 盘路径），改用新增的 `build/create_project.tcl` 和 `build/build_bitstream.tcl` 从源码重建。
- 新增 `board/host_tools/deploy.py`，用于一键部署；新增 `pynq_get.py`，用于带 md5 校验的下载。
- 新增两个 notebook：`chassis_motion_demo.ipynb`（底盘六动作数据采集）和 `watchdog_timeout_test.ipynb`（看门狗断电软件时间戳测试）。
- 新增 `src/ps/imu_six_face_calib.py`，做 IMU 六面法标定，保留全部原始数据。标定数据放在 `data/imu_calib/`。
- 新增 `report/register_map.md`（v1.3）。


\### 2026-11-03 软件安全修复

\- `robot_chassis.py`：后台喂狗线程改为「指令租约」模式。运动中超过 `command_timeout`（默认 1.0s）没有新的 `drive()`/`keepalive()` 就自动停车，`keepalive()` 随后抛 `CommandTimeout`。`command_timeout=None` 恢复旧行为。

\- `robot_chassis.py`：`drive()` 在看门狗未布防/已超时时抛 `WatchdogFault`；`move_distance`/`rotate_angle` 循环里检查故障和租约；同一进程重复创建 `Chassis` 时自动关闭旧实例；`shutdown()` 不再重复布防。

\- `test_1~5`：运动指令移入 `try`（Ctrl+C 不会漏停车），循环内调用 `keepalive()`。

\- `battery_display.py`：SSD1306 命令控制字节 0x80 → 0x00；已加载同一 bitstream 时不再重复下载（`--reload` 强制）。

\- `test_sensors.py`：IMU 地址 0x68 → 0x69。

\- `fpga/`：IP HDL、component.xml、XDC 与 `E:pga\ip_repo` / Vivado 工程同步（含 `tof_xshut`）。`robot_system_bd.tcl` 仍是旧版（无 I2C/XSHUT），需在 Vivado 里 `write_bd_tcl` 重新导出。



\### Planned



\- FPGA Safety FSM

\- Command Watchdog

\- Fault latch / status

\- Hardware motor output gate

\- Heartbeat

\- RTL / Build version register

\- FPGA debug observability

\- ILA verification

\- AXI register contract versioning

\- PYNQ Chassis driver API

\- Automated FPGA regression tests



\---



\## \[0.1.0-baseline] - 2026-09-29



\### Baseline



首次建立正式 Git 项目基线。



该版本不是项目最早期状态，而是 Git 仓库建立时已经完成实机验证的 FPGA 四轮闭环控制版本。



\### FPGA



\- Four-channel motor control

\- Four-channel encoder input

\- Quadrature encoder decoding

\- Fixed-period wheel speed measurement

\- Four-channel fixed-point PID control

\- PWM / direction generation

\- AXI4-Lite PS-to-PL control interface

\- PID output saturation

\- Four channels operating concurrently



\### Verified Hardware Behavior



已完成实机验证：



\- 四个电机均可独立运行

\- 四个电机均可正反转

\- 四路编码器均可读取

\- 四路编码器方向与电机方向对应

\- 四路 PID 可同时闭环运行

\- 目标速度 `\[-50, -50, +50, +50]` 可稳定跟踪

\- Vivado implementation timing passed

\- PYNQ `.bit + .hwh` Overlay 已实际加载运行



整车前进方向当前定义为：



```text

M1 = -V

M2 = -V

M3 = +V

M4 = +V

