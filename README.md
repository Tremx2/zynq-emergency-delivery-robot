# 基于 Zynq 软硬协同的应急物资自主投送机器人

基于 PYNQ-Z2（XC7Z020）的四轮差速移动机器人。

- **PL 端**：自定义 IP `robot_motor_axi`，包含四路正交解码、10 ms 测速、6 级流水线定点 PID、20 kHz PWM，以及 **500 ms 硬件看门狗**。
- **PS 端**（Linux/PYNQ）：负责底盘运动学、定距和定角运动、传感器（IMU / ToF / INA219 / OLED）、视频推流，以及风险感知导航（D* Lite + Pure Pursuit，目前只在仿真中运行）。

## 关键指标

| 指标 | 数值 |
|---|---|
| PL 时钟 / 时序 | 125 MHz；WNS +0.850 ns，WHS +0.035 ns，无违例 |
| 资源占用 | LUT 3358（6.31%），FF 3145（2.96%），DSP 0，BRAM 0，IOB 19 |
| 闭环 / PWM | 测速周期 10 ms；PWM 20 kHz（周期 6250），限幅 ±6249 |
| 看门狗 | 62 500 000 个时钟周期 = 500 ms；魔数布防、魔数心跳、停车互锁、故障锁存、PWM 引脚门控 |
| 里程标定 | 17.707 counts/mm；原地转 90°：左转 5611 counts，右转 4809.43 counts |
| IMU 静态标定 | 六面法，标定后 \|a\| 误差 RMS 从 5.03 mg 降到 1.07 mg |

## 目录结构

```
zynq-emergency-delivery-robot/
├── README.md            项目简介 + 复现步骤（本文件）
├── src/                 设计源码
│   ├── ip_repo/         自定义 IP robot_motor_axi v1.3（RTL：解码 / 测速 / PID / PWM / 看门狗 / AXI）
│   ├── bd/              Block Design（robot_system.bd）
│   ├── constraints/     引脚约束（robot_system.xdc）
│   └── ps/              PS 侧 Python：底盘驱动、传感器驱动、标定、推流、开机安全
├── sim/                 仿真与验证：navigation_sim（导航仿真 + HIL 框架 + 11 项回归测试）
├── build/               可复现构建脚本（Vivado 2022.2）+ 当前比特流的综合 / 实现报告
├── board/               上板工程：overlay（.bit/.hwh）、测试脚本与 notebook、systemd 服务、PC 端部署工具、实测输出
├── data/                测试数据与参考结果：传感器采集、IMU 六面标定、导航仿真指标
└── report/              设计报告、寄存器映射
```

每个目录里都有 README.md，说明其中的内容和用法。

除 README.md 外，所有目录名和文件名都使用小写英文、数字、下划线或连字符。有两个例外，都是 Vivado IP 打包工具生成的文件：
- `src/ip_repo/robot_motor_axi_1_0/hdl/robot_motor_axi_v1_0_S00_AXI.v`；
- 驱动目录下的 `Makefile`。

`component.xml` 按原文件名引用它们，Makefile 也是驱动构建工具要求的固定文件名，改名会破坏 IP 打包，所以保留原名。

## 复现步骤

### 0. 环境
- **硬件**：PYNQ-Z2、4 个 JGB37-520 编码电机、4 块 BTS7960、TXS0108E、ICM-42688-P、VL53L1X、INA219、SH1106 OLED、USB 摄像头。引脚分配见 `src/constraints/robot_system.xdc`，接线与系统结构见 `report/design_report.docx`。
- **FPGA**：Vivado 2022.2，器件 xc7z020clg400-1。
- **板端**：PYNQ 3.1 镜像，使用板载 Python venv `/usr/local/share/pynq-venv`。板子 IP `192.168.2.99`，账号密码 `xilinx/xilinx`。
- **PC**：Python 3.10 及以上；部署工具需要先 `pip install paramiko`；导航仿真只用标准库（含 tkinter）。

### 1. 生成比特流
这一步可以跳过：`board/overlay/` 里已经放了与板上一致的 .bit/.hwh。

```bat
build\build.bat
```
这是 Windows 的方式；如果 Vivado 不在默认路径，先设置环境变量 `VIVADO_BAT` 指向 vivado.bat。

```bash
bash build/build.sh      # Linux
```

产物输出在 `build/output/`，可以和 `build/reports/` 里的参考结果对比（参考时序 WNS +0.850 ns）。

### 2. 部署到板子
```bash
python board/host_tools/deploy.py            # 上传 overlay、src/ps、board/tests，逐文件做 md5 校验
python board/host_tools/deploy.py --service  # 首次部署时使用：额外安装开机关断电机的服务
```

### 3. 上板测试
在板上 `/home/xilinx/jupyter_notebooks/` 目录下运行，需要 root 权限并设置 `XILINX_XRT=/usr`：

```bash
sudo env XILINX_XRT=/usr /usr/local/share/pynq-venv/bin/python3 test_1_forward.py   # test_1~5：前进 / 后退 / 左弧 / 右弧 / 原地旋转
sudo env XILINX_XRT=/usr /usr/local/share/pynq-venv/bin/python3 test_sensors.py all # I2C 扫描 + IMU + ToF
```

在 Jupyter（http://192.168.2.99:9090）中运行两个 notebook：
- `chassis_motion_demo.ipynb`：底盘六个动作的演示，输出速度跟踪、轨迹、误差图表，并保存数据。
- `watchdog_timeout_test.ipynb`：看门狗超时断电测试，覆盖三种失效：停止心跳、kill -9、写错误魔数。用软件时间戳测量 500 ms 断电。

IMU 六面法标定的流程：
1. 运行 `imu_six_face_calib.py new` 新建会话；
2. 依次把车体摆成 flat、left、right、front、rear、flipped 六个姿态，每摆好一个就执行一次 `collect <face>`，每面采集约 15 s，期间保持静止；
3. 运行 `solve` 解算，结果写入 `imu_calib.json`，原始数据保存在 `imu_calib_data/` 下。

### 4. 导航仿真（PC）
```bash
cd sim/navigation_sim
python -m navigation.gui                    # 可视化界面
python -m navigation.run_headless --goal A  # 无界面运行
python -m unittest discover -s tests -v     # 11 项回归测试
```

## 安全须知
- **上电顺序**：先给 PYNQ 上电，再接 11.1 V 电机电源。PYNQ 建议用 DC 7~15 V 供电；用 USB 拓展坞供电时电流不足，板子会反复重启，还可能损坏 SD 卡。
- **看门狗默认状态**：复位后看门狗处于未布防状态，PWM 被硬件门控。`Chassis` 构造时会自动布防，并在后台喂狗；另外还有 1 s 指令租约，超时会自动停车。
- **首次测试**：第一次运行电机相关测试时，请把车轮架空。

## 已知限制
- `build/` 下的 tcl 脚本是从原 Vivado 工程整理成"从源码重建"流程的。整理时所用的电脑没有安装 Vivado，**这套流程还没有在全新环境中完整跑通**。参考产物以 `board/overlay/` 和 `build/reports/` 为准。
- 导航仿真里的 PL 行为模型沿用了早期的寄存器草案（200 ms 看门狗、Q16.16 PID），和实际比特流（见 `report/register_map.md`）不一致。仿真数据不代表实车结果。
- 陀螺标度的动态标定，以及 IMU 与编码器的融合定位，都还没有完成。
