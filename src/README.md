# src/ — 设计源码

| 路径 | 内容 |
|---|---|
| `ip_repo/robot_motor_axi_1_0/` | 自定义 AXI4-Lite IP（v1.3），即 `component.xml` 及其源码 |
| `ip_repo/.../hdl/robot_motor_axi_v1_0_S00_AXI.v` | AXI 寄存器文件、硬件看门狗、PWM 引脚门控、`tof_xshut` 输出 |
| `ip_repo/.../src/robot_motor_ctrl.v` | 四通道顶层 |
| `ip_repo/.../src/motor_channel.v` | 单通道：两级同步 → 解码 → 测速 → PID → PWM |
| `ip_repo/.../src/quadrature_decoder.v` | 四倍频正交解码（查表，非法跳变丢弃） |
| `ip_repo/.../src/speed_meter.v` | M 法测速，每 10 ms 一次，输出 `speed_valid` |
| `ip_repo/.../src/pid_controller.v` | 6 级流水线 Q8 定点 PID（Kp=1，Ki=1/16，Kd=0）；积分饱和 ±100000，输出 ±6249 |
| `ip_repo/.../src/motor_pwm.v` | 20 kHz PWM（周期 6250），由符号选择 RPWM 或 LPWM |
| `ip_repo/.../src/motor_test.v` | 早期单电机测试顶层（保留作参考，不参与当前 BD） |
| `bd/robot_system.bd` | Block Design：PS7 + AXI Interconnect + AXI IIC + robot_motor_axi |
| `constraints/robot_system.xdc` | 8 路 PWM、8 路编码器、I2C（T10/T11）、ToF XSHUT（W14）的引脚约束 |
| `ps/` | PS 侧 Python 程序，见下表 |

## ps/

| 文件 | 作用 |
|---|---|
| `robot_chassis.py` | ★ 底盘驱动 `Chassis`。负责看门狗布防与喂狗、1 s 指令租约、差速运动学、定距 / 定角运动、上电自检 |
| `test_sensors.py` | 直接用 MMIO 操作 AXI IIC 的驱动，加上 ICM42688 / VL53L1X 驱动；命令行支持 `scan/imu/tof/all/live` |
| `imu_static_calib.py` | IMU 水平静置标定，得到陀螺零偏和安装角 |
| `imu_six_face_calib.py` | IMU 六面法标定（会话方式，保留全部原始数据），流程 `new / collect / status / solve` |
| `collect_all.py` | 全量采集：I2C 扫描、IMU、ToF、电量，输出 JSON |
| `battery_display.py` | INA219 测电量，在 OLED 上显示 |
| `camera_stream.py` | USB 摄像头 MJPEG 推流（:8080） |
| `power_on_self_test.py` | 上电安全自检 |
| `motor_disable_boot.py` | 开机自启：加载 overlay 并主动拉低 PWM，配套 `board/systemd/` 中的服务 |

寄存器映射见 `report/register_map.md`。
