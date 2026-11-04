# AXI-Lite 寄存器映射 v0.1.0（草案）

唯一来源：`pynq_sw/drivers/regmap.py`。修改时 RTL、`sim_pl.py`、本文件同步，并升 `REG_VERSION`（`Chassis` 构造时校验主版本号）。

control_tick = 100 Hz（10 ms）。所有寄存器 32 bit，有符号量为二进制补码。轮序：0 FL, 1 FR, 2 RL, 3 RR。

| 偏移 | 名称 | R/W | 说明 |
|---|---|---|---|
| 0x00 | VERSION | R | 0x00MMmmpp |
| 0x04 | CTRL | R/W | bit0 motor_enable；bit1 clear_fault（写 1 脉冲，仅当 heartbeat 未超时才生效）；bit2 sw_estop |
| 0x08 | HEARTBEAT | W | 任意写入喂狗 |
| 0x0C | SAFETY_STATE | R | 0 IDLE（复位默认，PWM=0）/ 1 RUN / 2 WARNING（预留 ToF）/ 3 E_STOP |
| 0x10 | FAULT_BITS | R | bit0 WDT；bit1 TOF（预留）；bit2 SW_ESTOP。锁存直到 clear_fault |
| 0x14 | TICK_COUNT | R | control_tick 计数，作时间戳 |
| 0x18 | WDT_TIMEOUT | R/W | 单位 tick，默认 20（200 ms）；仅 motor_enable=1 时计时 |
| 0x1C | BUILD_ID | R | 比特流构建号 |
| 0x20–0x2C | TARGET[4] | R/W | 目标轮速，int32，counts/s |
| 0x30–0x3C | SPEED[4] | R | 实测轮速，int32，counts/s（4 tick 滑窗差分 × 25） |
| 0x40–0x4C | ENC[4] | R | 编码器累计计数，int32 |
| 0x50–0x5C | PID_OUT[4] | R | 有符号 PWM 计数，±PWM_MAX |
| 0x60 | KP | R/W | Q16.16 |
| 0x64 | KI | R/W | Q16.16，每 tick 累加；条件积分抗饱和，积分限幅 ±PWM_MAX |
| 0x68 | KD | R/W | Q16.16 |
| 0x6C | PWM_MAX | R/W | 默认 1000 |

Safety FSM（每个 control_tick 求值）：有任一 FAULT → E_STOP；否则 motor_enable=0 → IDLE；否则 RUN。只有 RUN 时 Motor Gate 打开，非 RUN 时 PID 积分清零、PWM=0。
