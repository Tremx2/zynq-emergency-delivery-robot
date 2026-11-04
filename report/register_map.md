# robot_motor_axi 寄存器映射（v1.3，当前比特流）

- 基址：`0x43C00000`（AXI4-Lite，PS GP0）。
- 所有寄存器都是 32 位；有符号量用二进制补码表示。
- 时钟为 125 MHz。

本映射以 `src/ip_repo/robot_motor_axi_1_0/hdl/robot_motor_axi_v1_0_S00_AXI.v` 和 `src/ps/robot_chassis.py` 为准。

| 偏移 | 访问 | 名称 | 说明 |
|---|---|---|---|
| `0x00` | RW | ENABLE | bit0~3 对应 M1~M4 的使能。实际生效的使能是 `ENABLE & wd_motor_ok` |
| `0x04` `0x08` `0x0C` `0x10` | RW | TARGET_SPEED M1~M4 | 目标速度，单位 counts/10 ms，有符号 |
| `0x14` `0x18` `0x1C` `0x20` | RO | ACTUAL_SPEED M1~M4 | 实测速度，单位 counts/10 ms |
| `0x24` `0x28` `0x2C` `0x30` | RO | POSITION M1~M4 | 编码器累计位置（4 倍频），32 位回绕 |
| `0x34` `0x38` `0x3C` `0x40` | RO | PWM_COMMAND M1~M4 | PID 输出，即 PWM 占空计数，范围 ±6249；符号决定走 RPWM 还是 LPWM |
| `0x44` | WO | WD_HEARTBEAT | 写 `0xC0DEC0DE` 表示喂狗。只在已布防时有效，必须 32 位全字节写入 |
| `0x48` | RO | WD_STATE | bit0 = armed，bit1 = fault（超时故障，锁存） |
| `0x4C` | WO | WD_ARM | 写 `0xA5C35A3C` 表示布防并清除故障，详见下方"看门狗"小节 |
| `0x50` | RW | SENSOR_CTRL | bit0 = ToF XSHUT（W14 引脚），复位值为 0（ToF 关断） |

## 电机与车轮
| 代码名 | 车轮 | 前进时目标速度的符号 |
|---|---|---|
| M1 | 右前 | − |
| M2 | 右后 | − |
| M3 | 左后 | + |
| M4 | 左前 | + |

## 看门狗（500 ms）
**状态与计时**
- 复位后状态为 DISARMED：PWM 引脚被强制拉低，PID 处于清零状态。
- 进入 ARMED 后，计数器开始累加。累加到 62 499 999 时进入 FAULT：`armed←0`、`fault←1`，PWM 引脚在同一拍被门控，PID 输出寄存器在下一拍清零。
- 超时判断优先于同一拍到来的心跳。

**布防规则**
- 布防条件：必须处于"停车"状态，即 `ENABLE[3:0]==0` 且四路目标速度都为 0。
- 已经布防时，再写 ARM 会被忽略，计数器不会清零。如果需要从 0 重新计时，要先让它超时，再重新布防。

**门控逻辑**
- `wd_motor_ok = ARESETN & armed & !fault`。
- 它同时门控 PID 使能和 PWM 引脚：`rpwm/lpwm = raw & wd_motor_ok`。
