"""AXI-Lite 寄存器映射（草案 v0.1）。

RTL、仿真 PL 模型（sim_pl.py）、Chassis 驱动（chassis.py）共用这一份定义，
修改时必须同步 docs/register_map.md 并升 REG_VERSION。

所有寄存器 32 bit；有符号量按二进制补码存放。
"""

# 版本号：0x00MMmmpp（major.minor.patch）
REG_VERSION_VALUE = 0x00000100  # v0.1.0

REG_VERSION = 0x00
REG_CTRL = 0x04            # bit0 motor_enable, bit1 clear_fault(写 1 脉冲), bit2 sw_estop
REG_HEARTBEAT = 0x08       # 任意写入即喂狗
REG_SAFETY_STATE = 0x0C    # 只读，SAFE_* 枚举
REG_FAULT_BITS = 0x10      # 只读，FAULT_* 位，锁存直到 clear_fault
REG_TICK_COUNT = 0x14      # 只读，control_tick 计数（时间戳）
REG_WDT_TIMEOUT = 0x18     # Watchdog 超时，单位 control_tick
REG_BUILD_ID = 0x1C        # 只读，bitstream 构建号

REG_TARGET_BASE = 0x20     # 4 × int32，目标轮速，counts/s
REG_SPEED_BASE = 0x30      # 4 × int32，实测轮速，counts/s（只读）
REG_ENC_BASE = 0x40        # 4 × int32，编码器累计计数（只读）
REG_PIDOUT_BASE = 0x50     # 4 × int32，PID 输出 = 有符号 PWM 占空计数（只读）

REG_KP = 0x60              # Q16.16，单位 PWM count / (counts/s)
REG_KI = 0x64              # Q16.16，每个 control_tick 累加
REG_KD = 0x68              # Q16.16
REG_PWM_MAX = 0x6C         # PWM 满量程计数

CTRL_MOTOR_EN = 1 << 0
CTRL_CLEAR_FAULT = 1 << 1
CTRL_SW_ESTOP = 1 << 2

SAFE_IDLE = 0      # 复位默认态：motor_enable=0, PWM=0
SAFE_RUN = 1
SAFE_WARNING = 2   # 预留给 ToF 近距减速（当前未建模）
SAFE_ESTOP = 3     # 故障锁存，Motor Gate 关闭

SAFE_NAMES = {SAFE_IDLE: "IDLE", SAFE_RUN: "RUN", SAFE_WARNING: "WARNING", SAFE_ESTOP: "E_STOP"}

FAULT_WDT = 1 << 0       # PS Heartbeat 超时
FAULT_TOF = 1 << 1       # 预留：ToF 过近（当前未建模）
FAULT_SW_ESTOP = 1 << 2  # 软件急停

WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR = range(4)
WHEEL_NAMES = ("FL", "FR", "RL", "RR")

CONTROL_TICK_HZ = 100


def to_u32(v):
    return int(v) & 0xFFFFFFFF


def to_s32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


def q16(x):
    return int(round(x * 65536))


def fault_names(bits):
    names = []
    if bits & FAULT_WDT:
        names.append("WDT")
    if bits & FAULT_TOF:
        names.append("TOF")
    if bits & FAULT_SW_ESTOP:
        names.append("SW_ESTOP")
    return names
