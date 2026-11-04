"""PL（FPGA）行为级仿真模型 + 直流电机/编码器物理模型。

这不是 RTL，只是按计划中的 PL 设计写的“金模型”，用于在没有比特流时跑通
导航 → Chassis API → 寄存器 → 控制链 → 编码器回传 的 HIL 闭环。
寄存器语义见 regmap.py；RTL 完成后应以 testbench 与本模型对比。

时序：物理积分 1 kHz；control_tick 100 Hz（测速、PID、Safety FSM、Watchdog 同一节拍）。
"""

import math
import random
from dataclasses import dataclass, field

from . import regmap as R


@dataclass
class MotorParams:
    counts_per_rev: int = 1560          # 输出轴每圈计数（13 PPR × 4 倍频 × 30 减速比，待实测）
    no_load_rad_s: float = 34.9         # 满占空比空载轮速（≈333 rpm，待实测）
    tau_s: float = 0.08                 # 一阶机械时间常数
    deadband: float = 0.08              # 静摩擦死区（占空比）
    gain_mismatch: tuple = (1.00, 0.93, 1.05, 0.97)  # 四轮个体差异，用于体现闭环价值
    noise_rad_s: float = 0.15


@dataclass
class PLConfig:
    wdt_timeout_ticks: int = 20         # 200 ms
    kp: float = 0.10
    ki: float = 0.012
    kd: float = 0.0
    pwm_max: int = 1000
    speed_window: int = 4               # 测速窗口（tick），40 ms 滑窗降低量化噪声
    build_id: int = 0x5100_0001         # 仿真构建号


@dataclass
class _Wheel:
    omega: float = 0.0                  # rad/s，真实轮速
    pos: float = 0.0                    # 连续计数位置
    count: int = 0
    hist: list = field(default_factory=list)
    target: int = 0
    speed: int = 0
    integ: int = 0                      # Q16.16
    prev_err: int = 0
    out: int = 0


class SimPL:
    PHYS_DT = 0.001

    def __init__(self, motor=None, cfg=None, seed=0):
        self.motor = motor or MotorParams()
        cfg = cfg or PLConfig()
        self.rng = random.Random(seed)
        self.wheels = [_Wheel(hist=[0] * (cfg.speed_window + 1)) for _ in range(4)]
        self.regs = {
            R.REG_WDT_TIMEOUT: cfg.wdt_timeout_ticks,
            R.REG_KP: R.q16(cfg.kp),
            R.REG_KI: R.q16(cfg.ki),
            R.REG_KD: R.q16(cfg.kd),
            R.REG_PWM_MAX: cfg.pwm_max,
        }
        self.build_id = cfg.build_id
        self.motor_enable = False
        self.sw_estop = False
        self.faults = 0
        self.state = R.SAFE_IDLE
        self.tick = 0
        self.ticks_since_hb = 0
        self.time = 0.0
        self._phys_acc = 0.0
        self._phys_steps = 0
        self._phys_per_tick = round(1.0 / (R.CONTROL_TICK_HZ * self.PHYS_DT))

    # ---------------- AXI 接口 ----------------
    def read(self, off):
        if off == R.REG_VERSION:
            return R.REG_VERSION_VALUE
        if off == R.REG_CTRL:
            return (R.CTRL_MOTOR_EN if self.motor_enable else 0) | (R.CTRL_SW_ESTOP if self.sw_estop else 0)
        if off == R.REG_SAFETY_STATE:
            return self.state
        if off == R.REG_FAULT_BITS:
            return self.faults
        if off == R.REG_TICK_COUNT:
            return R.to_u32(self.tick)
        if off == R.REG_BUILD_ID:
            return self.build_id
        for base, attr in ((R.REG_TARGET_BASE, "target"), (R.REG_SPEED_BASE, "speed"),
                           (R.REG_ENC_BASE, "count"), (R.REG_PIDOUT_BASE, "out")):
            if base <= off < base + 16 and off % 4 == 0:
                return R.to_u32(getattr(self.wheels[(off - base) // 4], attr))
        return self.regs.get(off, 0)

    def write(self, off, val):
        val = R.to_u32(val)
        if off == R.REG_CTRL:
            self.motor_enable = bool(val & R.CTRL_MOTOR_EN)
            self.sw_estop = bool(val & R.CTRL_SW_ESTOP)
            if self.sw_estop:
                self.faults |= R.FAULT_SW_ESTOP
            if val & R.CTRL_CLEAR_FAULT and not self.sw_estop:
                # 只有在 Heartbeat 新鲜时才允许清故障，防止上层已死时被误清
                if self.ticks_since_hb < self.regs[R.REG_WDT_TIMEOUT]:
                    self.faults = 0
        elif off == R.REG_HEARTBEAT:
            self.ticks_since_hb = 0
        elif R.REG_TARGET_BASE <= off < R.REG_TARGET_BASE + 16 and off % 4 == 0:
            self.wheels[(off - R.REG_TARGET_BASE) // 4].target = R.to_s32(val)
        elif off in (R.REG_WDT_TIMEOUT, R.REG_KP, R.REG_KI, R.REG_KD, R.REG_PWM_MAX):
            self.regs[off] = val
        # 只读寄存器写入被忽略

    # ---------------- 时间推进 ----------------
    def advance(self, dt):
        self._phys_acc += dt
        while self._phys_acc >= self.PHYS_DT - 1e-12:
            self._phys_acc -= self.PHYS_DT
            self._physics_step()
            self._phys_steps += 1
            if self._phys_steps % self._phys_per_tick == 0:
                self._control_tick()

    def wheel_omegas(self):
        return [w.omega for w in self.wheels]

    def _physics_step(self):
        m = self.motor
        pwm_max = max(1, self.regs[R.REG_PWM_MAX])
        dt = self.PHYS_DT
        for i, w in enumerate(self.wheels):
            duty = max(-1.0, min(1.0, w.out / pwm_max))
            mag = abs(duty)
            eff = 0.0 if mag < m.deadband else math.copysign((mag - m.deadband) / (1 - m.deadband), duty)
            omega_ss = eff * m.no_load_rad_s * m.gain_mismatch[i]
            w.omega += (omega_ss - w.omega) * dt / m.tau_s
            if eff:  # 负载扰动只在驱动时存在，静止轮不抖
                w.omega += self.rng.gauss(0.0, m.noise_rad_s) * math.sqrt(dt / m.tau_s)
            w.pos += w.omega * dt * m.counts_per_rev / (2 * math.pi)
            w.count = math.floor(w.pos)
        self.time += dt

    def _control_tick(self):
        self.tick += 1
        timeout = self.regs[R.REG_WDT_TIMEOUT]

        # Watchdog：只在使能时计时，超时锁存故障
        self.ticks_since_hb = min(self.ticks_since_hb + 1, 1 << 30)
        if self.motor_enable and self.ticks_since_hb >= timeout:
            self.faults |= R.FAULT_WDT

        # Safety FSM
        if self.faults:
            self.state = R.SAFE_ESTOP
        elif not self.motor_enable:
            self.state = R.SAFE_IDLE
        else:
            self.state = R.SAFE_RUN
        gate = self.state == R.SAFE_RUN

        kp, ki, kd = (R.to_s32(self.regs[r]) for r in (R.REG_KP, R.REG_KI, R.REG_KD))
        pwm_max = self.regs[R.REG_PWM_MAX]
        lim = pwm_max << 16
        hz = R.CONTROL_TICK_HZ
        for w in self.wheels:
            # 测速：窗口差分，单位 counts/s
            w.hist.append(w.count)
            w.hist.pop(0)
            n = len(w.hist) - 1
            w.speed = (w.hist[-1] - w.hist[0]) * hz // n

            if not gate:
                w.integ = 0
                w.prev_err = 0
                w.out = 0
                continue
            err = w.target - w.speed
            # 条件积分抗饱和：输出已饱和且误差同向时不再累积
            saturated = abs(w.out) >= pwm_max and (err > 0) == (w.out > 0)
            if not saturated:
                w.integ = max(-lim, min(lim, w.integ + ki * err))
            u = (kp * err + w.integ + kd * (err - w.prev_err)) >> 16
            w.prev_err = err
            w.out = max(-pwm_max, min(pwm_max, u))
