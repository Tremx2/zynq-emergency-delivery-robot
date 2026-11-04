"""统一 Chassis API —— 导航层/应用层唯一允许的底盘入口。

寄存器后端可以是：
  * SimPL（pynq_sw/drivers/sim_pl.py）：纯仿真
  * PynqMMIO（本文件）：在 PYNQ-Z2 上通过 pynq.MMIO 访问真实 PL
两者实现同样的 read(off)/write(off, val) 接口，导航代码不感知差异。
"""

import math
from dataclasses import dataclass

from . import regmap as R


@dataclass
class ChassisParams:
    wheel_radius_m: float = 0.0325     # 待实测
    track_width_m: float = 0.20        # 四轮差速的“等效”轮距，需标定（含打滑）
    counts_per_rev: int = 1560
    max_wheel_speed_mps: float = 0.8

    @property
    def m_per_count(self):
        return 2 * math.pi * self.wheel_radius_m / self.counts_per_rev


@dataclass
class Telemetry:
    tick: int
    state: int
    faults: int
    target: list
    speed: list
    enc: list
    pid_out: list

    @property
    def state_name(self):
        return R.SAFE_NAMES.get(self.state, f"?{self.state}")


class PynqMMIO:
    """真实板卡寄存器后端（需在 PYNQ 上运行，地址取自 overlay 的 hwh）。"""

    def __init__(self, base_addr, length=0x100):
        from pynq import MMIO  # 仅板上可用
        self._mmio = MMIO(base_addr, length)

    def read(self, off):
        return self._mmio.read(off)

    def write(self, off, val):
        self._mmio.write(off, R.to_u32(val))


class Chassis:
    def __init__(self, regs, params=None):
        self.regs = regs
        self.p = params or ChassisParams()
        ver = regs.read(R.REG_VERSION)
        # 必须完整比较: v0.1 的主版本号是 0, 只比高 16 位时真实 bitstream
        # (0x00 是 ENABLE 寄存器, 读回 0) 也会被误判为匹配。
        if ver != R.REG_VERSION_VALUE:
            raise RuntimeError(f"寄存器版本不匹配: PL=0x{ver:08X}, 驱动=0x{R.REG_VERSION_VALUE:08X}")
        self._ctrl = 0

    # ---- 控制面 ----
    def configure(self, kp=None, ki=None, kd=None, wdt_timeout_ms=None):
        for reg, val in ((R.REG_KP, kp), (R.REG_KI, ki), (R.REG_KD, kd)):
            if val is not None:
                self.regs.write(reg, R.q16(val))
        if wdt_timeout_ms is not None:
            self.regs.write(R.REG_WDT_TIMEOUT, max(1, round(wdt_timeout_ms * R.CONTROL_TICK_HZ / 1000)))

    def _write_ctrl(self, extra=0):
        self.regs.write(R.REG_CTRL, self._ctrl | extra)

    def enable(self):
        self._ctrl = (self._ctrl | R.CTRL_MOTOR_EN) & ~R.CTRL_SW_ESTOP
        self._write_ctrl()

    def disable(self):
        self._ctrl &= ~R.CTRL_MOTOR_EN
        self._write_ctrl()

    def estop(self):
        self._ctrl |= R.CTRL_SW_ESTOP
        self._write_ctrl()

    def clear_fault(self):
        self._ctrl &= ~R.CTRL_SW_ESTOP
        self.heartbeat()
        self._write_ctrl(R.CTRL_CLEAR_FAULT)
        self._write_ctrl()

    def heartbeat(self):
        self.regs.write(R.REG_HEARTBEAT, 1)

    # ---- 运动命令 ----
    def mps_to_cps(self, v):
        return int(round(v / self.p.m_per_count))

    def set_velocity(self, v, w):
        """v: m/s，w: rad/s（逆时针为正）。超限时按比例缩放，保持曲率。"""
        half = self.p.track_width_m / 2
        vl, vr = v - w * half, v + w * half
        peak = max(abs(vl), abs(vr))
        if peak > self.p.max_wheel_speed_mps:
            k = self.p.max_wheel_speed_mps / peak
            vl, vr = vl * k, vr * k
        cl, cr = self.mps_to_cps(vl), self.mps_to_cps(vr)
        self.set_wheel_targets([cl, cr, cl, cr])

    def set_wheel_targets(self, cps):
        """直接设置四轮目标（counts/s），顺序 FL, FR, RL, RR。用于 Demo A。"""
        for i, c in enumerate(cps):
            self.regs.write(R.REG_TARGET_BASE + 4 * i, c)

    def stop(self):
        self.set_wheel_targets([0, 0, 0, 0])

    # ---- 状态回读 ----
    def read_telemetry(self):
        rd = self.regs.read
        return Telemetry(
            tick=rd(R.REG_TICK_COUNT),
            state=rd(R.REG_SAFETY_STATE),
            faults=rd(R.REG_FAULT_BITS),
            target=[R.to_s32(rd(R.REG_TARGET_BASE + 4 * i)) for i in range(4)],
            speed=[R.to_s32(rd(R.REG_SPEED_BASE + 4 * i)) for i in range(4)],
            enc=[R.to_s32(rd(R.REG_ENC_BASE + 4 * i)) for i in range(4)],
            pid_out=[R.to_s32(rd(R.REG_PIDOUT_BASE + 4 * i)) for i in range(4)],
        )


class Odometry:
    """PS 侧差速里程计：由四轮编码器增量积分位姿（左=FL/RL 平均，右=FR/RR 平均）。"""

    def __init__(self, params, x=0.0, y=0.0, th=0.0):
        self.p = params
        self.x, self.y, self.th = x, y, th
        self._last = None
        self.distance = 0.0

    def reset(self, x, y, th, enc=None):
        self.x, self.y, self.th = x, y, th
        self._last = list(enc) if enc is not None else None
        self.distance = 0.0

    def update(self, enc):
        if self._last is None:
            self._last = list(enc)
            return
        d = [R.to_s32(e - l) for e, l in zip(enc, self._last)]
        self._last = list(enc)
        k = self.p.m_per_count
        dl = (d[R.WHEEL_FL] + d[R.WHEEL_RL]) / 2 * k
        dr = (d[R.WHEEL_FR] + d[R.WHEEL_RR]) / 2 * k
        ds = (dl + dr) / 2
        dth = (dr - dl) / self.p.track_width_m
        mid = self.th + dth / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = (self.th + dth + math.pi) % (2 * math.pi) - math.pi
        self.distance += abs(ds)
