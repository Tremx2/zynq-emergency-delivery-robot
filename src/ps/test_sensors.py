#!/usr/bin/env python3
"""传感器测试: IMU (ICM-42688-P @0x69) + TOF (VL53L1X @0x29)

两个传感器都挂在 PL 的 axi_iic_0 (0x41600000, 100kHz, SCL=T10 / SDA=T11) 上。
这里直接用 MMIO 轮询操作 AXI IIC 寄存器 (动态模式), 不依赖中断 / Linux 驱动 / pynq AxiIIC。

用法:
  python3 test_sensors.py          全部测试 (总线扫描 + IMU 静止测试 + TOF 测距测试)
  python3 test_sensors.py scan     只扫描 I2C 总线
  python3 test_sensors.py imu      只测 IMU
  python3 test_sensors.py tof      只测 TOF
  python3 test_sensors.py live     IMU + TOF 实时打印 (Ctrl+C 停), 用来手动晃 IMU / 挡 TOF
"""
import math
import os
import statistics
import sys
import time

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

from pynq import Overlay, MMIO, PL

BIT_PATH = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"
IIC_BASE = 0x41600000

IMU_ADDR = 0x69   # 实车 AD0 接高, 2026-10-30 实测 WHO_AM_I=0x47 @0x69
TOF_ADDR = 0x29
MOTOR_BASE = 0x43C00000     # robot_motor_axi (电机 IP), 复用其 0x50 寄存器拉 TOF XSHUT
TOF_XSHUT_ADDR = 0x50       # bit0 = TOF XSHUT (PMOD B pin1 = W14)


# ============================================================
# AXI IIC (PG090) 寄存器, 动态模式轮询驱动
# ============================================================

ISR, SOFTR, CR, SR = 0x020, 0x040, 0x100, 0x104
TX_FIFO, RX_FIFO, RX_FIFO_PIRQ = 0x108, 0x10C, 0x120

SR_BB, SR_TX_FULL, SR_RX_EMPTY, SR_TX_EMPTY = 0x04, 0x10, 0x40, 0x80
ISR_ARB_LOST, ISR_TX_ERR = 0x01, 0x02

START, STOP = 0x100, 0x200


class I2CError(Exception):
    pass


class AxiIic:
    def __init__(self, base=IIC_BASE, timeout=0.05):
        self.mmio = MMIO(base, 0x1000)
        self.timeout = timeout
        self.errors = 0
        self.reset()

    def reset(self):
        m = self.mmio
        m.write(SOFTR, 0xA)
        time.sleep(0.001)
        m.write(RX_FIFO_PIRQ, 0x0F)
        m.write(CR, 0x02)          # 复位 TX FIFO
        m.write(CR, 0x01)          # 使能, 解除 TX FIFO 复位
        m.write(ISR, m.read(ISR))  # 清中断状态 (toggle-on-write)

    def _check_err(self, mask=ISR_TX_ERR | ISR_ARB_LOST):
        isr = self.mmio.read(ISR)
        if isr & mask:
            what = "arbitration lost" if isr & ISR_ARB_LOST else "NACK"
            raise I2CError(f"{what} (ISR=0x{isr:02X} SR=0x{self.mmio.read(SR):02X})")

    def _wait(self, cond, what, mask=ISR_TX_ERR | ISR_ARB_LOST):
        t_end = time.monotonic() + self.timeout
        while not cond():
            self._check_err(mask)
            if time.monotonic() > t_end:
                raise I2CError(f"timeout ({what}), SR=0x{self.mmio.read(SR):02X} "
                               f"ISR=0x{self.mmio.read(ISR):02X}")

    def _push(self, word):
        self._wait(lambda: not self.mmio.read(SR) & SR_TX_FULL, "tx fifo full")
        self.mmio.write(TX_FIFO, word)

    def _begin(self):
        self._wait(lambda: not self.mmio.read(SR) & SR_BB, "bus busy")
        self.mmio.write(ISR, self.mmio.read(ISR))

    def _finish(self):
        self._wait(lambda: self.mmio.read(SR) & SR_TX_EMPTY, "tx drain")
        self._wait(lambda: not self.mmio.read(SR) & SR_BB, "stop")
        self._check_err()

    def _guard(self, fn):
        try:
            return fn()
        except I2CError:
            self.errors += 1
            self.reset()
            raise

    def write(self, addr, data):
        """写 data (bytes, 至少 1 字节), 最后一个字节带 STOP"""
        def go():
            self._begin()
            self._push(START | (addr << 1))
            for i, b in enumerate(data):
                self._push((STOP if i == len(data) - 1 else 0) | b)
            self._finish()
        self._guard(go)

    def write_read(self, addr, wdata, n):
        """写 wdata (寄存器地址) -> 重复起始 -> 读 n 字节 (n<=255)"""
        def go():
            self._begin()
            if wdata:
                self._push(START | (addr << 1))
                for b in wdata:
                    self._push(b)
            self._push(START | (addr << 1) | 1)
            self._push(STOP | n)
            # 注意: 主机读最后一个字节时自己回 NACK, 这也会置 ISR.TX_ERR,
            # 所以只有 "一个字节都没收到 + 总线已空闲" 时 TX_ERR 才代表从机没应答
            out = bytearray()
            t_end = time.monotonic() + self.timeout
            while len(out) < n:
                sr = self.mmio.read(SR)
                if not sr & SR_RX_EMPTY:
                    out.append(self.mmio.read(RX_FIFO) & 0xFF)
                    continue
                self._check_err(ISR_ARB_LOST)
                if not out and not sr & SR_BB and self.mmio.read(ISR) & ISR_TX_ERR:
                    raise I2CError(f"NACK (ISR=0x{self.mmio.read(ISR):02X} SR=0x{sr:02X})")
                if time.monotonic() > t_end:
                    raise I2CError(f"timeout (rx {len(out)}/{n}), SR=0x{sr:02X} "
                                   f"ISR=0x{self.mmio.read(ISR):02X}")
            self._wait(lambda: not self.mmio.read(SR) & SR_BB, "stop", ISR_ARB_LOST)
            return bytes(out)
        return self._guard(go)

    def probe(self, addr):
        try:
            self.write_read(addr, b"", 1)
            return True
        except I2CError:
            return False

    def scan(self):
        return [a for a in range(0x08, 0x78) if self.probe(a)]


# ============================================================
# ICM-42688-P
# ============================================================

class ICM42688:
    WHO_AM_I, WHO_AM_I_VAL = 0x75, 0x47
    DEVICE_CONFIG, BANK_SEL = 0x11, 0x76
    PWR_MGMT0, GYRO_CONFIG0, ACCEL_CONFIG0 = 0x4E, 0x4F, 0x50
    TEMP_DATA1 = 0x1D

    ACC_LSB_PER_G = 8192.0    # ±4g
    GYRO_LSB_PER_DPS = 65.5   # ±500dps

    def __init__(self, bus, addr=IMU_ADDR):
        self.bus, self.addr = bus, addr

    def rd(self, reg, n=1):
        return self.bus.write_read(self.addr, bytes([reg]), n)

    def wr(self, reg, val):
        self.bus.write(self.addr, bytes([reg, val]))

    def who_am_i(self):
        return self.rd(self.WHO_AM_I)[0]

    def init(self):
        self.wr(self.BANK_SEL, 0x00)
        self.wr(self.DEVICE_CONFIG, 0x01)   # 软复位
        time.sleep(0.01)
        self.wr(self.BANK_SEL, 0x00)
        self.wr(self.GYRO_CONFIG0, 0x48)    # ±500dps, 100Hz
        self.wr(self.ACCEL_CONFIG0, 0x48)   # ±4g, 100Hz
        self.wr(self.PWR_MGMT0, 0x0F)       # 陀螺 + 加计 低噪声模式
        time.sleep(0.1)                     # 陀螺启动需要 >45ms
        return self.rd(self.PWR_MGMT0)[0] == 0x0F

    def read(self):
        """返回 (ax, ay, az [g], gx, gy, gz [dps], temp [°C])"""
        d = self.rd(self.TEMP_DATA1, 14)
        v = [int.from_bytes(d[i:i + 2], "big", signed=True) for i in range(0, 14, 2)]
        acc = [x / self.ACC_LSB_PER_G for x in v[1:4]]
        gyr = [x / self.GYRO_LSB_PER_DPS for x in v[4:7]]
        return (*acc, *gyr, v[0] / 132.48 + 25.0)


# ============================================================
# VL53L1X (ST ULD 精简移植, 参考 adafruit_vl53l1x)
# ============================================================

VL53L1X_DEFAULT_CONFIG = bytes([
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
])  # 0x2D ~ 0x87

# RESULT__RANGE_STATUS 低 5 位 -> ULD RangeStatus (0 = 有效)
_RANGE_STATUS_MAP = [255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255,
                     255, 255, 255, 10, 6, 255, 255, 11, 12]
RANGE_STATUS_TEXT = {0: "OK", 1: "sigma fail", 2: "signal fail", 4: "out of bounds",
                     7: "wrap around", 255: "unknown"}


class VL53L1X:
    MODEL_ID = 0x010F
    FIRMWARE_SYSTEM_STATUS = 0x00E5
    GPIO_HV_MUX_CTRL, GPIO_TIO_HV_STATUS = 0x0030, 0x0031
    SYSTEM_INTERRUPT_CLEAR, SYSTEM_MODE_START = 0x0086, 0x0087
    RESULT_RANGE_STATUS = 0x0089

    def __init__(self, bus, addr=TOF_ADDR):
        self.bus, self.addr = bus, addr

    def rd(self, reg, n=1):
        return self.bus.write_read(self.addr, reg.to_bytes(2, "big"), n)

    def wr(self, reg, data):
        # TX FIFO 由 _push 边发边填, 长数据也没问题
        self.bus.write(self.addr, reg.to_bytes(2, "big") + bytes(data))

    def model_info(self):
        return tuple(self.rd(self.MODEL_ID, 3))   # (0xEA, 0xCC, 0x10)

    def init(self, timeout=1.0):
        t_end = time.monotonic() + timeout
        while not self.rd(self.FIRMWARE_SYSTEM_STATUS)[0] & 0x01:
            if time.monotonic() > t_end:
                raise RuntimeError("VL53L1X 固件未启动 (0x00E5 bit0 一直为 0)")
            time.sleep(0.005)
        self.wr(0x002D, VL53L1X_DEFAULT_CONFIG)
        self.start()
        self.wait_ready()
        self.clear_int()
        self.stop()
        self.wr(0x0008, [0x09])   # VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND
        self.wr(0x000B, [0x00])
        # 默认即长距离模式, timing budget 改成 50ms
        self.wr(0x005E, [0x00, 0xAD])
        self.wr(0x0061, [0x00, 0xC6])

    def start(self):
        self.wr(self.SYSTEM_MODE_START, [0x40])

    def stop(self):
        self.wr(self.SYSTEM_MODE_START, [0x00])

    def clear_int(self):
        self.wr(self.SYSTEM_INTERRUPT_CLEAR, [0x01])

    def data_ready(self):
        pol = 0 if self.rd(self.GPIO_HV_MUX_CTRL)[0] & 0x10 else 1
        return (self.rd(self.GPIO_TIO_HV_STATUS)[0] & 0x01) == pol

    def wait_ready(self, timeout=1.0):
        t_end = time.monotonic() + timeout
        while not self.data_ready():
            if time.monotonic() > t_end:
                raise RuntimeError("VL53L1X 测距超时 (data ready 一直不来)")
            time.sleep(0.002)

    def read(self):
        """返回 (distance_mm, range_status, signal_kcps, ambient_kcps), 并清中断"""
        r = self.rd(self.RESULT_RANGE_STATUS, 17)   # 0x0089 ~ 0x0099
        raw = r[0] & 0x1F
        status = _RANGE_STATUS_MAP[raw] if raw < len(_RANGE_STATUS_MAP) else 255
        ambient = int.from_bytes(r[7:9], "big") * 8      # 0x0090
        dist = int.from_bytes(r[13:15], "big")           # 0x0096
        signal = int.from_bytes(r[15:17], "big") * 8     # 0x0098
        self.clear_int()
        return dist, status, signal, ambient


# ============================================================
# 测试项
# ============================================================

results = {}


def report(name, ok, detail=""):
    results[name] = ok
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def test_scan(bus):
    print("\n=== I2C 总线扫描 ===", flush=True)
    found = bus.scan()
    bus.errors = 0   # 扫描时空地址的 NACK 不算错误
    print("  发现设备: " + (" ".join(f"0x{a:02X}" for a in found) or "无"), flush=True)
    report(f"IMU 0x{IMU_ADDR:02X} 应答", IMU_ADDR in found,
           "" if IMU_ADDR in found else "(若 AD0 接地地址会是 0x68)")
    report("TOF 0x29 应答", TOF_ADDR in found)
    return found


def test_imu(bus, n=200):
    print("\n=== IMU (ICM-42688-P) ===", flush=True)
    imu = ICM42688(bus)
    try:
        who = imu.who_am_i()
    except Exception as e:
        report("IMU WHO_AM_I", False, f"读失败: {e}")
        return
    report("IMU WHO_AM_I", who == ICM42688.WHO_AM_I_VAL, f"= 0x{who:02X} (期望 0x47)")
    if who != ICM42688.WHO_AM_I_VAL:
        return
    report("IMU 初始化", imu.init())

    print(f"  采样 {n} 次 (@~100Hz), 请保持静止 ...", flush=True)
    samples, fails = [], 0
    t0 = time.monotonic()
    for _ in range(n):
        try:
            samples.append(imu.read())
        except I2CError:
            fails += 1
        time.sleep(0.01)
    dt = time.monotonic() - t0
    report("IMU 连续读取", fails == 0 and len(samples) > 0,
           f"成功 {len(samples)}/{n}, 失败 {fails}, 实际 {len(samples) / dt:.0f} Hz")
    if len(samples) < 10:
        return

    cols = list(zip(*samples))
    mean = [statistics.fmean(c) for c in cols]
    std = [statistics.pstdev(c) for c in cols]
    ax, ay, az, gx, gy, gz, temp = mean
    norm = math.sqrt(ax * ax + ay * ay + az * az)

    print(f"  加速度均值 (g):   X={ax:+.3f} Y={ay:+.3f} Z={az:+.3f}   |a|={norm:.3f}")
    print(f"  加速度噪声 (mg):  X={std[0]*1e3:.2f} Y={std[1]*1e3:.2f} Z={std[2]*1e3:.2f}")
    print(f"  陀螺零偏 (dps):   X={gx:+.3f} Y={gy:+.3f} Z={gz:+.3f}")
    print(f"  陀螺噪声 (dps):   X={std[3]:.3f} Y={std[4]:.3f} Z={std[5]:.3f}")
    print(f"  温度: {temp:.1f} °C", flush=True)

    all_zero = all(abs(v) < 1e-9 for c in cols[:6] for v in c)
    frozen = all(s == 0 for s in std[:6])
    report("IMU 数据非全 0 / 非冻结", not all_zero and not frozen)
    report("IMU 重力模长 ≈ 1g", 0.9 < norm < 1.1, f"(|a|={norm:.3f}g)")
    report("IMU 陀螺零偏 < 5 dps", max(abs(gx), abs(gy), abs(gz)) < 5.0)
    report("IMU 噪声正常", max(std[:3]) < 0.02 and max(std[3:6]) < 0.5,
           "(加计 <20mg, 陀螺 <0.5dps; 不过多半是测试时没放稳)")
    report("IMU 温度合理", 0 < temp < 70)
    up = max(range(3), key=lambda i: abs(mean[i]))
    print(f"  -> 当前竖直方向是 {'XYZ'[up]} 轴 ({'+' if mean[up] > 0 else '-'})", flush=True)


def test_tof(bus, n=50):
    print("\n=== TOF (VL53L1X) ===", flush=True)
    tof = VL53L1X(bus)
    try:
        info = tof.model_info()
    except Exception as e:
        report("TOF 型号 ID", False, f"读失败: {e}")
        return
    report("TOF 型号 ID", info == (0xEA, 0xCC, 0x10),
           "= " + " ".join(f"0x{x:02X}" for x in info) + " (期望 0xEA 0xCC 0x10)")
    if info[0] != 0xEA:
        return
    try:
        tof.init()
        report("TOF 初始化", True)
    except Exception as e:
        report("TOF 初始化", False, str(e))
        return

    print(f"  连续测距 {n} 次 (50ms 一次), 请在传感器前 10cm~2m 处放一个物体 ...", flush=True)
    tof.start()
    dists, bad, fails = [], 0, 0
    t0 = time.monotonic()
    try:
        for i in range(n):
            try:
                tof.wait_ready()
                d, st, sig, amb = tof.read()
            except (I2CError, RuntimeError) as e:
                fails += 1
                print(f"    #{i:02d} 错误: {e}")
                continue
            if st == 0:
                dists.append(d)
            else:
                bad += 1
            if i % 5 == 0:
                print(f"    #{i:02d} dist={d:5d} mm  status={st}({RANGE_STATUS_TEXT.get(st, st)}) "
                      f"signal={sig} kcps ambient={amb} kcps", flush=True)
    finally:
        tof.stop()
    dt = time.monotonic() - t0

    report("TOF 测距无通信错误", fails == 0, f"(失败 {fails}/{n}, {n / dt:.1f} Hz)")
    report("TOF 有效测距 ≥ 80%", len(dists) >= 0.8 * n,
           f"(有效 {len(dists)}/{n}, 无效状态 {bad})")
    if len(dists) >= 2:
        mean, std = statistics.fmean(dists), statistics.pstdev(dists)
        print(f"  距离: 均值 {mean:.1f} mm, 标准差 {std:.1f} mm, "
              f"范围 {min(dists)}~{max(dists)} mm", flush=True)
        report("TOF 读数稳定 (std < 15mm)", std < 15, "(物体/传感器不动时)")


def live(bus):
    imu, tof = ICM42688(bus), VL53L1X(bus)
    has_imu = has_tof = False
    try:
        has_imu = imu.who_am_i() == ICM42688.WHO_AM_I_VAL and imu.init()
    except I2CError:
        pass
    try:
        tof.init()
        tof.start()
        has_tof = True
    except (I2CError, RuntimeError):
        pass
    print(f"IMU={'OK' if has_imu else '无'}  TOF={'OK' if has_tof else '无'}  (Ctrl+C 停止)", flush=True)
    dist_txt = "  ---"
    try:
        while True:
            line = ""
            if has_imu:
                ax, ay, az, gx, gy, gz, t = imu.read()
                line += (f"ACC(g) {ax:+6.2f} {ay:+6.2f} {az:+6.2f} | "
                         f"GYRO(dps) {gx:+7.1f} {gy:+7.1f} {gz:+7.1f} | ")
            if has_tof and tof.data_ready():
                d, st, _, _ = tof.read()
                dist_txt = f"{d:5d}mm" if st == 0 else f"{d:5d}mm({RANGE_STATUS_TEXT.get(st, st)})"
            if has_tof:
                line += f"TOF {dist_txt}"
            print(line, flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if has_tof:
            tof.stop()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"

    if PL.bitfile_name and os.path.basename(PL.bitfile_name) == os.path.basename(BIT_PATH):
        print("overlay 已加载, 不重复下载", flush=True)
        Overlay(BIT_PATH, download=False)
    else:
        print("加载 overlay ...", flush=True)
        Overlay(BIT_PATH)
    # 拉高 TOF XSHUT (W14), 否则 VL53L1X 处于硬件关断、0x29 不应答
    motor = MMIO(MOTOR_BASE, 0x1000)
    motor.write(TOF_XSHUT_ADDR, 0x1)
    time.sleep(0.01)

    bus = AxiIic()

    if mode == "live":
        live(bus)
        return
    if mode in ("all", "scan"):
        test_scan(bus)
    if mode in ("all", "imu"):
        test_imu(bus)
    if mode in ("all", "tof"):
        test_tof(bus)

    print(f"\n=== 汇总: {sum(results.values())}/{len(results)} 通过, I2C 错误重试 {bus.errors} 次 ===")
    for k, v in results.items():
        if not v:
            print(f"  FAIL: {k}")


if __name__ == "__main__":
    main()
