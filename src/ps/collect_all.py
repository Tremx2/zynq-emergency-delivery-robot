#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最新版全量数据采集：I2C 扫描 + IMU(ICM-42688-P) + ToF(VL53L1X) + 电量(INA219)。

一次跑完，输出一份 JSON（含原始样本 + 汇总统计）和一份文本日志，
保存到板上 /home/xilinx/jupyter_notebooks/collect_latest.{json,log}。

用法（板上，需 root + venv）：
    sudo /usr/local/share/pynq-venv/bin/python3 collect_all.py
"""
import datetime
import json
import math
import os
import statistics
import sys
import time

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

from pynq import Overlay, MMIO
sys.path.insert(0, "/home/xilinx/jupyter_notebooks")
from test_sensors import AxiIic, ICM42688, VL53L1X, I2CError

OVERLAY = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"
MOTOR_BASE = 0x43C00000
TOF_XSHUT = 0x50

OUT_JSON = "/home/xilinx/jupyter_notebooks/collect_latest.json"
OUT_LOG = "/home/xilinx/jupyter_notebooks/collect_latest.log"

IMU_ADDRS = (0x69, 0x68)   # AD0 高 -> 0x69，低 -> 0x68，自动识别
TOF_ADDR = 0x29
INA_ADDR = 0x40

IMU_N = 200
TOF_N = 50
BAT_N = 8

# 3S 锂电池 电压 -> 电量% 分段查表
LUT = [(12.60, 100.0), (12.00, 85.0), (11.40, 60.0),
       (10.80, 35.0), (10.20, 15.0), (9.60, 0.0)]


def battery_pct(v):
    if v >= LUT[0][0]:
        return 100.0
    if v <= LUT[-1][0]:
        return 0.0
    for (v1, p1), (v2, p2) in zip(LUT, LUT[1:]):
        if v2 <= v <= v1:
            return p2 + (p1 - p2) * (v - v2) / (v1 - v2)
    return 0.0


def main():
    log = []

    def L(s):
        print(s, flush=True)
        log.append(s)

    L("=== collect_all 最新版全量采集 ===")
    L("loading overlay ...")
    Overlay(OVERLAY)
    # 拉高 ToF XSHUT (W14, robot_motor_axi @0x50 bit0)，否则 0x29 不应答
    MMIO(MOTOR_BASE, 0x1000).write(TOF_XSHUT, 0x1)
    time.sleep(0.01)

    bus = AxiIic()
    found = bus.scan()
    bus.errors = 0
    L("scan: " + " ".join(f"0x{a:02X}" for a in found))

    result = {
        "timestamp": datetime.datetime.now().isoformat(),
        "scan": [f"0x{a:02X}" for a in found],
        "imu": {}, "tof": {}, "battery": {},
    }

    # ---- IMU ----
    imu_addr = next((a for a in IMU_ADDRS if a in found), None)
    if imu_addr is None:
        L("IMU: 未发现 (0x68/0x69)")
        result["imu"] = {"ok": False, "error": "未发现 ICM-42688"}
    else:
        imu = ICM42688(bus, addr=imu_addr)
        who = imu.who_am_i()
        imu.init()
        samples, fails = [], 0
        for _ in range(IMU_N):
            try:
                samples.append(imu.read())
            except I2CError:
                fails += 1
            time.sleep(0.01)
        cols = list(zip(*samples))
        mean = [statistics.fmean(c) for c in cols]
        std = [statistics.pstdev(c) for c in cols]
        norm = math.sqrt(mean[0] ** 2 + mean[1] ** 2 + mean[2] ** 2)
        result["imu"] = {
            "ok": 0.9 < norm < 1.1,
            "addr": imu_addr, "who_am_i": who,
            "mean": {"ax": mean[0], "ay": mean[1], "az": mean[2],
                     "gx": mean[3], "gy": mean[4], "gz": mean[5], "temp": mean[6]},
            "std": {"ax": std[0], "ay": std[1], "az": std[2],
                    "gx": std[3], "gy": std[4], "gz": std[5]},
            "norm_g": norm,
            "acc_noise_mg": max(std[:3]) * 1e3,
            "gyro_bias_dps": {"gx": mean[3], "gy": mean[4], "gz": mean[5]},
            "fails": fails, "count": len(samples),
            "samples": [list(s) for s in samples],
        }
        L(f"IMU @0x{imu_addr:02X} who=0x{who:02X} |a|={norm:.4f}g "
          f"acc_noise={max(std[:3]) * 1e3:.2f}mg "
          f"gyro_bias=({mean[3]:+.2f},{mean[4]:+.2f},{mean[5]:+.2f})dps "
          f"temp={mean[6]:.1f}C n={len(samples)} fails={fails}")

    # ---- ToF ----
    if TOF_ADDR not in found:
        L(f"TOF: 0x{TOF_ADDR:02X} 不在总线")
        result["tof"] = {"ok": False, "error": f"0x{TOF_ADDR:02X} 不在总线"}
    else:
        tof = VL53L1X(bus, addr=TOF_ADDR)
        info = tof.model_info()
        tof.init()
        tof.start()
        dists = []
        for _ in range(TOF_N):
            tof.wait_ready()
            d, st, sig, amb = tof.read()
            dists.append({"dist_mm": d, "status": st,
                          "signal_kcps": sig, "ambient_kcps": amb})
            time.sleep(0.05)
        tof.stop()
        valid = [x["dist_mm"] for x in dists if x["status"] == 0 and x["dist_mm"] > 0]
        result["tof"] = {
            "ok": len(valid) >= int(0.8 * TOF_N),
            "addr": TOF_ADDR, "model_id": list(info),
            "valid_count": len(valid), "total": len(dists),
            "mean_mm": statistics.fmean(valid) if valid else None,
            "std_mm": statistics.pstdev(valid) if len(valid) >= 2 else None,
            "samples": dists,
        }
        L(f"TOF @0x{TOF_ADDR:02X} model={' '.join(f'0x{x:02X}' for x in info)} "
          f"valid={len(valid)}/{len(dists)} "
          f"mean={result['tof']['mean_mm']}mm std={result['tof']['std_mm']}mm")

    # ---- 电量 (INA219) ----
    if INA_ADDR not in found:
        L(f"BATTERY: 0x{INA_ADDR:02X} 不在总线")
        result["battery"] = {"ok": False, "error": f"0x{INA_ADDR:02X} 不在总线"}
    else:
        bus.write(INA_ADDR, bytes([0x00, 0x39, 0x9F]))   # config 0x399F
        bus.write(INA_ADDR, bytes([0x05, 0x10, 0x00]))   # cal 0x1000 (0.1Ω)
        volts = []
        for _ in range(BAT_N):
            raw = int.from_bytes(bus.write_read(INA_ADDR, bytes([0x02]), 2), "big")
            volts.append((raw >> 3) * 0.004)
            time.sleep(0.3)
        avg = statistics.fmean(volts)
        pct = battery_pct(avg)
        result["battery"] = {
            "ok": 8.0 < avg < 13.0,
            "addr": INA_ADDR,
            "voltage_v": avg,
            "percent": pct,
            "samples": volts,
        }
        L(f"BATTERY @0x{INA_ADDR:02X}: {avg:.3f}V  {pct:.0f}%  "
          f"(samples={[f'{x:.3f}' for x in volts]})")

    # ---- 保存 ----
    with open(OUT_JSON, "w") as f:
        json.dump(result, f)
    with open(OUT_LOG, "w") as f:
        f.write("\n".join(log) + "\n")
    L(f"saved {OUT_JSON}")
    L("DONE")


if __name__ == "__main__":
    main()
