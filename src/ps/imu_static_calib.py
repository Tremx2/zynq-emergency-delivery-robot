#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU (ICM-42688-P) 静态校准 / 解算（逐姿态版）。

本版改成"一次只采一个姿态"，避免中途断线丢全部数据；每个姿态采完自动判断
重力落在哪根轴并做校验，最后 solve 用六面数据解加速度计零偏+标度。

用法（在板上）:
  python3 imu_static_calib.py level                      # 单面水平(陀螺零偏+安装角)
  python3 imu_static_calib.py collect <key>              # 采一个姿态
  python3 imu_static_calib.py solve                      # 用已采的姿态解算

<key> ∈ {flat, flipped, left, right, front, rear}，对应机器人的 6 个摆放：
  flat     放平（四轮着地）
  flipped  翻面（四轮朝天）
  left     左侧朝上（侧立，左轮朝天）
  right    右侧朝上（侧立，右轮朝天）
  front    车头朝上（车尾着地，头竖直朝上）
  rear     车尾朝上（车头着地，尾竖直朝上）

结果存:
  /home/xilinx/jupyter_notebooks/imu_faces.json   六面原始均值
  /home/xilinx/jupyter_notebooks/imu_calib.json   最终标定(零偏+标度+陀螺+安装角)
"""
import os
import sys
import time
import json
import math
import statistics

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

from pynq import Overlay, MMIO, PL
from test_sensors import AxiIic, ICM42688, I2CError

BIT_PATH = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"
FACES_PATH = "/home/xilinx/jupyter_notebooks/imu_faces.json"
CALIB_PATH = "/home/xilinx/jupyter_notebooks/imu_calib.json"

N_SAMPLES = 500
COUNTDOWN = 10
SAMPLE_DT = 0.01

FACES = {
    "flat":    "放平（四轮着地，正常水平）",
    "flipped": "翻面（四轮朝天，倒扣）",
    "left":    "左侧朝上（侧立，左轮朝天）",
    "right":   "右侧朝上（侧立，右轮朝天）",
    "front":   "车头朝上（车尾着地，头竖直朝上）",
    "rear":    "车尾朝上（车头着地，尾竖直朝上）",
}
AXIS_NAMES = "XYZ"


def countdown(sec):
    for i in range(sec, 0, -1):
        print(f"    {i} ...", end="\r", flush=True)
        time.sleep(1)
    print("    GO           ", flush=True)


def collect(imu, n, dt):
    samples, fails = [], 0
    for _ in range(n):
        try:
            samples.append(imu.read())
        except I2CError:
            fails += 1
        time.sleep(dt)
    if not samples:
        raise RuntimeError("无有效采样")
    cols = list(zip(*samples))
    mean = [statistics.fmean(c) for c in cols]
    std = [statistics.pstdev(c) for c in cols]
    return mean, std, fails


def roll_pitch(ax, ay, az):
    return math.degrees(math.atan2(ay, az)), math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))


def load_overlay():
    if PL.bitfile_name and os.path.basename(PL.bitfile_name) == os.path.basename(BIT_PATH):
        Overlay(BIT_PATH, download=False)
        print("overlay 已加载, 复用", flush=True)
    else:
        Overlay(BIT_PATH)
        print("overlay 已加载", flush=True)


def make_imu():
    bus = AxiIic()
    imu = ICM42688(bus)
    who = imu.who_am_i()
    print(f"IMU WHO_AM_I = 0x{who:02X}", flush=True)
    if who != ICM42688.WHO_AM_I_VAL:
        print("IMU 未识别", flush=True)
        sys.exit(1)
    if not imu.init():
        print("IMU 初始化失败", flush=True)
        sys.exit(1)
    print("IMU 初始化 OK (±4g, ±500dps, 100Hz)", flush=True)
    return imu


def dominant_axis(mean):
    """返回 (主轴下标, 符号, 主轴分量g)。主轴 = 绝对值最大的分量。"""
    a = [mean[0], mean[1], mean[2]]
    i = max(range(3), key=lambda k: abs(a[k]))
    return i, "+" if a[i] >= 0 else "-", a[i]


def do_level(imu):
    print("\n=== 水平静止（陀螺零偏 + 安装角）===", flush=True)
    print("  请把机器人放平，保持完全静止，不要碰它。", flush=True)
    countdown(COUNTDOWN)
    mean, std, fails = collect(imu, N_SAMPLES, SAMPLE_DT)
    ax, ay, az, gx, gy, gz, temp = mean
    print(f"  采样 {N_SAMPLES} 组, 失败 {fails}", flush=True)
    print(f"  加计(g): X={ax:+.4f} Y={ay:+.4f} Z={az:+.4f}  |a|={math.sqrt(ax*ax+ay*ay+az*az):.4f}", flush=True)
    print(f"  陀螺(dps): X={gx:+.4f} Y={gy:+.4f} Z={gz:+.4f}", flush=True)
    roll, pitch = roll_pitch(ax, ay, az)
    print(f"  安装角: roll={roll:+.3f}°  pitch={pitch:+.3f}°", flush=True)

    calib = {"gyro": {"bias": [round(gx, 5), round(gy, 5), round(gz, 5)], "unit": "dps"},
             "mounting": {"roll_deg": round(roll, 3), "pitch_deg": round(pitch, 3)}}
    _merge_save(calib)
    return calib


def do_collect(imu, key):
    if key not in FACES:
        print(f"未知姿态 key '{key}'，可选: {', '.join(FACES)}", flush=True)
        sys.exit(2)
    print(f"\n=== 采集姿态 [{key}] : {FACES[key]} ===", flush=True)
    print("  摆好后保持完全静止，别碰它。", flush=True)
    countdown(COUNTDOWN)
    mean, std, fails = collect(imu, N_SAMPLES, SAMPLE_DT)
    ax, ay, az = mean[0], mean[1], mean[2]
    nrm = math.sqrt(ax * ax + ay * ay + az * az)
    dom_i, dom_s, dom_v = dominant_axis(mean)
    print(f"  加计均值(g): X={ax:+.4f} Y={ay:+.4f} Z={az:+.4f}  |a|={nrm:.4f}  失败{fails}", flush=True)
    print(f"  -> 主重力在 {AXIS_NAMES[dom_i]}{dom_s} 轴 ({dom_v:+.3f}g)", flush=True)

    # 校验：|a| 应≈1g，且主轴分量应占主导(>0.9g)
    ok = 0.9 < nrm < 1.1 and abs(dom_v) > 0.9
    print(f"  -> 姿态合理性: {'OK' if ok else '异常(检查是否摆到位/是否静止)'}", flush=True)

    faces = {}
    if os.path.exists(FACES_PATH):
        try:
            faces = json.load(open(FACES_PATH))
        except Exception:
            faces = {}
    faces[key] = {"acc": [round(ax, 6), round(ay, 6), round(az, 6)],
                  "dom": f"{AXIS_NAMES[dom_i]}{dom_s}"}
    json.dump(faces, open(FACES_PATH, "w"), indent=2, ensure_ascii=False)
    print(f"  已记录到 {FACES_PATH}（已采 {len(faces)} 个姿态）", flush=True)
    print(f"  当前各姿态主轴: " + "  ".join(f"{k}:{v['dom']}" for k, v in faces.items()), flush=True)


def do_solve():
    if not os.path.exists(FACES_PATH):
        print("还没有姿态数据，先 collect", flush=True)
        sys.exit(1)
    faces = json.load(open(FACES_PATH))
    print(f"已采姿态: {list(faces.keys())}", flush=True)
    print("各姿态主轴: " + "  ".join(f"{k}:{v['dom']}" for k, v in faces.items()), flush=True)

    accs = {k: v["acc"] for k, v in faces.items()}
    bias = [0.0, 0.0, 0.0]
    scale = [0.0, 0.0, 0.0]
    coverage_ok = True
    for i, axname in enumerate("XYZ"):
        vals = [a[i] for a in accs.values()]
        hi, lo = max(vals), min(vals)
        scale[i] = (hi - lo) / 2.0
        bias[i] = (hi + lo) / 2.0
        # 覆盖校验：该轴应既有接近 +1g 也有接近 -1g 的面
        if not (hi > 0.9 and lo < -0.9):
            coverage_ok = False
            print(f"  [警告] {axname} 轴覆盖不全: 最高 {hi:+.3f}g / 最低 {lo:+.3f}g "
                  f"(需要 ±1g 两面都采到)", flush=True)

    print("\n=== 六面解算（加速度计）===", flush=True)
    print(f"  零偏 bias (g):  X={bias[0]:+.5f}  Y={bias[1]:+.5f}  Z={bias[2]:+.5f}", flush=True)
    print(f"  标度 scale   :  X={scale[0]:.5f}  Y={scale[1]:.5f}  Z={scale[2]:.5f}  (应≈1.0)", flush=True)
    print("  校验(标定后 |a| 应≈1.0):", flush=True)
    for k, a in accs.items():
        a_cal = [(a[i] - bias[i]) / scale[i] for i in range(3)]
        nrm = math.sqrt(sum(v * v for v in a_cal))
        print(f"    {k:8s}: |a|={nrm:.4f}", flush=True)

    if not coverage_ok:
        print("\n  [注意] 有轴覆盖不全，结果可能不准；建议补齐该轴的正/反面再 solve。", flush=True)

    calib = {"accel": {"bias": [round(b, 6) for b in bias],
                       "scale": [round(s, 6) for s in scale],
                       "unit": "g"}}
    _merge_save(calib)
    return calib


def _merge_save(new_part):
    """把 new_part 合并进 calib.json（保留已存的 gyro/mounting 等）。"""
    calib = {}
    if os.path.exists(CALIB_PATH):
        try:
            calib = json.load(open(CALIB_PATH))
        except Exception:
            calib = {}
    calib.update(new_part)
    calib["note"] = f"ICM-42688-P 静态校准, {time.strftime('%Y-%m-%d %H:%M:%S')}"
    json.dump(calib, open(CALIB_PATH, "w"), indent=2, ensure_ascii=False)
    print(f"\n已保存到 {CALIB_PATH}", flush=True)
    print(json.dumps(calib, indent=2, ensure_ascii=False), flush=True)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "level"
    load_overlay()
    imu = make_imu()
    if mode == "level":
        do_level(imu)
    elif mode == "collect":
        key = sys.argv[2] if len(sys.argv) > 2 else "flat"
        do_collect(imu, key)
    elif mode == "solve":
        do_solve()
    else:
        print("用法: imu_static_calib.py [level|collect <key>|solve]", flush=True)
        sys.exit(2)


if __name__ == "__main__":
    main()
