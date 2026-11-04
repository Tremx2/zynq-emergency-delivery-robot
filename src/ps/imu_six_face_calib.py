#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ICM-42688-P 六面法静态标定（会话版，数据全保留）

每次标定建一个会话目录，所有原始采样 / 每面统计 / 解算结果都存在里面，互不覆盖：
  /home/xilinx/jupyter_notebooks/imu_calib_data/session_YYYYmmdd_HHMMSS/
      face_<key>_raw.csv      该面全部原始采样 (t, ax, ay, az [g], gx, gy, gz [dps], temp [°C])
      face_<key>.json         该面统计: 均值/标准差/采样数/失败数/主轴/静止检查/时间
      calib_result.json       解算结果 (六面法 + 12 参数最小二乘 + 陀螺零偏统计)
      report.txt              可读报告
      imu_calib_backup.json   写入前 imu_calib.json 的备份

用法（板上, root + venv）:
  imu_six_face_calib.py new                 新建会话（之后 collect 都进这个会话）
  imu_six_face_calib.py collect <key>       采一个面 (key: flat flipped left right front rear)
  imu_six_face_calib.py status              查看当前会话已采的面
  imu_six_face_calib.py solve               解算并写入 imu_calib.json 的 accel 部分

6 个摆放（以机器人车体为准）:
  flat     放平（四轮着地）
  flipped  倒扣（四轮朝天）
  left     左侧朝上（右侧着地）
  right    右侧朝上（左侧着地）
  front    车头朝上（车尾着地竖立）
  rear     车尾朝上（车头着地竖立）
"""
import os
import sys
import time
import json
import math
import shutil
import statistics

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

NB_DIR = "/home/xilinx/jupyter_notebooks"
BIT_PATH = os.path.join(NB_DIR, "robot_system_wrapper.bit")
DATA_ROOT = os.path.join(NB_DIR, "imu_calib_data")
CURRENT = os.path.join(DATA_ROOT, "CURRENT_SESSION")
CALIB_PATH = os.path.join(NB_DIR, "imu_calib.json")

N_SAMPLES = 1000          # 每面采样数
SAMPLE_DT = 0.01          # 100 Hz
SETTLE_S = 3              # 开始采集前等待（手离开、车体停稳）
STILL_ACC_STD_G = 0.005   # 静止判据: 加计各轴标准差 < 5 mg
STILL_GYRO_STD_DPS = 0.5  # 静止判据: 陀螺各轴标准差 < 0.5 dps

FACES = {
    "flat":    "放平（四轮着地）",
    "flipped": "倒扣（四轮朝天）",
    "left":    "左侧朝上（右侧着地）",
    "right":   "右侧朝上（左侧着地）",
    "front":   "车头朝上（车尾着地竖立）",
    "rear":    "车尾朝上（车头着地竖立）",
}
AX = "XYZ"


# ------------------------------------------------------------------ 会话
def session_dir(create=False):
    if create:
        d = os.path.join(DATA_ROOT, "session_" + time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(d, exist_ok=True)
        with open(CURRENT, "w") as f:
            f.write(d)
        return d
    if not os.path.exists(CURRENT):
        sys.exit("还没有会话，先运行: imu_six_face_calib.py new")
    d = open(CURRENT).read().strip()
    if not os.path.isdir(d):
        sys.exit(f"会话目录不存在: {d}")
    return d


def load_faces(d):
    faces = {}
    for k in FACES:
        p = os.path.join(d, f"face_{k}.json")
        if os.path.exists(p):
            faces[k] = json.load(open(p, encoding="utf-8"))
    return faces


def fsync_write(path, text):
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())


# ------------------------------------------------------------------ 硬件
def make_imu():
    from pynq import Overlay, PL
    from test_sensors import AxiIic, ICM42688
    if PL.bitfile_name and os.path.basename(PL.bitfile_name) == os.path.basename(BIT_PATH):
        Overlay(BIT_PATH, download=False)
        print("overlay 已加载，复用（不重新下载 PL）", flush=True)
    else:
        Overlay(BIT_PATH)
        print("overlay 已加载", flush=True)
    imu = ICM42688(AxiIic(), addr=0x69)
    who = imu.who_am_i()
    if who != 0x47:
        sys.exit(f"IMU WHO_AM_I=0x{who:02X}，不是 0x47")
    if not imu.init():
        sys.exit("IMU 初始化失败")
    print("IMU ICM-42688-P @0x69 OK（±4g, ±500dps, 100Hz, 低噪声）", flush=True)
    return imu


# ------------------------------------------------------------------ 采集
def do_collect(key):
    if key not in FACES:
        sys.exit(f"未知面 '{key}'，可选: {', '.join(FACES)}")
    d = session_dir()
    from test_sensors import I2CError
    imu = make_imu()
    print(f"\n=== 采集 [{key}] {FACES[key]} ===", flush=True)
    print(f"  {SETTLE_S}s 后开始，期间及采集中请勿触碰车体 ...", flush=True)
    time.sleep(SETTLE_S)
    for _ in range(20):          # 丢弃前几帧
        try:
            imu.read()
        except I2CError:
            pass
        time.sleep(SAMPLE_DT)
    rows, fails = [], 0
    t0 = time.monotonic()
    nxt = t0
    while len(rows) < N_SAMPLES:
        nxt += SAMPLE_DT
        try:
            rows.append((time.monotonic() - t0, *imu.read()))
        except I2CError:
            fails += 1
            if fails > N_SAMPLES // 5:
                sys.exit(f"I2C 失败过多 ({fails})，放弃本面")
        time.sleep(max(0.0, nxt - time.monotonic()))
    dur = time.monotonic() - t0

    raw_path = os.path.join(d, f"face_{key}_raw.csv")
    lines = ["t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c"]
    lines += [",".join(f"{v:.6f}" for v in r) for r in rows]
    fsync_write(raw_path, "\n".join(lines) + "\n")

    cols = list(zip(*rows))[1:]
    mean = [statistics.fmean(c) for c in cols]
    std = [statistics.pstdev(c) for c in cols]
    acc, gyr = mean[0:3], mean[3:6]
    nrm = math.sqrt(sum(a * a for a in acc))
    dom = max(range(3), key=lambda i: abs(acc[i]))
    dom_s = "+" if acc[dom] >= 0 else "-"
    still = max(std[0:3]) < STILL_ACC_STD_G and max(std[3:6]) < STILL_GYRO_STD_DPS
    posture = 0.9 < nrm < 1.1 and abs(acc[dom]) > 0.9
    tilt = math.degrees(math.acos(min(1.0, abs(acc[dom]) / nrm)))

    rec = dict(face=key, desc=FACES[key], time=time.strftime("%Y-%m-%d %H:%M:%S"),
               n=len(rows), fails=fails, duration_s=round(dur, 3), rate_hz=round(len(rows) / dur, 2),
               acc_mean_g=[round(v, 6) for v in acc], acc_std_g=[round(v, 6) for v in std[0:3]],
               gyro_mean_dps=[round(v, 5) for v in gyr], gyro_std_dps=[round(v, 5) for v in std[3:6]],
               temp_c=round(mean[6], 2), norm_g=round(nrm, 6), dominant=f"{AX[dom]}{dom_s}",
               tilt_from_axis_deg=round(tilt, 3), still_ok=still, posture_ok=posture,
               raw_csv=os.path.basename(raw_path))
    fsync_write(os.path.join(d, f"face_{key}.json"), json.dumps(rec, indent=2, ensure_ascii=False))

    print(f"  采样 {len(rows)} 组 / 失败 {fails} / {rec['rate_hz']} Hz / 温度 {rec['temp_c']}°C", flush=True)
    print(f"  加计均值 (g):  X={acc[0]:+.5f}  Y={acc[1]:+.5f}  Z={acc[2]:+.5f}   |a|={nrm:.5f}", flush=True)
    print(f"  加计 σ (mg):   X={std[0]*1e3:.2f}  Y={std[1]*1e3:.2f}  Z={std[2]*1e3:.2f}", flush=True)
    print(f"  陀螺均值 (dps): X={gyr[0]:+.4f}  Y={gyr[1]:+.4f}  Z={gyr[2]:+.4f}", flush=True)
    print(f"  主重力轴 {AX[dom]}{dom_s}，偏离该轴 {tilt:.2f}°", flush=True)
    print(f"  静止检查: {'OK' if still else '不通过（采集中有晃动，建议重采）'}", flush=True)
    print(f"  姿态检查: {'OK' if posture else '不通过（未摆到位，建议重采）'}", flush=True)
    others = {k: v["dominant"] for k, v in load_faces(d).items() if k != key}
    dup = [k for k, v in others.items() if v == rec["dominant"]]
    if dup:
        print(f"  [警告] 主轴 {rec['dominant']} 与已采的 {dup} 相同 —— 可能摆错了面", flush=True)
    print(f"  已保存: {raw_path}", flush=True)
    do_status(quiet_header=True)


def do_status(quiet_header=False):
    d = session_dir()
    faces = load_faces(d)
    if not quiet_header:
        print("会话:", d)
    print(f"  已采 {len(faces)}/6: " + "  ".join(
        f"{k}:{v['dominant']}{'' if v['still_ok'] and v['posture_ok'] else '(!)'}" for k, v in faces.items()), flush=True)
    todo = [k for k in FACES if k not in faces]
    if todo:
        print("  未采: " + ", ".join(f"{k}({FACES[k]})" for k in todo), flush=True)


# ------------------------------------------------------------------ 解算
def solve3(A, b):
    """解 4 元线性最小二乘 (正规方程 + 高斯消元)，A: m×4, b: m"""
    n = len(A[0])
    M = [[sum(A[r][i] * A[r][j] for r in range(len(A))) for j in range(n)] for i in range(n)]
    v = [sum(A[r][i] * b[r] for r in range(len(A))) for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        v[c], v[p] = v[p], v[c]
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [M[r][k] - f * M[c][k] for k in range(n)]
                v[r] -= f * v[c]
    return [v[i] / M[i][i] for i in range(n)]


def inv3(m):
    a, b, c = m[0]; d, e, f = m[1]; g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    return [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]


def do_solve():
    d = session_dir()
    faces = load_faces(d)
    out = []
    def p(s=""):
        print(s, flush=True)
        out.append(s)

    p(f"会话: {d}")
    p(f"已采 {len(faces)}/6: " + "  ".join(f"{k}:{v['dominant']}" for k, v in faces.items()))
    doms = sorted(v["dominant"] for v in faces.values())
    if len(faces) < 6 or doms != sorted(["X+", "X-", "Y+", "Y-", "Z+", "Z-"]):
        p("[错误] 六个面的主重力轴必须恰好覆盖 X± Y± Z± 各一次，当前: " + ", ".join(doms))
        sys.exit(1)
    bad = [k for k, v in faces.items() if not (v["still_ok"] and v["posture_ok"])]
    if bad:
        p(f"[警告] 以下面静止/姿态检查未通过: {bad}（结果仍会计算，建议重采）")

    acc = {k: v["acc_mean_g"] for k, v in faces.items()}
    gref = {}
    for k, v in faces.items():
        g = [0.0, 0.0, 0.0]
        g[AX.index(v["dominant"][0])] = 1.0 if v["dominant"][1] == "+" else -1.0
        gref[k] = g

    # 方法 A: 经典六面法（每轴 ±1g 两面）
    by_dom = {v["dominant"]: acc[k] for k, v in faces.items()}
    biasA, scaleA = [], []
    for i, a in enumerate(AX):
        up, dn = by_dom[a + "+"][i], by_dom[a + "-"][i]
        biasA.append((up + dn) / 2)
        scaleA.append((up - dn) / 2)

    # 方法 B: 12 参数最小二乘  a_raw = K·g_ref + b   →  a_cal = K⁻¹(a_raw − b)
    keys = list(faces)
    A = [[*gref[k], 1.0] for k in keys]
    K, biasB = [], []
    for i in range(3):
        sol = solve3(A, [acc[k][i] for k in keys])
        K.append(sol[0:3])
        biasB.append(sol[3])
    Kinv = inv3(K)

    def calA(a):
        return [(a[i] - biasA[i]) / scaleA[i] for i in range(3)]
    def calB(a):
        c = [a[i] - biasB[i] for i in range(3)]
        return [sum(Kinv[i][j] * c[j] for j in range(3)) for i in range(3)]
    def nrm(v):
        return math.sqrt(sum(x * x for x in v))

    p("\n=== 方法 A：经典六面法 ===")
    p(f"  零偏 (mg):  X={biasA[0]*1e3:+8.3f}  Y={biasA[1]*1e3:+8.3f}  Z={biasA[2]*1e3:+8.3f}")
    p(f"  标度:       X={scaleA[0]:.5f}  Y={scaleA[1]:.5f}  Z={scaleA[2]:.5f}"
      f"   （标度误差 {', '.join(f'{(s-1)*100:+.2f}%' for s in scaleA)}）")
    p("\n=== 方法 B：12 参数最小二乘（含轴间不正交）===")
    p(f"  零偏 (mg):  X={biasB[0]*1e3:+8.3f}  Y={biasB[1]*1e3:+8.3f}  Z={biasB[2]*1e3:+8.3f}")
    p("  K 矩阵（对角=标度，非对角=交叉耦合）:")
    for i in range(3):
        p("    [" + "  ".join(f"{K[i][j]:+.5f}" for j in range(3)) + "]")
    cross = max(abs(K[i][j]) for i in range(3) for j in range(3) if i != j)
    p(f"  最大交叉耦合 {cross*1e3:.2f} mg/g（≈ {math.degrees(math.asin(min(1, cross))):.2f}° 轴偏）")

    p("\n=== 标定效果：各面 |a| 与 1g 的偏差 (mg) ===")
    p(f"  {'面':<8}{'主轴':<6}{'原始':>10}{'方法A':>10}{'方法B':>10}")
    errs = {"raw": [], "A": [], "B": []}
    for k in keys:
        e0 = (nrm(acc[k]) - 1) * 1e3
        eA = (nrm(calA(acc[k])) - 1) * 1e3
        eB = (nrm(calB(acc[k])) - 1) * 1e3
        errs["raw"].append(abs(e0)); errs["A"].append(abs(eA)); errs["B"].append(abs(eB))
        p(f"  {k:<8}{faces[k]['dominant']:<6}{e0:>+10.2f}{eA:>+10.2f}{eB:>+10.2f}")
    rms = {m: math.sqrt(sum(x * x for x in v) / len(v)) for m, v in errs.items()}
    p(f"  {'RMS':<14}{rms['raw']:>10.2f}{rms['A']:>10.2f}{rms['B']:>10.2f}")

    gyr = [faces[k]["gyro_mean_dps"] for k in keys]
    gmean = [statistics.fmean(g[i] for g in gyr) for i in range(3)]
    gspread = [max(g[i] for g in gyr) - min(g[i] for g in gyr) for i in range(3)]
    noise = [statistics.fmean(faces[k]["acc_std_g"][i] for k in keys) * 1e3 for i in range(3)]
    gnoise = [statistics.fmean(faces[k]["gyro_std_dps"][i] for k in keys) for i in range(3)]
    temps = [faces[k]["temp_c"] for k in keys]
    p("\n=== 陀螺零偏（6 面静止数据）===")
    p(f"  均值 (dps):     X={gmean[0]:+.4f}  Y={gmean[1]:+.4f}  Z={gmean[2]:+.4f}")
    p(f"  6 面极差 (dps): X={gspread[0]:.4f}  Y={gspread[1]:.4f}  Z={gspread[2]:.4f}（零偏稳定性 / g 敏感度）")
    p("\n=== 噪声与环境 ===")
    p(f"  加计噪声 σ (mg):   X={noise[0]:.3f}  Y={noise[1]:.3f}  Z={noise[2]:.3f}")
    p(f"  陀螺噪声 σ (dps):  X={gnoise[0]:.4f}  Y={gnoise[1]:.4f}  Z={gnoise[2]:.4f}")
    p(f"  温度 {min(temps):.1f} ~ {max(temps):.1f} °C")

    result = dict(
        time=time.strftime("%Y-%m-%d %H:%M:%S"), session=d, faces={k: faces[k]["dominant"] for k in keys},
        method_A_six_position=dict(bias_g=biasA, scale=scaleA),
        method_B_least_squares=dict(bias_g=biasB, K=K, K_inv=Kinv, max_cross_coupling=cross,
                                    model="a_cal = K_inv · (a_raw - bias)"),
        norm_error_rms_mg=rms,
        gyro_bias_dps=dict(mean=gmean, spread_6faces=gspread),
        noise=dict(acc_std_mg=noise, gyro_std_dps=gnoise), temp_c=[min(temps), max(temps)],
        warnings=dict(faces_failed_checks=bad))
    fsync_write(os.path.join(d, "calib_result.json"), json.dumps(result, indent=2, ensure_ascii=False))

    # 主结果选 |a| 残差 RMS 更小的方法。手工摆放有 1~4° 倾斜时，6 面 12 参数 LS 会把摆放倾斜
    # 误当成轴间耦合（过拟合），通常经典六面法更可靠。
    primary = "A" if rms["A"] <= rms["B"] else "B"
    result["primary_method"] = primary
    fsync_write(os.path.join(d, "calib_result.json"), json.dumps(result, indent=2, ensure_ascii=False))
    p("")
    p(f"主结果采用方法 {primary}（RMS {rms[primary]:.2f} mg，另一方法 {rms['B' if primary == 'A' else 'A']:.2f} mg）")
    if primary == "A":
        p("  说明：方法 B 的非对角项主要来自手工摆放倾斜，不代表真实轴间耦合，仅作参考保留。")

    # 合并写入 imu_calib.json（首次解算前备份一次；保留 gyro / mounting）
    calib = {}
    if os.path.exists(CALIB_PATH):
        bk = os.path.join(d, "imu_calib_backup.json")
        if not os.path.exists(bk):
            shutil.copy(CALIB_PATH, bk)
        try:
            calib = json.load(open(CALIB_PATH, encoding="utf-8"))
        except Exception:
            calib = {}
    if primary == "A":
        acc_part = {"bias": [round(v, 6) for v in biasA], "scale": [round(v, 6) for v in scaleA],
                    "model": "a_cal[i] = (a_raw[i] - bias[i]) / scale[i]  (six-position)"}
    else:
        acc_part = {"bias": [round(v, 6) for v in biasB], "scale": [round(K[i][i], 6) for i in range(3)],
                    "K_inv": [[round(v, 7) for v in r] for r in Kinv],
                    "model": "a_cal = K_inv · (a_raw - bias)  (six-face 12-param LS)"}
    calib["accel"] = dict(acc_part, unit="g", method=primary, norm_error_rms_mg=round(rms[primary], 3),
                          reference_least_squares={"bias": [round(v, 6) for v in biasB],
                                                   "K_inv": [[round(v, 7) for v in r] for r in Kinv],
                                                   "norm_error_rms_mg": round(rms["B"], 3)},
                          reference_six_position={"bias": [round(v, 6) for v in biasA], "scale": [round(v, 6) for v in scaleA],
                                                  "norm_error_rms_mg": round(rms["A"], 3)},
                          session=os.path.basename(d))
    calib["gyro_six_face"] = {"bias_mean": [round(v, 5) for v in gmean], "spread": [round(v, 5) for v in gspread], "unit": "dps"}
    calib["note"] = f"ICM-42688-P 静态标定（六面法），{time.strftime('%Y-%m-%d %H:%M:%S')}"
    fsync_write(CALIB_PATH, json.dumps(calib, indent=2, ensure_ascii=False))
    p(f"\n结果已保存: {os.path.join(d, 'calib_result.json')}")
    p(f"已更新: {CALIB_PATH}（旧版备份在会话目录 imu_calib_backup.json）")
    fsync_write(os.path.join(d, "report.txt"), "\n".join(out) + "\n")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "status"
    if mode == "new":
        d = session_dir(create=True)
        print("新会话:", d)
    elif mode == "collect":
        do_collect(sys.argv[2] if len(sys.argv) > 2 else "")
    elif mode == "status":
        do_status()
    elif mode == "solve":
        do_solve()
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
