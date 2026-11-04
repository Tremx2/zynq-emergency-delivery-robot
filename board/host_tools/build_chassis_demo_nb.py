# Generates chassis_motion_demo.ipynb (Jupyter notebook for the PYNQ board).
import json, sys

cells = []
def md(src): cells.append({"cell_type": "markdown", "metadata": {}, "source": src.strip("\n")})
def code(src): cells.append({"cell_type": "code", "metadata": {}, "execution_count": None, "outputs": [], "source": src.strip("\n")})

md(r"""
# 底盘六动作演示与数据采集（小场地版）

依次执行 **直行前进 → 直行后退 → 前进左弧线 → 前进右弧线 → 原地左转 → 原地右转**，全程 50 Hz 记录四轮
目标速度 / 实测速度 / PID-PWM 指令 / 编码器位置、看门狗状态，以及 IMU 陀螺 Z 轴（航向角速度），最后输出：

1. 每个动作的数据汇总表（距离、转角、速度跟踪误差、响应时间、制动滑行、弧线半径……）
2. 图表：速度跟踪 / PWM / 轨迹（编码器 vs IMU 航向）/ 航向角曲线 / 跟踪误差对比
3. 保存全部原始数据 CSV、汇总 JSON、图片 PNG 到 `chassis_demo_results/run_时间戳/`

### 场地与安全
- 默认参数需要约 **1.2 m（前后）× 1.0 m（左右）** 空地；车放在场地后部中央、车头朝前。场地不够就把下面的 `DIST_STRAIGHT_MM` / `DIST_ARC_MM` 改小。
- 车会按编码器距离/角度**自动停车**；每段还有超时保护。
- **急停**：点工具栏 ■（Interrupt）。程序捕获后立即 `stop()`；即使 Python 卡死，底盘 1 s 指令租约 + FPGA 500 ms 硬件看门狗也会切断动力。
- 上电顺序：先 PYNQ 后 11.1 V 电机电源。**PYNQ 供电要稳定**（不稳会在运行中掉电重启）。

### 运行方法
从上到下逐个运行（Shift+Enter）。第 3 个代码单元会让车真正动起来。
""")

code(r"""
# ===== 1. 参数（按场地大小修改） =====
SPEED_STRAIGHT   = 30      # 直行速度 target (counts/10ms)，30 ≈ 0.17 m/s
ARC_OUTER        = 35      # 弧线外侧轮速度
ARC_INNER        = 20      # 弧线内侧轮速度
SPEED_ROTATE     = 30      # 原地旋转速度（与旋转标定一致，勿改）

DIST_STRAIGHT_MM = 500     # 直行前进 / 后退距离 (mm)
DIST_ARC_MM      = 400     # 左 / 右弧线：车体中心走过的路程 (mm)
ANGLE_ROTATE_DEG = 90      # 原地左 / 右转角度 (deg)

SEG_TIMEOUT_S    = 20      # 单个动作超时 (s)
PAUSE_S          = 1.5     # 动作之间静止间隔 (s)
SETTLE_S         = 0.8     # 停车后继续记录的时间 (s)，用于测制动滑行
SAMPLE_DT        = 0.02    # 采样周期 (s) = 50 Hz

USE_IMU          = True    # 记录 IMU 陀螺 Z 轴；读不到会自动关闭，不影响运动
IMU_YAW_SIGN     = -1      # IMU 倒装 (roll≈-179°)：车体左转为正 = -gz
IMU_BIAS_S       = 1.5     # 开始前静止估计陀螺零偏的时长 (s)

# (key, 中文名, 图中标签, 左侧速度, 右侧速度, 目标类型, 目标值)
SEQUENCE = [
    ("forward",      "直行前进", "Forward",   +SPEED_STRAIGHT, +SPEED_STRAIGHT, "dist",  DIST_STRAIGHT_MM),
    ("backward",     "直行后退", "Backward",  -SPEED_STRAIGHT, -SPEED_STRAIGHT, "dist",  DIST_STRAIGHT_MM),
    ("arc_left",     "左弧线",   "Arc left",  +ARC_INNER,      +ARC_OUTER,      "dist",  DIST_ARC_MM),
    ("arc_right",    "右弧线",   "Arc right", +ARC_OUTER,      +ARC_INNER,      "dist",  DIST_ARC_MM),
    ("rotate_left",  "原地左转", "Spin left", -SPEED_ROTATE,   +SPEED_ROTATE,   "angle", ANGLE_ROTATE_DEG),
    ("rotate_right", "原地右转", "Spin right",+SPEED_ROTATE,   -SPEED_ROTATE,   "angle", ANGLE_ROTATE_DEG),
]
print("动作序列：", " → ".join(s[1] for s in SEQUENCE))
""")

code(r"""
# ===== 2. 加载 overlay、初始化底盘与 IMU（不会让车动） =====
import os, sys, time, math, json, csv
os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"
NB_DIR = "/home/xilinx/jupyter_notebooks"
if NB_DIR not in sys.path:
    sys.path.insert(0, NB_DIR)

import numpy as np
from pynq import Overlay, MMIO
import robot_chassis as rc
from robot_chassis import Chassis, delta32

CPM        = rc.GROUND_COUNTS_PER_MM          # counts / mm（直线落地标定）
LEFT_CPD   = rc.LEFT_COUNTS_PER_DEG           # 原地左转 counts / deg
RIGHT_CPD  = rc.RIGHT_COUNTS_PER_DEG          # 原地右转 counts / deg
TRACK_MM   = rc.TRACK_WIDTH_MM
WHEELS     = ["M1 右前", "M2 右后", "M3 左后", "M4 左前"]
WHEEL_SIGN = np.array([-1, -1, +1, +1])       # 寄存器值 × sign = 车辆前进方向为正

print("加载 overlay ...")
overlay = Overlay(rc.DEFAULT_OVERLAY_PATH)
robot = Chassis(MMIO(0x43C00000, 0x1000), overlay=overlay)   # 停车 + 布防 + 喂狗 + 上电自检
armed, fault = robot.watchdog_state()
rep = robot.self_test()
print(f"看门狗 armed={armed} fault={fault} | 上电自检: {'PASS' if rep['safe'] else 'FAIL ' + str(rep['problems'])}")
print(f"标定: {CPM:.3f} counts/mm | 左转 {LEFT_CPD:.2f} / 右转 {RIGHT_CPD:.2f} counts/deg | 轮距 {TRACK_MM} mm")

imu = None
if USE_IMU:
    try:
        from test_sensors import AxiIic, ICM42688
        _bus = AxiIic()
        _imu = ICM42688(_bus, addr=0x69)
        assert _imu.who_am_i() == 0x47, "WHO_AM_I 不对"
        assert _imu.init(), "IMU 初始化失败"
        _imu.read()
        imu = _imu
        print("IMU ICM-42688-P @0x69 就绪")
    except Exception as e:
        print("IMU 不可用，本次只用编码器：", repr(e))
""")

md(r"""
### 3. 运行六个动作（车会动！确认场地清空、人手离开车轮）
""")

code(r"""
# ===== 3. 执行全部动作并记录 =====
COLS = (["t", "seg", "moving"] + [f"tgt{i+1}" for i in range(4)] + [f"spd{i+1}" for i in range(4)]
        + [f"pos{i+1}" for i in range(4)] + [f"pwm{i+1}" for i in range(4)]
        + ["armed", "fault", "gz_dps", "x_mm", "y_mm", "th_enc_deg", "x_imu_mm", "y_imu_mm", "th_imu_deg"])
rows = []
seg_meta = []

def read_imu_gz():
    if imu is None:
        return float("nan")
    try:
        return imu.read()[5]
    except Exception:
        return float("nan")

# --- 陀螺零偏（静止） ---
gz_bias = 0.0
if imu is not None:
    vals = []
    t_end = time.monotonic() + IMU_BIAS_S
    while time.monotonic() < t_end:
        v = read_imu_gz()
        if not math.isnan(v):
            vals.append(v)
        time.sleep(0.01)
    gz_bias = float(np.mean(vals)) if vals else 0.0
    print(f"陀螺 Z 零偏 = {gz_bias:+.3f} dps（{len(vals)} 个样本）")

state = dict(x=0.0, y=0.0, th=0.0, xi=0.0, yi=0.0, thi=0.0, last_pos=None, last_t=None)
T0 = time.monotonic()

def sample(seg_idx, moving, targets):
    # 读一次全部寄存器 + IMU，更新两套航迹推算，追加一行
    now = time.monotonic()
    pos = robot.get_positions()
    spd = robot.get_speeds()
    pwm = robot.get_pwms()
    armed, fault = robot.watchdog_state()
    gz = read_imu_gz()
    if state["last_pos"] is not None:
        d = [delta32(pos[i], state["last_pos"][i]) for i in range(4)]
        dL = (d[2] + d[3]) / 2.0
        dR = -(d[0] + d[1]) / 2.0
        ds = (dL + dR) / 2.0 / CPM                       # mm
        rot = (dR - dL) / 2.0
        dth = math.radians(rot / (LEFT_CPD if rot >= 0 else RIGHT_CPD))
        th_mid = state["th"] + dth / 2
        state["x"] += ds * math.cos(th_mid); state["y"] += ds * math.sin(th_mid); state["th"] += dth
        dt = now - state["last_t"]
        dthi = 0.0 if math.isnan(gz) else math.radians(IMU_YAW_SIGN * (gz - gz_bias) * dt)
        thi_mid = state["thi"] + dthi / 2
        state["xi"] += ds * math.cos(thi_mid); state["yi"] += ds * math.sin(thi_mid); state["thi"] += dthi
    state["last_pos"], state["last_t"] = pos, now
    rows.append([now - T0, seg_idx, int(moving), *targets, *spd, *pos, *pwm, int(armed), int(fault), gz,
                 state["x"], state["y"], math.degrees(state["th"]),
                 state["xi"], state["yi"], math.degrees(state["thi"])])
    if fault or not armed:
        raise rc.WatchdogFault(f"看门狗异常 armed={armed} fault={fault}")
    return pos

def log_idle(seg_idx, seconds):
    t_end = time.monotonic() + seconds
    nxt = time.monotonic()
    while time.monotonic() < t_end:
        sample(seg_idx, False, [0, 0, 0, 0])
        nxt += SAMPLE_DT
        time.sleep(max(0.0, nxt - time.monotonic()))

def run_segment(seg_idx, key, name, label, left, right, goal_type, goal):
    targets = [-right, -right, left, left]               # 写入寄存器的 M1..M4 目标
    start_pos = robot.get_positions()
    def progress(pos):
        raw = [delta32(pos[i], start_pos[i]) for i in range(4)]
        dL = (raw[2] + raw[3]) / 2.0
        dR = -(raw[0] + raw[1]) / 2.0
        rot = (dR - dL) / 2.0
        center_mm = (dL + dR) / 2.0 / CPM
        ang = rot / (LEFT_CPD if rot >= 0 else RIGHT_CPD)
        return center_mm, ang
    t_cmd = time.monotonic()
    robot.drive(left, right)
    reason = "timeout"
    at_stop = None
    try:
        nxt = time.monotonic()
        while True:
            nxt += SAMPLE_DT
            time.sleep(max(0.0, nxt - time.monotonic()))
            robot.keepalive()
            pos = sample(seg_idx, True, targets)
            center_mm, ang = progress(pos)
            done = abs(center_mm) >= goal if goal_type == "dist" else abs(ang) >= goal
            if done:
                reason = "reached"
                break
            if time.monotonic() - t_cmd > SEG_TIMEOUT_S:
                break
    finally:
        robot.stop()
    t_stop = time.monotonic()
    at_stop = progress(robot.get_positions())
    log_idle(seg_idx, SETTLE_S)
    final = progress(robot.get_positions())
    seg_meta.append(dict(idx=seg_idx, key=key, name=name, label=label, left=left, right=right,
                         goal_type=goal_type, goal=goal, reason=reason,
                         t_cmd=t_cmd - T0, t_stop=t_stop - T0, t_end=time.monotonic() - T0,
                         center_at_stop_mm=at_stop[0], ang_at_stop_deg=at_stop[1],
                         center_mm=final[0], ang_deg=final[1],
                         start_pos=start_pos, end_pos=robot.get_positions()))
    print(f"  [{seg_idx+1}/{len(SEQUENCE)}] {name:<5} {reason:<7} 用时 {t_stop - t_cmd:5.2f}s | "
          f"中心路程 {final[0]:+7.1f} mm | 转角(编码器) {final[1]:+6.1f}°")

print("开始运行，急停请点 ■")
try:
    log_idle(-1, 0.5)
    for i, seg in enumerate(SEQUENCE):
        run_segment(i, *seg)
        if i < len(SEQUENCE) - 1:
            log_idle(-1, PAUSE_S)
    print("全部动作完成，车已停止。")
except KeyboardInterrupt:
    print("!! 手动急停")
except Exception as e:
    print("!! 运行异常，已停车：", repr(e))
finally:
    try:
        robot.stop()
    except Exception as e:
        print("stop() 异常：", e)

data = np.array(rows, dtype=float)
C = {c: i for i, c in enumerate(COLS)}
print(f"共记录 {len(data)} 个采样点，时长 {data[-1, C['t']] - data[0, C['t']]:.1f} s，"
      f"平均采样率 {len(data) / max(1e-9, data[-1, C['t']] - data[0, C['t']]):.1f} Hz")
""")

md(r"""
### 4. 数据汇总
""")

code(r"""
# ===== 4. 每个动作的指标 =====
from IPython.display import HTML, display
if not seg_meta:
    raise RuntimeError("没有完成任何动作，无数据可汇总（检查上一单元的报错）")

def to_mm_s(counts_per_10ms):
    return np.asarray(counts_per_10ms) * 100.0 / CPM

summary = []
for m in seg_meta:
    sel = (data[:, C["seg"]] == m["idx"])
    mov = sel & (data[:, C["moving"]] == 1)
    t = data[mov, C["t"]]
    tgt = np.array([-m["right"], -m["right"], m["left"], m["left"]], dtype=float)
    spd = data[mov][:, [C[f"spd{i+1}"] for i in range(4)]]
    pwm = data[mov][:, [C[f"pwm{i+1}"] for i in range(4)]]
    n = len(t)
    steady = spd[n // 2:] if n >= 4 else spd                     # 后半段视为稳态
    mean_ss = steady.mean(axis=0) if len(steady) else np.full(4, np.nan)
    std_ss = steady.std(axis=0) if len(steady) else np.full(4, np.nan)
    err_pct = np.abs(mean_ss - tgt) / np.abs(tgt) * 100.0
    # 响应时间：实测速度首次达到目标 90% 的时刻
    rise = []
    for i in range(4):
        hit = np.where(spd[:, i] * np.sign(tgt[i]) >= 0.9 * abs(tgt[i]))[0]
        rise.append(t[hit[0]] - m["t_cmd"] if len(hit) else np.nan)
    raw = np.array([delta32(m["end_pos"][i], m["start_pos"][i]) for i in range(4)], dtype=float)
    wheel_mm = raw * WHEEL_SIGN / CPM
    sel_all = data[sel]
    th_enc = sel_all[-1, C["th_enc_deg"]] - sel_all[0, C["th_enc_deg"]] if len(sel_all) else np.nan
    th_imu = sel_all[-1, C["th_imu_deg"]] - sel_all[0, C["th_imu_deg"]] if (len(sel_all) and imu is not None) else np.nan
    row = dict(
        动作=m["name"], 结果=("到达" if m["reason"] == "reached" else "超时"),
        目标=(f"{m['goal']} mm" if m["goal_type"] == "dist" else f"{m['goal']}°"),
        左右目标速度=f"{m['left']:+d} / {m['right']:+d}",
        运动用时_s=m["t_stop"] - m["t_cmd"],
        中心路程_mm=m["center_mm"],
        转角_编码器_deg=m["ang_deg"],
        转角_IMU_deg=th_imu,
        制动滑行=(f"{abs(m['center_mm'] - m['center_at_stop_mm']):.1f} mm" if m["goal_type"] == "dist"
                 else f"{abs(m['ang_deg'] - m['ang_at_stop_deg']):.2f}°"),
        平均车速_mm_s=float(np.mean(np.abs(to_mm_s(mean_ss)))),
        最大跟踪误差_pct=float(np.nanmax(err_pct)),
        平均跟踪误差_pct=float(np.nanmean(err_pct)),
        最慢响应_s=float(np.nanmax(rise)) if not np.all(np.isnan(rise)) else np.nan,
        最大PWM=int(np.abs(pwm).max()) if len(pwm) else 0,
    )
    if m["goal_type"] == "dist" and m["left"] != m["right"]:
        vl, vr = m["left"], m["right"]
        row["理论半径_mm"] = TRACK_MM / 2 * (vl + vr) / (vr - vl)
        row["实测半径_mm"] = m["center_mm"] / math.radians(m["ang_deg"]) if abs(m["ang_deg"]) > 1 else np.nan
    if m["goal_type"] == "angle":
        row["目标误差"] = f"{abs(m['ang_deg']) - m['goal']:+.2f}°"
    elif m["goal_type"] == "dist":
        row["目标误差"] = f"{abs(m['center_mm']) - m['goal']:+.1f} mm"
    row["_wheel"] = dict(target=tgt, mean=mean_ss, std=std_ss, err=err_pct, rise=np.array(rise), mm=wheel_mm)
    summary.append(row)

def fmt(v):
    if isinstance(v, float):
        return "—" if math.isnan(v) else f"{v:.1f}" if abs(v) >= 10 else f"{v:.2f}"
    return str(v)

keys = [k for k in summary[0].keys() if not k.startswith("_")]
for r in summary:
    for k in r:
        if not k.startswith("_") and k not in keys:
            keys.append(k)
css = ("<style>.rt td,.rt th{padding:4px 10px;border-bottom:1px solid #e3e2de;text-align:right;font-size:13px}"
       ".rt th{background:#f0efec;color:#0b0b0b}.rt td:first-child,.rt th:first-child{text-align:left}</style>")
html = css + "<table class='rt'><tr><th>指标</th>" + "".join(f"<th>{r['动作']}</th>" for r in summary) + "</tr>"
for k in keys[1:]:
    html += f"<tr><td>{k.replace('_', ' ')}</td>" + "".join(f"<td>{fmt(r.get(k, '—'))}</td>" for r in summary) + "</tr>"
html += "</table>"
display(HTML("<h4>表1 六个动作汇总</h4>" + html))

html2 = css + "<table class='rt'><tr><th>动作</th><th>车轮</th><th>目标</th><th>稳态均值</th><th>稳态波动σ</th><th>跟踪误差%</th><th>响应时间 s</th><th>轮行程 mm</th></tr>"
for r in summary:
    w = r["_wheel"]
    for i in range(4):
        html2 += (f"<tr><td>{r['动作'] if i == 0 else ''}</td><td>{WHEELS[i]}</td><td>{w['target'][i]:+.0f}</td>"
                  f"<td>{w['mean'][i]:+.1f}</td><td>{w['std'][i]:.2f}</td><td>{w['err'][i]:.1f}</td>"
                  f"<td>{fmt(float(w['rise'][i]))}</td><td>{w['mm'][i]:+.1f}</td></tr>")
html2 += "</table>"
display(HTML("<h4>表2 四轮闭环明细（速度单位 counts/10ms；寄存器原始符号）</h4>" + html2))

wd = data[:, C["fault"]].max() if len(data) else 0
print(f"全程看门狗：armed 始终为 1 = {bool(data[:, C['armed']].min())}，fault 出现 = {bool(wd)}")
print(f"终点位姿（编码器航迹）：x={data[-1, C['x_mm']]:+.0f} mm, y={data[-1, C['y_mm']]:+.0f} mm, "
      f"航向 {data[-1, C['th_enc_deg']]:+.1f}°")
if imu is not None:
    print(f"终点位姿（IMU 航向）：  x={data[-1, C['x_imu_mm']]:+.0f} mm, y={data[-1, C['y_imu_mm']]:+.0f} mm, "
          f"航向 {data[-1, C['th_imu_deg']]:+.1f}°")
""")

md(r"""
### 5. 图表
""")

code(r"""
# ===== 5. 作图 =====
%matplotlib inline
import matplotlib.pyplot as plt
from matplotlib import font_manager
for f in ["/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"]:
    if os.path.exists(f):
        font_manager.fontManager.addfont(f)
        plt.rcParams["font.sans-serif"] = [font_manager.FontProperties(fname=f).get_name(), "DejaVu Sans"]
plt.rcParams.update({
    "axes.unicode_minus": False, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
    "axes.edgecolor": "#b5b4ad", "axes.labelcolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e",
    "axes.grid": True, "grid.color": "#e3e2de", "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 2, "legend.frameon": False, "font.size": 10,
    "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlecolor": "#0b0b0b",
})
WHEEL_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]    # M1..M4（固定顺序）
ENC_COLOR, IMU_COLOR = "#2a78d6", "#eb6834"
TGT_COLOR = "#52514e"
figs = {}
t = data[:, C["t"]]

def shade_segments(ax, label=True):
    for k, m in enumerate(seg_meta):
        ax.axvspan(m["t_cmd"], m["t_stop"], color="#f0efec" if k % 2 == 0 else "#e8e7e3", zorder=0, lw=0)
        if label:
            ax.text((m["t_cmd"] + m["t_stop"]) / 2, 1.0, m["name"], transform=ax.get_xaxis_transform(),
                    ha="center", va="bottom", fontsize=9, color="#52514e")

# 图1：全程四轮速度（车辆前进方向为正）
fig, ax = plt.subplots(figsize=(13, 4.2))
shade_segments(ax)
for i in range(4):
    ax.plot(t, data[:, C[f"spd{i+1}"]] * WHEEL_SIGN[i], color=WHEEL_COLORS[i], label=WHEELS[i])
for i in range(4):
    ax.plot(t, data[:, C[f"tgt{i+1}"]] * WHEEL_SIGN[i], color=TGT_COLOR, lw=1, ls="--",
            label="目标速度" if i == 0 else None)
ax.set_xlabel("时间 (s)"); ax.set_ylabel("车轮速度 (counts/10ms，前进为正)")
ax.set_title("图1  六个动作全程四轮速度闭环跟踪", pad=18)
ax.legend(ncol=5, loc="lower center", bbox_to_anchor=(0.5, -0.32))
fig.tight_layout(); figs["fig1_speed_overview"] = fig; plt.show()

# 图2：全程 PID 输出 / PWM 指令
fig, ax = plt.subplots(figsize=(13, 3.6))
shade_segments(ax)
for i in range(4):
    ax.plot(t, data[:, C[f"pwm{i+1}"]] * WHEEL_SIGN[i], color=WHEEL_COLORS[i], label=WHEELS[i])
ax.axhline(6249, color="#b5b4ad", lw=1, ls=":"); ax.axhline(-6249, color="#b5b4ad", lw=1, ls=":")
ax.set_xlabel("时间 (s)"); ax.set_ylabel("PWM 指令（±6249 限幅，前进为正）")
ax.set_title("图2  四路 PID 输出（PWM 占空计数）", pad=18)
ax.legend(ncol=4, loc="lower center", bbox_to_anchor=(0.5, -0.38))
fig.tight_layout(); figs["fig2_pwm_overview"] = fig; plt.show()

# 图3：每个动作的速度跟踪（小多图）
n = len(seg_meta)
fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharey=False)
for ax, m in zip(axes.flat, seg_meta):
    sel = (data[:, C["seg"]] == m["idx"])
    tt = t[sel] - m["t_cmd"]
    for i in range(4):
        ax.plot(tt, data[sel, C[f"spd{i+1}"]] * WHEEL_SIGN[i], color=WHEEL_COLORS[i], label=WHEELS[i])
    tg = np.array([m["right"], m["right"], m["left"], m["left"]])
    for v in sorted(set(tg.tolist())):
        ax.axhline(v, color=TGT_COLOR, lw=1, ls="--")
    ax.axvline(m["t_stop"] - m["t_cmd"], color="#d03b3b", lw=1, ls=":")
    ax.set_title(m["name"]); ax.set_xlabel("动作开始后时间 (s)")
for ax in axes.flat[n:]:
    ax.axis("off")
axes[0, 0].set_ylabel("速度 (counts/10ms)"); axes[1, 0].set_ylabel("速度 (counts/10ms)")
h, l = axes.flat[0].get_legend_handles_labels()
fig.legend(h + [plt.Line2D([], [], color=TGT_COLOR, lw=1, ls="--"), plt.Line2D([], [], color="#d03b3b", lw=1, ls=":")],
           l + ["目标速度", "停车时刻"], ncol=6, loc="lower center")
fig.suptitle("图3  各动作四轮速度响应", fontweight="bold")
fig.tight_layout(rect=(0, 0.05, 1, 0.97)); figs["fig3_speed_per_segment"] = fig; plt.show()

# 图4：轨迹（编码器航向 vs IMU 航向），两张并排同坐标
has_imu = imu is not None and not np.all(np.isnan(data[:, C["gz_dps"]]))
fig, axs = plt.subplots(1, 2 if has_imu else 1, figsize=(13 if has_imu else 7, 6), squeeze=False)
def draw_traj(ax, xc, yc, thc, color, title):
    X, Y = data[:, C[yc]], data[:, C[xc]]           # 画面：车头朝上（前进 = +y），左 = -x
    ax.plot(-X, Y, color=color)
    for m in seg_meta:
        j = np.where(data[:, C["seg"]] == m["idx"])[0]
        if len(j):
            k = j[len(j) // 2]
            ax.annotate(m["name"], (-X[k], Y[k]), textcoords="offset points", xytext=(6, 4), fontsize=9, color="#52514e")
    ax.plot(0, 0, "o", ms=9, color="#0b0b0b", mec="#fcfcfb", mew=2); ax.annotate("起点", (0, 0), xytext=(-28, -14), textcoords="offset points")
    th = math.radians(data[-1, C[thc]])
    ax.plot(-X[-1], Y[-1], "s", ms=9, color=color, mec="#fcfcfb", mew=2)
    ax.annotate("", xy=(-X[-1] - 80 * math.sin(th), Y[-1] + 80 * math.cos(th)), xytext=(-X[-1], Y[-1]),
                arrowprops=dict(arrowstyle="->", color=color, lw=2))
    ax.annotate(f"终点 航向 {math.degrees(th):+.1f}°", (-X[-1], Y[-1]), xytext=(8, -14), textcoords="offset points", fontsize=9)
    ax.set_aspect("equal"); ax.set_xlabel("横向 (mm，右为正)"); ax.set_ylabel("纵向 (mm，前为正)"); ax.set_title(title)
draw_traj(axs[0, 0], "x_mm", "y_mm", "th_enc_deg", ENC_COLOR, "编码器航迹推算（航向来自轮速差）")
if has_imu:
    draw_traj(axs[0, 1], "x_imu_mm", "y_imu_mm", "th_imu_deg", IMU_COLOR, "编码器路程 + IMU 陀螺航向")
# 两图同一坐标、正方形视野（至少 600 mm 见方），避免直线轨迹被压成窄条
_xs = np.concatenate([-data[:, C["y_mm"]], [0.0]] + ([-data[:, C["y_imu_mm"]]] if has_imu else []))
_ys = np.concatenate([data[:, C["x_mm"]], [0.0]] + ([data[:, C["x_imu_mm"]]] if has_imu else []))
_span = max(600.0, np.nanmax(_xs) - np.nanmin(_xs), np.nanmax(_ys) - np.nanmin(_ys)) + 200.0
_cx, _cy = (np.nanmax(_xs) + np.nanmin(_xs)) / 2, (np.nanmax(_ys) + np.nanmin(_ys)) / 2
for a in axs[0]:
    a.set_xlim(_cx - _span / 2, _cx + _span / 2); a.set_ylim(_cy - _span / 2, _cy + _span / 2)
fig.suptitle("图4  车体轨迹（俯视，起点车头朝上）", fontweight="bold")
fig.tight_layout(); figs["fig4_trajectory"] = fig; plt.show()

# 图5：航向角随时间（编码器 vs IMU）
fig, ax = plt.subplots(figsize=(13, 3.8))
shade_segments(ax)
ax.plot(t, data[:, C["th_enc_deg"]], color=ENC_COLOR, label="编码器航向")
if has_imu:
    ax.plot(t, data[:, C["th_imu_deg"]], color=IMU_COLOR, label="IMU 陀螺积分航向")
ax.set_xlabel("时间 (s)"); ax.set_ylabel("航向角 (°，左转为正)")
ax.set_title("图5  航向角变化", pad=18)
ax.legend(ncol=2, loc="lower center", bbox_to_anchor=(0.5, -0.38))
fig.tight_layout(); figs["fig5_heading"] = fig; plt.show()

# 图6：各动作四轮稳态跟踪误差（分组柱状）
fig, ax = plt.subplots(figsize=(11, 3.8))
xs = np.arange(len(summary)); bw = 0.19
for i in range(4):
    vals = [r["_wheel"]["err"][i] for r in summary]
    ax.bar(xs + (i - 1.5) * bw, vals, width=bw - 0.02, color=WHEEL_COLORS[i], label=WHEELS[i])
ax.set_xticks(xs); ax.set_xticklabels([r["动作"] for r in summary])
ax.set_ylabel("稳态跟踪误差 (%)"); ax.set_title("图6  各动作四轮稳态速度跟踪误差")
ax.grid(axis="x", visible=False)
ax.legend(ncol=4, loc="upper left")
fig.tight_layout(); figs["fig6_tracking_error"] = fig; plt.show()
""")

md(r"""
### 6. 保存结果
""")

code(r"""
# ===== 6. 保存原始数据 / 汇总 / 图片 =====
RUN_DIR = os.path.join(NB_DIR, "chassis_demo_results", "run_" + time.strftime("%Y%m%d_%H%M%S"))
os.makedirs(RUN_DIR, exist_ok=True)

with open(os.path.join(RUN_DIR, "samples.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(COLS)
    for r in rows:
        w.writerow([f"{v:.6g}" if isinstance(v, float) else v for v in r])

def jsonable(v):
    if isinstance(v, np.ndarray): return [jsonable(x) for x in v.tolist()]
    if isinstance(v, (np.floating, float)): return None if math.isnan(float(v)) else float(v)
    if isinstance(v, (np.integer,)): return int(v)
    if isinstance(v, dict): return {k: jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return [jsonable(x) for x in v]
    return v

with open(os.path.join(RUN_DIR, "segments.csv"), "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f); w.writerow(keys)
    for r in summary:
        w.writerow([fmt(r.get(k, "")) for k in keys])

result = dict(
    params=dict(SPEED_STRAIGHT=SPEED_STRAIGHT, ARC_OUTER=ARC_OUTER, ARC_INNER=ARC_INNER, SPEED_ROTATE=SPEED_ROTATE,
                DIST_STRAIGHT_MM=DIST_STRAIGHT_MM, DIST_ARC_MM=DIST_ARC_MM, ANGLE_ROTATE_DEG=ANGLE_ROTATE_DEG,
                SAMPLE_DT=SAMPLE_DT, IMU=imu is not None, gz_bias_dps=gz_bias),
    calibration=dict(counts_per_mm=CPM, left_counts_per_deg=LEFT_CPD, right_counts_per_deg=RIGHT_CPD, track_mm=TRACK_MM),
    segments=[jsonable({k: (v if not k.startswith("_") else {kk: jsonable(vv) for kk, vv in v.items()})
                        for k, v in r.items()}) for r in summary],
    meta=[jsonable({k: v for k, v in m.items()}) for m in seg_meta],
    final_pose=dict(x_mm=float(data[-1, C["x_mm"]]), y_mm=float(data[-1, C["y_mm"]]), th_enc_deg=float(data[-1, C["th_enc_deg"]]),
                    th_imu_deg=jsonable(float(data[-1, C["th_imu_deg"]])) if imu is not None else None),
)
with open(os.path.join(RUN_DIR, "summary.json"), "w", encoding="utf-8") as f:
    json.dump(result, f, ensure_ascii=False, indent=2)

for name, fig in figs.items():
    fig.savefig(os.path.join(RUN_DIR, name + ".png"), dpi=150, bbox_inches="tight")

print("已保存到：", RUN_DIR)
for fn in sorted(os.listdir(RUN_DIR)):
    print(f"  {fn:<28} {os.path.getsize(os.path.join(RUN_DIR, fn)) / 1024:8.1f} KB")
""")

md(r"""
### 7. 保存 notebook 本身（含全部输出）
先按 **Ctrl+S** 保存本 notebook，再运行下一个单元：把带输出的 notebook 和一份 HTML 报告复制到同一个结果目录。
""")

code(r"""
# ===== 7. 导出 notebook（先 Ctrl+S！） =====
import shutil, subprocess
NB_PATH = os.path.join(NB_DIR, "chassis_motion_demo.ipynb")
shutil.copy(NB_PATH, os.path.join(RUN_DIR, "chassis_motion_demo.ipynb"))
r = subprocess.run([sys.executable, "-m", "nbconvert", "--to", "html", "--output-dir", RUN_DIR, NB_PATH],
                   capture_output=True, text=True)
print("HTML 报告：", "OK" if r.returncode == 0 else r.stderr[-500:])
print(sorted(os.listdir(RUN_DIR)))
""")

md(r"""
### 8. 结束（释放底盘）
""")

code(r"""
robot.shutdown()
print("底盘已关断，后台喂狗线程已停止。")
""")

nb = {"cells": cells,
      "metadata": {"kernelspec": {"display_name": "Python 3 (ipykernel)", "language": "python", "name": "python3"},
                   "language_info": {"name": "python"}},
      "nbformat": 4, "nbformat_minor": 5}
for i, c in enumerate(nb["cells"]):
    c["id"] = f"cell-{i:02d}"
out = sys.argv[1]
with open(out, "w", encoding="utf-8") as f:
    json.dump(nb, f, ensure_ascii=False, indent=1)
print("wrote", out, len(cells), "cells")
