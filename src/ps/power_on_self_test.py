#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上电安全自检（独立脚本）

用法（板上，用 pynq 的 venv 运行）：
    sudo /usr/local/share/pynq-venv/bin/python3 power_on_self_test.py

流程：
  1. 加载 overlay
  2. 无条件关断四路电机（enable=0 + 四个 target=0，不布防看门狗）
  3. 回读并打印：看门狗状态 / enable / 四路 pwm_command / 四路实际速度 / 位置
  4. 判定 PASS/FAIL：看门狗有故障、enable 非 0、任何一路 pwm 或速度非零 => FAIL

作用：排查“上电后某个电机自己转”这类问题时，先跑一遍确认 FPGA 侧确实把
PWM 关到了 0。如果这里 FAIL 且速度非零，说明问题仍在硬件（浮空输入/线松/共地），
不是软件。
"""
from pynq import Overlay
import time

DEFAULT_OVERLAY_PATH = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"

ENABLE_ADDR    = 0x00
TARGET_ADDR    = [0x04, 0x08, 0x0C, 0x10]
SPEED_ADDR     = [0x14, 0x18, 0x1C, 0x20]
POSITION_ADDR  = [0x24, 0x28, 0x2C, 0x30]
PWM_ADDR       = [0x34, 0x38, 0x3C, 0x40]
WD_STATE_ADDR  = 0x48
WD_ARM_ADDR    = 0x4C
WD_ARM_MAGIC   = 0xA5C35A3C


def read_s32(motor, addr):
    v = motor.read(addr)
    if v & 0x80000000:
        v -= 0x100000000
    return v


def main():
    print("加载 overlay ...")
    overlay = Overlay(DEFAULT_OVERLAY_PATH)
    motor = overlay.robot_motor_axi_0

    # 1) 无条件关断（不布防；此时硬件看门狗未布防，rpwm/lpwm 也被门控为 0）
    motor.write(ENABLE_ADDR, 0x00)
    for a in TARGET_ADDR:
        motor.write(a, 0)
    time.sleep(0.05)

    # 2) 回读安全状态
    st = motor.read(WD_STATE_ADDR)
    armed = bool(st & 1)
    fault = bool(st & 2)

    enable = motor.read(ENABLE_ADDR)
    pwms   = [read_s32(motor, a) for a in PWM_ADDR]
    speeds = [read_s32(motor, a) for a in SPEED_ADDR]
    poss   = [read_s32(motor, a) for a in POSITION_ADDR]

    print("=" * 52)
    print("上电安全自检")
    print("=" * 52)
    print(f"看门狗: armed={armed}  fault={fault}")
    print(f"enable : 0x{enable:08X}")
    print(f"pwm_cmd: {pwms}")
    print(f"速度   : {speeds}")
    print(f"位置   : {poss}")

    problems = []
    if fault:
        problems.append("看门狗故障已锁存（fault=1）")
    if enable != 0:
        problems.append(f"enable 非 0（0x{enable:08X}）")
    if any(v != 0 for v in pwms):
        problems.append(f"存在非零 pwm_command {pwms}")
    if any(v != 0 for v in speeds):
        problems.append(f"存在非零速度 {speeds}，电机可能在转！")

    if problems:
        print("-" * 52)
        print("FAIL:")
        for p in problems:
            print("  -", p)
        print("建议：检查 PWM 线是否松动 / 是否有下拉电阻 / 是否共地。")
    else:
        print("-" * 52)
        print("PASS：四路电机均处于关断状态，可安全使用。")
        print("提示：正常初始化请用 robot_chassis.create_robot()。")


if __name__ == "__main__":
    main()
