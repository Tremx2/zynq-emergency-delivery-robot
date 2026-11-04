#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""开机自启：加载电机 overlay 并把四路 PWM 主动关断（拉低）。

背景：BTS7960 模块的 RPWM/LPWM 输入带内部上拉到 5V，断电时靠外部 4.7k/8.2k 下拉
只能压到约 4.5V（仍是高电平），无法真正拉低；且个别通道存在内部漏电，会把其中一路
PWM 拉低、造成"一高一低"的半桥误导通 -> 电机自转。根治办法是让 FPGA 尽快配置完成，
由电机 IP 主动输出低电平（FPGA 驱动能力远大于模块内部上拉）。

本脚本：加载 robot_system_wrapper.bit -> 显式 enable=0 / target=0。
看门狗保持复位默认【未布防】，硬件把 rpwm/lpwm 门控为 0，四路 PWM 被 FPGA 主动拉低。
"""
import os
import sys
import time

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

from pynq import Overlay, MMIO

BIT = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"
MOTOR = 0x43C00000
RETRIES = 15
RETRY_DELAY = 2.0


def main():
    ol = None
    last_err = None
    for i in range(RETRIES):
        try:
            ol = Overlay(BIT)
            break
        except Exception as e:
            last_err = e
            print(f"[boot] overlay 加载第 {i + 1} 次失败: {e}", flush=True)
            time.sleep(RETRY_DELAY)

    if ol is None:
        print(f"[boot] overlay 最终加载失败: {last_err}", flush=True)
        sys.exit(1)

    m = MMIO(MOTOR, 0x1000)
    m.write(0x00, 0)                      # enable = 0
    for a in (0x04, 0x08, 0x0C, 0x10):
        m.write(a, 0)                     # target = 0

    wd = m.read(0x48)
    enable = m.read(0x00)
    print(f"[boot] overlay 已加载，四路已关断 "
          f"(enable=0x{enable:X}, 看门狗 0x{wd:08X}, armed={bool(wd & 1)})", flush=True)


if __name__ == "__main__":
    main()
