#!/usr/bin/env python3
"""测试 2/5: 后退 (不限时, Ctrl+C 停止)"""
import os
import time

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

from pynq import Overlay, MMIO
from robot_chassis import Chassis

BIT_PATH = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"
MOTOR_BASE = 0x43C00000
SPEED = 30   # 慢速, 需要更慢/更快改这里


def main():
    print("加载 overlay ...", flush=True)
    overlay = Overlay(BIT_PATH)
    robot = Chassis(MMIO(MOTOR_BASE, 0x1000), overlay=overlay)

    armed, fault = robot.watchdog_state()
    print(f"看门狗: armed={armed} fault={fault}", flush=True)
    if not armed:
        print("!! 看门狗未布防, 退出", flush=True)
        return

    print(f"=== 后退测试 (speed={SPEED}, 不限时, Ctrl+C 停止) ===", flush=True)
    try:
        robot.backward(SPEED)
        while True:
            time.sleep(0.5)
            robot.keepalive()  # 续租, 否则 1s 后自动停车
            print(
                f"speed={robot.get_speeds()} "
                f"pwm={robot.get_pwms()} "
                f"pos={robot.get_positions()}",
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        print("已停车", flush=True)


if __name__ == "__main__":
    main()
