#!/usr/bin/env python3
"""测试 3/5: 左转 (前进中弧线左转, 不限时, Ctrl+C 停止)"""
import os
import time

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

from pynq import Overlay, MMIO
from robot_chassis import Chassis

BIT_PATH = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"
MOTOR_BASE = 0x43C00000

# 弧线转弯: 内圈慢 / 外圈快, 都保持低速
ARC_OUTER_SPEED = 35
ARC_INNER_SPEED = 25


def main():
    print("加载 overlay ...", flush=True)
    overlay = Overlay(BIT_PATH)
    robot = Chassis(MMIO(MOTOR_BASE, 0x1000), overlay=overlay)

    armed, fault = robot.watchdog_state()
    print(f"看门狗: armed={armed} fault={fault}", flush=True)
    if not armed:
        print("!! 看门狗未布防, 退出", flush=True)
        return

    print(
        f"=== 左转测试 (外圈={ARC_OUTER_SPEED}, 内圈={ARC_INNER_SPEED}, "
        f"不限时, Ctrl+C 停止) ===",
        flush=True,
    )
    try:
        robot.turn_left(ARC_OUTER_SPEED, ARC_INNER_SPEED)
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
