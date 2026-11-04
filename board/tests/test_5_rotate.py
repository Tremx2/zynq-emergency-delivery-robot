#!/usr/bin/env python3
"""测试 5/5: 原地旋转 (不限时, Ctrl+C 停止)

默认原地左转; 加参数 `right` 改为原地右转:
    python3 test_5_rotate.py right
"""
import os
import sys
import time

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

from pynq import Overlay, MMIO
from robot_chassis import Chassis

BIT_PATH = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"
MOTOR_BASE = 0x43C00000
SPEED = 25   # 原地旋转速度, 慢速


def main():
    direction = sys.argv[1].lower() if len(sys.argv) > 1 else "left"
    if direction not in ("left", "right"):
        print("用法: python3 test_5_rotate.py [left|right]", flush=True)
        return

    print("加载 overlay ...", flush=True)
    overlay = Overlay(BIT_PATH)
    robot = Chassis(MMIO(MOTOR_BASE, 0x1000), overlay=overlay)

    armed, fault = robot.watchdog_state()
    print(f"看门狗: armed={armed} fault={fault}", flush=True)
    if not armed:
        print("!! 看门狗未布防, 退出", flush=True)
        return

    label = "原地左转" if direction == "left" else "原地右转"
    print(f"=== {label}测试 (speed={SPEED}, 不限时, Ctrl+C 停止) ===", flush=True)

    try:
        if direction == "left":
            robot.rotate_left(SPEED)
        else:
            robot.rotate_right(SPEED)

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
