from pynq import Overlay
import time
import threading


# ============================================================
# FPGA / AXI 配置
# ============================================================

DEFAULT_OVERLAY_PATH = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"

ENABLE_ADDR = 0x00

TARGET_ADDR   = [0x04, 0x08, 0x0C, 0x10]
SPEED_ADDR    = [0x14, 0x18, 0x1C, 0x20]
POSITION_ADDR = [0x24, 0x28, 0x2C, 0x30]
PWM_ADDR      = [0x34, 0x38, 0x3C, 0x40]

# ============================================================
# 电机安全看门狗 (FPGA 硬件看门狗, 500ms @ 125MHz)
#
# 复位后默认【未布防】, 此时 rpwm/lpwm 与 enable 被硬件全部门控为 0,
# 所以不布防电机永远不转。
#
#   0x4C  写 0xA5C35A3C : 布防 / 清故障 (前提: enable=0 且四个 target=0)
#   0x44  写 0xC0DE_C0DE : 喂狗心跳 (布防后必须 <500ms 一次)
#   0x48  读             : bit0=armed, bit1=fault
# ============================================================

WD_HEARTBEAT_ADDR  = 0x44
WD_STATE_ADDR      = 0x48
WD_ARM_ADDR        = 0x4C
WD_HEARTBEAT_MAGIC = 0xC0DEC0DE
WD_ARM_MAGIC       = 0xA5C35A3C


# ============================================================
# 已标定的底盘参数
# ============================================================

# 车轮参数
WHEEL_DIAMETER_MM = 75.0
WHEEL_RADIUS_MM   = 37.5
WHEEL_WIDTH_MM    = 28.0
TRACK_WIDTH_MM    = 261.6

# 四个编码器实测 counts / rev
CPR_M1 = 3941.7
CPR_M2 = 3942.7
CPR_M3 = 3938.9
CPR_M4 = 3952.15

COUNTS_PER_REV = (
    CPR_M1 + CPR_M2 + CPR_M3 + CPR_M4
) / 4.0


# ============================================================
# 直线落地标定
#
# 实测：
# 18061 counts -> 1020 mm
# ============================================================

GROUND_COUNTS_PER_MM = 18061.0 / 1020.0


# ============================================================
# 原地旋转落地标定
#
# speed = 30
#
# 左转90°：5611 counts
# 右转90°：4809.43 counts
# ============================================================

LEFT_90_COUNTS  = 5611.0
RIGHT_90_COUNTS = 4809.43

LEFT_COUNTS_PER_DEG  = LEFT_90_COUNTS / 90.0
RIGHT_COUNTS_PER_DEG = RIGHT_90_COUNTS / 90.0

ROTATE_SPEED = 30


# ============================================================
# 32位有符号数辅助
# ============================================================

def delta32(current, start):
    return ((current - start + 2**31) % 2**32) - 2**31


# ============================================================
# 底盘控制类
# ============================================================

# 运动指令租约 (秒): 电机处于运动状态时, 若超过这么久没有新的 drive()/keepalive(),
# 后台线程主动停车。防止主线程卡死 / Jupyter 单元报错后电机一直转。
# 设为 None 可关闭 (恢复旧行为: 只有进程退出时看门狗才会触发)。
DEFAULT_COMMAND_TIMEOUT = 1.0

# 同一进程内只允许一个 Chassis 在喂狗, 重复 create_robot() 时关掉旧实例
_active_chassis = None
_active_lock = threading.Lock()


class WatchdogFault(RuntimeError):
    """FPGA 看门狗未布防或已超时, 电机被硬件门控。"""


class CommandTimeout(RuntimeError):
    """运动过程中指令租约超时, 后台线程已主动停车。"""


class Chassis:

    def __init__(self, motor, overlay=None,
                 command_timeout=DEFAULT_COMMAND_TIMEOUT):

        global _active_chassis

        self.motor = motor
        self.overlay = overlay

        self.TARGET_ADDR   = TARGET_ADDR
        self.SPEED_ADDR    = SPEED_ADDR
        self.POSITION_ADDR = POSITION_ADDR
        self.PWM_ADDR      = PWM_ADDR

        # 看门狗后台喂狗线程
        self._wd_thread   = None
        self._wd_stop_evt = threading.Event()

        # 指令租约
        self.command_timeout = command_timeout
        self._cmd_lock   = threading.RLock()
        self._moving     = False
        self._last_cmd   = time.monotonic()
        self._lease_expired = False

        # 关掉同进程里的旧实例, 避免旧线程继续给新 IP 喂狗
        with _active_lock:
            old = _active_chassis
            _active_chassis = self
        if old is not None and old is not self:
            try:
                old.shutdown()
            except Exception:
                pass

        # 1) 无条件关断（不布防）：overlay 刚加载的第一时间就把电机钉死
        self._hard_disable()

        # 2) 先启动后台喂狗线程（此时看门狗尚未布防，喂狗为无害写）
        self._start_heartbeat()

        # 3) 标准停车：关断 + 清零 + 布防看门狗 + 回读校验
        self.stop()

        # 4) 上电安全自检：任何一项不通过立即抛异常，绝不带隐患返回
        self.verify_safe()


    # --------------------------------------------------------
    # 看门狗
    # --------------------------------------------------------

    def _hard_disable(self):
        """无条件关断四路电机：enable=0 + 四个 target=0，不布防看门狗。

        用在 overlay 一加载、还没确认安全之前的第一时间；此时看门狗仍未布防，
        硬件本身也会把 rpwm/lpwm 门控为 0，属双保险。
        """
        self.motor.write(ENABLE_ADDR, 0x00)
        for addr in self.TARGET_ADDR:
            self._write_s32(addr, 0)


    def _arm_watchdog(self):
        """布防 / 清故障。前提是已停车 (enable=0 且 target 全 0)。"""
        self.motor.write(WD_ARM_ADDR, WD_ARM_MAGIC)


    def _heartbeat(self):
        self.motor.write(WD_HEARTBEAT_ADDR, WD_HEARTBEAT_MAGIC)


    def watchdog_state(self):
        """读看门狗状态, 返回 (armed, fault)。"""
        st = self.motor.read(WD_STATE_ADDR)
        return bool(st & 1), bool(st & 2)


    def check_watchdog(self):
        """未布防或已超时则抛 WatchdogFault。调用 stop() 可重新布防。"""
        armed, fault = self.watchdog_state()
        if fault or not armed:
            raise WatchdogFault(
                f"FPGA 看门狗 armed={armed} fault={fault}, 电机已被硬件门控; "
                "确认安全后调用 stop() 重新布防"
            )


    def keepalive(self):
        """刷新运动指令租约。长时间保持同一速度的循环里需要周期性调用。

        若租约已过期 (后台线程已自动停车), 抛 CommandTimeout, 不会悄悄恢复运动。
        """
        self._check_lease()
        self._last_cmd = time.monotonic()


    def _check_lease(self):
        if self._lease_expired:
            raise CommandTimeout(
                f"超过 {self.command_timeout}s 没有新的运动指令, 已自动停车"
            )


    def _start_heartbeat(self):
        self._wd_stop_evt.clear()
        self._wd_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
        )
        self._wd_thread.start()


    def _heartbeat_loop(self):
        while not self._wd_stop_evt.is_set():
            try:
                if (
                    self._moving
                    and self.command_timeout is not None
                    and time.monotonic() - self._last_cmd > self.command_timeout
                ):
                    # 主线程长时间没发指令: 主动停车。stop() 写 0 后会重新布防,
                    # 之后继续按静止状态喂狗。若本线程也卡住, 硬件看门狗 500ms 兜底。
                    with self._cmd_lock:
                        if self._moving:
                            self.stop()
                            self._lease_expired = True
                self._heartbeat()
            except Exception:
                pass
            self._wd_stop_evt.wait(0.1)


    def shutdown(self):
        """停止喂狗线程并停车, 释放后台资源。

        停止心跳后, 已布防的看门狗会在 500ms 后超时并锁存 fault (预期行为,
        电机保持门控)。下次创建 Chassis 时 stop() 会重新布防并清除。
        """
        global _active_chassis
        self._wd_stop_evt.set()
        if self._wd_thread is not None:
            self._wd_thread.join(timeout=0.5)
        with self._cmd_lock:
            self._moving = False
            self.motor.write(ENABLE_ADDR, 0x00)
            for addr in self.TARGET_ADDR:
                self._write_s32(addr, 0)
        with _active_lock:
            if _active_chassis is self:
                _active_chassis = None


    # --------------------------------------------------------
    # AXI signed 32-bit
    # --------------------------------------------------------

    def _write_s32(self, addr, value):

        self.motor.write(
            addr,
            int(value) & 0xFFFFFFFF
        )


    def _read_s32(self, addr):

        value = self.motor.read(addr)

        if value & 0x80000000:
            value -= 0x100000000

        return value


    # --------------------------------------------------------
    # 四电机底层控制
    #
    # M1 = 右前
    # M2 = 右后
    # M3 = 左后
    # M4 = 左前
    # --------------------------------------------------------

    def _set_motors(self, m1, m2, m3, m4):

        if self._wd_stop_evt.is_set():
            raise RuntimeError("Chassis 已 shutdown, 不能再发运动指令")

        values = [m1, m2, m3, m4]

        with self._cmd_lock:
            # 看门狗已超时 / 未布防时拒绝执行, 不再静默空跑
            self.check_watchdog()

            self._lease_expired = False
            self._last_cmd = time.monotonic()
            self._moving = any(int(v) != 0 for v in values)

            for addr, value in zip(
                self.TARGET_ADDR,
                values
            ):
                self._write_s32(addr, value)

            self.motor.write(ENABLE_ADDR, 0x0F)


    # --------------------------------------------------------
    # 左右轮逻辑控制
    #
    # 已实车验证：
    #
    # 车辆前进：
    # M1 = -
    # M2 = -
    # M3 = +
    # M4 = +
    # --------------------------------------------------------

    def drive(self, left_speed, right_speed):

        m1 = -right_speed
        m2 = -right_speed
        m3 =  left_speed
        m4 =  left_speed

        self._set_motors(
            m1,
            m2,
            m3,
            m4
        )


    # --------------------------------------------------------
    # 停车
    # --------------------------------------------------------

    def stop(self):

        with self._cmd_lock:
            self._moving = False

            self.motor.write(ENABLE_ADDR, 0x00)

            for addr in self.TARGET_ADDR:
                self._write_s32(addr, 0)

            # 停车后重新布防看门狗 (同时清除可能发生的超时故障)
            self._arm_watchdog()

        # 回读校验：关断命令确实生效，否则立即抛异常，绝不静默带病返回
        armed, fault = self.watchdog_state()
        enable = self.motor.read(ENABLE_ADDR)
        if fault or not armed:
            raise RuntimeError(
                f"stop() 后看门狗状态异常 armed={armed} fault={fault}"
            )
        if enable != 0:
            raise RuntimeError(f"stop() 后 enable 回读非 0：0x{enable:08X}")


    # --------------------------------------------------------
    # 基础运动
    # --------------------------------------------------------

    def forward(self, speed=30):

        speed = abs(speed)

        self.drive(
            speed,
            speed
        )


    def backward(self, speed=30):

        speed = abs(speed)

        self.drive(
            -speed,
            -speed
        )


    # 前进过程中向左走弧线
    def turn_left(
        self,
        outer_speed=45,
        inner_speed=30
    ):

        self.drive(
            abs(inner_speed),
            abs(outer_speed)
        )


    # 前进过程中向右走弧线
    def turn_right(
        self,
        outer_speed=45,
        inner_speed=30
    ):

        self.drive(
            abs(outer_speed),
            abs(inner_speed)
        )


    # 原地左转
    def rotate_left(self, speed=ROTATE_SPEED):

        speed = abs(speed)

        self.drive(
            -speed,
            speed
        )


    # 原地右转
    def rotate_right(self, speed=ROTATE_SPEED):

        speed = abs(speed)

        self.drive(
            speed,
            -speed
        )


    # --------------------------------------------------------
    # 状态读取
    # --------------------------------------------------------

    def get_speeds(self):

        return [
            self._read_s32(addr)
            for addr in self.SPEED_ADDR
        ]


    def get_positions(self):

        return [
            self._read_s32(addr)
            for addr in self.POSITION_ADDR
        ]


    def get_pwms(self):

        return [
            self._read_s32(addr)
            for addr in self.PWM_ADDR
        ]


    # --------------------------------------------------------
    # 上电安全自检
    # --------------------------------------------------------

    def self_test(self):
        """读取所有安全相关寄存器，返回报告 dict（不抛异常）。

        返回键：safe(是否安全)、armed、fault、enable、pwm_command、
                actual_speed、problems(问题列表)。
        """
        armed, fault = self.watchdog_state()
        enable = self.motor.read(ENABLE_ADDR)
        pwms = self.get_pwms()
        speeds = self.get_speeds()

        problems = []
        if fault:
            problems.append("看门狗已锁存超时故障(fault=1)，需调用 stop() 清除")
        if enable != 0:
            problems.append(f"enable 非 0：0x{enable:08X}")
        if any(v != 0 for v in pwms):
            problems.append(f"存在非零 pwm_command：{pwms}")
        if any(v != 0 for v in speeds):
            problems.append(f"存在非零实际速度：{speeds}（电机可能在转！）")

        return {
            "safe": not problems,
            "armed": armed,
            "fault": fault,
            "enable": enable,
            "pwm_command": pwms,
            "actual_speed": speeds,
            "problems": problems,
        }


    def verify_safe(self, expect_armed=True):
        """上电安全自检：任何一项不安全立即抛 RuntimeError。

        expect_armed=True 时还要求看门狗已布防（未布防虽更安全但无法运动）。
        返回 self_test() 的报告。
        """
        report = self.self_test()
        if not report["safe"]:
            raise RuntimeError(
                "上电安全自检未通过：\n  " + "\n  ".join(report["problems"])
            )
        if expect_armed and not report["armed"]:
            raise RuntimeError(
                "看门狗未布防（armed=0）。未布防虽安全但电机无法运动，"
                "请调用 stop() 重新布防。"
            )
        return report


    # ========================================================
    # 定距离直行
    #
    # distance_mm > 0 ：前进
    # distance_mm < 0 ：后退
    #
    # 已实车验证前进/后退
    # ========================================================

    def move_distance(
        self,
        distance_mm,
        speed=50,
        slow_speed=30,
        slow_zone_mm=100,
        timeout=30
    ):

        if distance_mm == 0:
            self.stop()
            return 0.0

        direction = (
            1 if distance_mm > 0 else -1
        )

        target_mm = abs(float(distance_mm))

        target_counts = (
            target_mm *
            GROUND_COUNTS_PER_MM
        )

        slow_zone_counts = (
            slow_zone_mm *
            GROUND_COUNTS_PER_MM
        )

        start_pos = self.get_positions()

        start_time = time.time()

        slowed = False

        try:

            # 启动车辆
            if direction > 0:
                self.forward(speed)
            else:
                self.backward(speed)

            while True:

                time.sleep(0.02)

                # 循环仍在运行: 续租; 若期间被自动停车或看门狗超时则报错退出
                self._check_lease()
                self.check_watchdog()
                self.keepalive()

                pos = self.get_positions()

                raw = [
                    delta32(
                        pos[i],
                        start_pos[i]
                    )
                    for i in range(4)
                ]

                # 转换成“车辆前进方向”为正
                forward_counts = [
                    -raw[0],
                    -raw[1],
                     raw[2],
                     raw[3]
                ]

                travel_counts = [
                    x * direction
                    for x in forward_counts
                ]

                avg_counts = (
                    sum(travel_counts) / 4.0
                )

                remaining_counts = (
                    target_counts -
                    avg_counts
                )

                # 接近目标时减速
                if (
                    not slowed
                    and remaining_counts <= slow_zone_counts
                    and remaining_counts > 0
                ):

                    slowed = True

                    if direction > 0:
                        self.forward(slow_speed)
                    else:
                        self.backward(slow_speed)

                # 到达目标
                if avg_counts >= target_counts:
                    break

                # 超时保护
                if (
                    time.time() -
                    start_time
                    >= timeout
                ):
                    print("MOVE TIMEOUT")
                    break

        finally:

            self.stop()


        # 等待车辆完全停止
        time.sleep(0.7)

        end_pos = self.get_positions()

        raw = [
            delta32(
                end_pos[i],
                start_pos[i]
            )
            for i in range(4)
        ]

        forward_counts = [
            -raw[0],
            -raw[1],
             raw[2],
             raw[3]
        ]

        final_counts = [
            x * direction
            for x in forward_counts
        ]

        avg_final = (
            sum(final_counts) / 4.0
        )

        final_mm = (
            avg_final /
            GROUND_COUNTS_PER_MM
        )

        return final_mm


    # ========================================================
    # 定角度原地旋转
    #
    # angle_deg > 0 ：左转
    # angle_deg < 0 ：右转
    #
    # 左右90°已经分别实车标定
    # ========================================================

    def rotate_angle(
        self,
        angle_deg,
        timeout=30
    ):

        if angle_deg == 0:
            self.stop()
            return 0.0

        angle = abs(float(angle_deg))


        # ---------------------------------------------
        # 根据方向使用不同实测标定值
        # ---------------------------------------------

        if angle_deg > 0:

            # 左转
            counts_per_deg = (
                LEFT_COUNTS_PER_DEG
            )

        else:

            # 右转
            counts_per_deg = (
                RIGHT_COUNTS_PER_DEG
            )


        target_counts = (
            angle *
            counts_per_deg
        )

        start_pos = self.get_positions()

        start_time = time.time()


        try:

            # 固定 speed=30
            # 与标定时保持完全相同
            if angle_deg > 0:
                self.rotate_left(ROTATE_SPEED)
            else:
                self.rotate_right(ROTATE_SPEED)


            while True:

                time.sleep(0.02)

                # 循环仍在运行: 续租; 若期间被自动停车或看门狗超时则报错退出
                self._check_lease()
                self.check_watchdog()
                self.keepalive()

                pos = self.get_positions()

                raw = [
                    delta32(
                        pos[i],
                        start_pos[i]
                    )
                    for i in range(4)
                ]

                wheel_counts = [
                    abs(x)
                    for x in raw
                ]

                avg_counts = (
                    sum(wheel_counts) / 4.0
                )


                if avg_counts >= target_counts:
                    break


                if (
                    time.time() -
                    start_time
                    >= timeout
                ):

                    print("ROTATE TIMEOUT")
                    break


        finally:

            self.stop()


        # 等车完全停稳
        time.sleep(0.7)

        end_pos = self.get_positions()

        raw = [
            delta32(
                end_pos[i],
                start_pos[i]
            )
            for i in range(4)
        ]

        final_counts = [
            abs(x)
            for x in raw
        ]

        avg_final = (
            sum(final_counts) / 4.0
        )

        actual_angle = (
            avg_final /
            counts_per_deg
        )

        return actual_angle


# ============================================================
# 创建机器人
# ============================================================

def create_robot(
    overlay_path=DEFAULT_OVERLAY_PATH,
    command_timeout=DEFAULT_COMMAND_TIMEOUT
):

    print("Loading FPGA overlay...")

    overlay = Overlay(
        overlay_path
    )

    motor = overlay.robot_motor_axi_0

    robot = Chassis(
        motor,
        overlay=overlay,
        command_timeout=command_timeout
    )

    print("==============================")
    print("ROBOT READY")
    print("==============================")
    print("Straight calibration:")
    print(
        " counts/mm =",
        GROUND_COUNTS_PER_MM
    )

    print("Rotation calibration:")
    print(
        " left 90  =",
        LEFT_90_COUNTS
    )
    print(
        " right 90 =",
        RIGHT_90_COUNTS
    )

    # 上电安全自检报告
    report = robot.self_test()
    if report["safe"]:
        print("上电自检 PASS：enable=0 / 4路 pwm=0 / 4路速度=0 / 看门狗已布防无故障")
    else:
        print("上电自检 FAIL：", report["problems"])

    print("Motors are disabled.")

    return robot
