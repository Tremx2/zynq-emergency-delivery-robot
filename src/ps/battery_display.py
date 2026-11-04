#!/usr/bin/env python3
"""OLED (SH1106 / SSD1306) 显示 3S 锂电池电量 + INA219 电压检测。

INA219 与 OLED 都挂在 PL 的 axi_iic_0 (0x41600000, SCL=T10 / SDA=T11) 上,
与 ToF(0x29)/IMU(0x69) 共用同一条 I2C 总线:
  - INA219 @ 0x40 (默认 A0=A1=GND)
  - OLED   @ 0x3C (1.3寸 SH1106 / 0.96寸 SSD1306, 默认 SA0/DC=GND)

这两个都是 I2C 器件, 不需要新增 FPGA/Verilog 逻辑 —— 测电量和驱动 OLED
全部由 PS 端通过已有的 axi_iic 读寄存器完成。

用法:
  python3 battery_display.py          循环刷新 (~1Hz), Ctrl+C 停止; 默认 1.3寸 SH1106
  python3 battery_display.py --once   只读一次电压并显示一帧后退出
  python3 battery_display.py --ssd1306  改用 SSD1306 驱动 (0.96寸)
  python3 battery_display.py --reload 强制重新下载 bitstream (默认已加载则复用)
"""
import os
import statistics
import sys
import time

os.environ["XILINX_XRT"] = "/usr"
os.environ["BOARD"] = "Pynq-Z2"

from pynq import Overlay, MMIO, PL
from test_sensors import AxiIic, I2CError

BIT_PATH = "/home/xilinx/jupyter_notebooks/robot_system_wrapper.bit"
IIC_BASE = 0x41600000
MOTOR_BASE = 0x43C00000
TOF_XSHUT_ADDR = 0x50       # 拉高 ToF XSHUT, 让 0x29 也出现在总线扫描里 (可选)

INA219_ADDR = 0x40
OLED_ADDR = 0x3C


# ============================================================
# INA219 (总线电压 / 电流检测)
# ============================================================

class INA219:
    REG_CONFIG = 0x00
    REG_SHUNT = 0x01
    REG_BUS = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CAL = 0x05

    # 32V 量程, ±320mV 分流, 12bit ADC, 连续采样 (总线电压 LSB 恒为 4mV)
    CONFIG_DEFAULT = 0x399F
    # 板上分流电阻 0.1Ω; 分流 LSB=10µV -> 电流 LSB=100µA -> Cal=4096
    SHUNT_OHM = 0.1
    CURRENT_LSB = 100e-6
    CAL_VALUE = int(0.04096 / (CURRENT_LSB * SHUNT_OHM))

    def __init__(self, bus, addr=INA219_ADDR):
        self.bus = bus
        self.addr = addr

    def rd(self, reg):
        return int.from_bytes(self.bus.write_read(self.addr, bytes([reg]), 2), "big")

    def wr(self, reg, val):
        self.bus.write(self.addr, bytes([reg, (val >> 8) & 0xFF, val & 0xFF]))

    def init(self):
        self.wr(self.REG_CONFIG, self.CONFIG_DEFAULT)
        self.wr(self.REG_CAL, self.CAL_VALUE)

    def bus_voltage(self):
        """总线电压 (V) —— 也就是电池电压。寄存器高 13 位为电压, LSB=4mV。"""
        return (self.rd(self.REG_BUS) >> 3) * 0.004

    def shunt_voltage(self):
        raw = self.rd(self.REG_SHUNT)
        if raw & 0x8000:
            raw -= 0x10000
        return raw * 10e-6          # 10µV LSB @ ±320mV

    def current(self):
        raw = self.rd(self.REG_CURRENT)
        if raw & 0x8000:
            raw -= 0x10000
        return raw * self.CURRENT_LSB


# 3S LiPo 放电电压 -> 电量% (分段线性插值; 4.2V/节满, 3.2V/节空)
_BAT_LUT = [
    (12.60, 100.0),
    (12.00, 85.0),
    (11.40, 60.0),
    (10.80, 35.0),
    (10.20, 15.0),
    (9.60, 0.0),
]


def battery_percent(v):
    if v >= _BAT_LUT[0][0]:
        return 100.0
    if v <= _BAT_LUT[-1][0]:
        return 0.0
    for (v1, p1), (v2, p2) in zip(_BAT_LUT, _BAT_LUT[1:]):
        if v2 <= v <= v1:
            return p2 + (p1 - p2) * (v - v2) / (v1 - v2)
    return 0.0


# ============================================================
# 128x64 OLED (I2C): SSD1306 (0.96寸) 与 SH1106 (1.3寸)
# ============================================================

# 5x7 点阵字库 (每字符 5 字节 = 5 列, bit0 = 最上面一行)
_FONT = {
    '0': (0x3E, 0x51, 0x49, 0x45, 0x3E),
    '1': (0x00, 0x42, 0x7F, 0x40, 0x00),
    '2': (0x42, 0x61, 0x51, 0x49, 0x46),
    '3': (0x21, 0x41, 0x45, 0x4B, 0x31),
    '4': (0x18, 0x14, 0x12, 0x7F, 0x10),
    '5': (0x27, 0x45, 0x45, 0x45, 0x39),
    '6': (0x3C, 0x4A, 0x49, 0x49, 0x30),
    '7': (0x01, 0x71, 0x09, 0x05, 0x03),
    '8': (0x36, 0x49, 0x49, 0x49, 0x36),
    '9': (0x06, 0x49, 0x49, 0x29, 0x1E),
    'A': (0x7E, 0x11, 0x11, 0x11, 0x7E),
    'B': (0x7F, 0x49, 0x49, 0x49, 0x36),
    'C': (0x3E, 0x41, 0x41, 0x41, 0x22),
    'D': (0x7F, 0x41, 0x41, 0x22, 0x1C),
    'E': (0x7F, 0x49, 0x49, 0x49, 0x41),
    'F': (0x7F, 0x09, 0x09, 0x09, 0x01),
    'G': (0x3E, 0x41, 0x49, 0x49, 0x7A),
    'H': (0x7F, 0x08, 0x08, 0x08, 0x7F),
    'I': (0x00, 0x41, 0x7F, 0x41, 0x00),
    'J': (0x20, 0x40, 0x41, 0x3F, 0x01),
    'K': (0x7F, 0x08, 0x14, 0x22, 0x41),
    'L': (0x7F, 0x40, 0x40, 0x40, 0x40),
    'M': (0x7F, 0x02, 0x0C, 0x02, 0x7F),
    'N': (0x7F, 0x04, 0x08, 0x10, 0x7F),
    'O': (0x3E, 0x41, 0x41, 0x41, 0x3E),
    'P': (0x7F, 0x09, 0x09, 0x09, 0x06),
    'Q': (0x3E, 0x41, 0x51, 0x21, 0x5E),
    'R': (0x7F, 0x09, 0x19, 0x29, 0x46),
    'S': (0x46, 0x49, 0x49, 0x49, 0x31),
    'T': (0x01, 0x01, 0x7F, 0x01, 0x01),
    'U': (0x3F, 0x40, 0x40, 0x40, 0x3F),
    'V': (0x1F, 0x20, 0x40, 0x20, 0x1F),
    'W': (0x3F, 0x40, 0x38, 0x40, 0x3F),
    'X': (0x63, 0x14, 0x08, 0x14, 0x63),
    'Y': (0x07, 0x08, 0x70, 0x08, 0x07),
    'Z': (0x61, 0x51, 0x49, 0x45, 0x43),
    ' ': (0x00, 0x00, 0x00, 0x00, 0x00),
    '.': (0x00, 0x60, 0x60, 0x00, 0x00),
    '%': (0x23, 0x13, 0x08, 0x64, 0x62),
    ':': (0x00, 0x36, 0x36, 0x00, 0x00),
    '-': (0x08, 0x08, 0x08, 0x08, 0x08),
    '/': (0x20, 0x10, 0x08, 0x04, 0x02),
}


class SSD1306:
    def __init__(self, bus, addr=OLED_ADDR, width=128, height=64):
        self.bus = bus
        self.addr = addr
        self.width = width
        self.height = height
        self.pages = height // 8
        self.buffer = bytearray(width * self.pages)
        self._init()

    # ---- 底层 I2C ----
    def _cmd(self, *cmds):
        # 控制字节 0x00 = Co=0, D/C#=0: 之后所有字节都是命令 (含命令参数)。
        # 不能用 0x80 (Co=1): 那样每个后续字节都会被当成新的控制字节, 多字节命令的参数会丢。
        self.bus.write(self.addr, bytes([0x00]) + bytes(cmds))

    def _data(self, chunk):
        # 控制字节 0x40 = 之后全为显示数据
        self.bus.write(self.addr, bytes([0x40]) + bytes(chunk))

    def _init(self):
        self._cmd(0xAE)              # display off
        self._cmd(0xD5, 0x80)        # clock div
        self._cmd(0xA8, 0x3F)        # mux ratio 64
        self._cmd(0xD3, 0x00)        # display offset
        self._cmd(0x40)              # start line 0
        self._cmd(0x8D, 0x14)        # charge pump on
        self._cmd(0x20, 0x00)        # memory mode: horizontal
        self._cmd(0xA1)              # segment remap
        self._cmd(0xC8)              # COM scan dec
        self._cmd(0xDA, 0x12)        # COM pins
        self._cmd(0x81, 0xCF)        # contrast
        self._cmd(0xD9, 0xF1)        # precharge
        self._cmd(0xDB, 0x40)        # VCOM detect
        self._cmd(0xA4)              # resume
        self._cmd(0xA6)              # normal (不反色)
        self._cmd(0xAF)              # display on
        self.clear()
        self.show()

    # ---- 帧缓冲操作 ----
    def clear(self):
        for i in range(len(self.buffer)):
            self.buffer[i] = 0

    def pixel(self, x, y, on=1):
        if 0 <= x < self.width and 0 <= y < self.height:
            if on:
                self.buffer[x + (y // 8) * self.width] |= 1 << (y % 8)
            else:
                self.buffer[x + (y // 8) * self.width] &= ~(1 << (y % 8))

    def fill_rect(self, x, y, w, h, on=1):
        for yy in range(y, y + h):
            for xx in range(x, x + w):
                self.pixel(xx, yy, on)

    def rect(self, x, y, w, h, on=1):
        self.fill_rect(x, y, w, 1, on)
        self.fill_rect(x, y + h - 1, w, 1, on)
        self.fill_rect(x, y, 1, h, on)
        self.fill_rect(x + w - 1, y, 1, h, on)

    def text(self, s, x, y, scale=1):
        cx = x
        for ch in s.upper():
            glyph = _FONT.get(ch, _FONT[' '])
            for col in range(5):
                bits = glyph[col]
                for row in range(7):
                    if bits & (1 << row):
                        self.fill_rect(cx + col * scale, y + row * scale, scale, scale)
            cx += 6 * scale

    def battery_bar(self, x, y, w, h, pct):
        self.rect(x, y, w, h)
        self.fill_rect(x + w, y + h // 4, 3, h // 2)      # 正极小凸起
        inner = int((w - 2) * max(0.0, min(100.0, pct)) / 100.0)
        if inner > 0:
            self.fill_rect(x + 2, y + 2, inner, h - 4)

    def show(self):
        # 定位到 (col 0..127, page 0..7) 后整屏推数据
        self._cmd(0x21, 0x00, self.width - 1)   # 列地址范围
        self._cmd(0x22, 0x00, self.pages - 1)   # 页地址范围
        for i in range(0, len(self.buffer), 16):
            self._data(self.buffer[i:i + 16])


class SH1106(SSD1306):
    """1.3寸 128x64 OLED (SH1106 控制器)。

    与 SSD1306 的区别: 显存 132 列(可见 128 列), 只支持页寻址(不支持
    0x21/0x22 列/页地址范围命令), 无 0x8D 电荷泵命令(改用 0xAD 使能 DC-DC)。
    数据从列 COL_OFFSET 开始写入。
    """
    COL_OFFSET = 2   # SH1106 显存前 2 列是 dummy, 可见区从第 2 列开始

    def _init(self):
        self._cmd(0xAE)              # display off
        self._cmd(0xD5, 0x80)        # clock div
        self._cmd(0xA8, 0x3F)        # mux ratio 64
        self._cmd(0xD3, 0x00)        # display offset
        self._cmd(0x40)              # start line 0
        self._cmd(0xAD, 0x8B)        # DC-DC 使能 (SH1106 内置电荷泵)
        self._cmd(0xA1)              # segment remap
        self._cmd(0xC8)              # COM scan dec
        self._cmd(0xDA, 0x12)        # COM pins
        self._cmd(0x81, 0xCF)        # contrast
        self._cmd(0xD9, 0xF1)        # precharge
        self._cmd(0x83, 0x00)        # 输出电压内部参考 (可选, 更稳)
        self._cmd(0xDB, 0x40)        # VCOM detect
        self._cmd(0xA4)              # resume
        self._cmd(0xA6)              # normal (不反色)
        self._cmd(0xAF)              # display on
        self.clear()
        self.show()

    def show(self):
        # SH1106 只能页寻址: 每页先设页地址+列起始, 再推 128 字节
        for page in range(self.pages):
            self._cmd(0xB0 | page)                              # 页地址
            self._cmd(0x00 | (self.COL_OFFSET & 0x0F))          # 列低 4 位
            self._cmd(0x10 | (self.COL_OFFSET >> 4))            # 列高 4 位
            start = page * self.width
            chunk = self.buffer[start:start + self.width]
            for i in range(0, len(chunk), 16):
                self._data(chunk[i:i + 16])


# ============================================================
# 显示与主流程
# ============================================================

def render(oled, v, pct):
    oled.clear()
    oled.text(f"{v:.2f}V", 0, 0, scale=2)          # 电压大字
    oled.text(f"{pct:.0f}%", 0, 22, scale=2)       # 电量百分号大字
    oled.battery_bar(8, 50, 112, 12, pct)          # 底部电池条
    if pct < 20.0:
        oled.text("LOW", 84, 26)                   # 低电量提示
    oled.show()


def main():
    once = "--once" in sys.argv

    # 已加载同一 bitstream 时不重复下载: 重新下载会复位 PL 里的电机 IP,
    # 如果此时另一个进程在开车, 电机会被突然断开。需要强制重载时加 --reload。
    if ("--reload" not in sys.argv and PL.bitfile_name
            and os.path.basename(PL.bitfile_name) == os.path.basename(BIT_PATH)):
        print("overlay 已加载, 不重复下载", flush=True)
        Overlay(BIT_PATH, download=False)
    else:
        print("加载 overlay ...", flush=True)
        Overlay(BIT_PATH)
    # 拉高 ToF XSHUT (W14), 让 0x29 也出现在总线扫描里 (与最终 1-ToF 版一致)
    MMIO(MOTOR_BASE, 0x1000).write(TOF_XSHUT_ADDR, 0x1)

    # OLED 整屏刷数据耗时较长, 用更宽松的 I2C 超时
    bus = AxiIic(timeout=0.5)

    found = bus.scan()
    bus.errors = 0
    print("总线设备:", " ".join(f"0x{a:02X}" for a in found), flush=True)

    if INA219_ADDR not in found:
        print(f"错误: INA219 不在总线 (0x{INA219_ADDR:02X}), 检查接线/VCC", flush=True)
        return
    if OLED_ADDR not in found:
        print(f"错误: OLED 不在总线 (0x{OLED_ADDR:02X}), 检查接线", flush=True)
        return

    ina = INA219(bus)
    ina.init()
    use_ssd1306 = "--ssd1306" in sys.argv
    oled = SSD1306(bus) if use_ssd1306 else SH1106(bus)
    print("显示驱动:", "SSD1306 (0.96寸)" if use_ssd1306 else "SH1106 (1.3寸)", flush=True)

    print("开始测电量 (Ctrl+C 停止) ...", flush=True)
    hist = []
    try:
        while True:
            try:
                v = ina.bus_voltage()
            except I2CError as e:
                print(f"I2C 错误: {e}", flush=True)
                time.sleep(0.5)
                continue
            hist.append(v)
            hist = hist[-8:]                      # 约 8 秒滑动平均, 平滑电机抖动
            v_avg = statistics.fmean(hist)
            pct = battery_percent(v_avg)
            render(oled, v_avg, pct)
            print(f"{v_avg:.3f}V  {pct:.0f}%", flush=True)
            if once:
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
