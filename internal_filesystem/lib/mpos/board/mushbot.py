# internal_filesystem/lib/mpos/board/mushbot.py
#
# Hardware initialization for MushBot — a custom ESP32-S3 dev board running MushOS.
# MCU: ESP32-S3 (ESP32S3R8: 8MB Octal PSRAM), 8MB Flash
# Display: ST7789, 320x240 (横向使用; 面板原生 240x320), SPI
#
# >>> 把下面的引脚常量改成你开发板原理图上的实际连线 <<<
# 改完用 REPL (CTRL-E) 手动试，确认屏幕亮、颜色对，再固化到此文件。

import logging

logger = logging.getLogger(__name__)

if __debug__:
    logger.debug("mushbot.py initialization")

import time

import machine
import lcd_bus
import lvgl as lv
import drivers.display.st7789 as st7789
import mpos.ui
from machine import Pin
from micropython import const

# ===================== 显示 SPI 引脚（来自开发板原理图） =====================
SPI_BUS = const(1)        # FSPI (SPI2_HOST)。Arduino_GFX 默认 FSPI，freenove 官方板也用 host=1
SPI_FREQ = const(40000000)
PIN_SCLK = const(38)      # SCK
PIN_MOSI = const(39)      # MOSI / SDA
PIN_MISO = const(48)      # MISO（屏未接，占位空闲脚）
PIN_DC   = const(45)      # D/C
PIN_CS   = const(21)      # CS（若硬件已拉低接地，可设 -1）
PIN_RST  = const(40)      # 复位（若接到 EN / 3.3V，可设 -1）
PIN_BL   = const(46)      # 背光（普通 GPIO 或由 PWM 驱动）

LCD_SCLK = PIN_SCLK
LCD_MOSI = PIN_MOSI
LCD_MISO = PIN_MISO
LCD_DC   = PIN_DC
LCD_CS   = PIN_CS
LCD_RST  = PIN_RST
LCD_BL   = PIN_BL

# ===================== 分辨率 =====================
# 面板: ST7789，可视区 320x240（横向使用）。
#
# ⚠️ ST7789 的显存是**固定**的 240 列 x 320 行，所以"320x240"是**横向**用法，
#    必须分两步做，不能直接把 display_width 写成 320：
#      ① 按原生方向创建驱动 —— display_width=240, display_height=320；
#      ② 再用 set_rotation() 让 LVGL 把横向/纵向分辨率对调 → 逻辑 320x240。
#    若反过来写 display_width=320 / display_height=240，超出行/列范围的像素
#    会被静默丢弃（表现为画面右侧或底部被裁掉、错位），而不是报错。
#
#    参考同仓库的 ST7789 320x240 板：matouch_esp32_s3_spi_ips_2_8_*.py
#    （它按原生 240x320 创建，用竖屏；本板要横屏，所以额外加一次旋转）
TFT_NATIVE_WIDTH  = const(240)   # ST7789 原生列数（硬件固定，不要改）
TFT_NATIVE_HEIGHT = const(320)   # ST7789 原生行数（硬件固定，不要改）

# 逻辑方向: 目标 320x240 横向
#   · 画面上下/左右颠倒  -> 把 _90 改成 _270
#   · 画面变成 240x320 竖屏 -> 这行没生效（set_rotation 必须在 init() 之后调用）
#   · 只要竖屏 240x320      -> 直接删掉下面那次 set_rotation() 调用即可
DISPLAY_ROTATION = lv.DISPLAY_ROTATION._90

TFT_WIDTH  = TFT_NATIVE_WIDTH     # 传给驱动的宽度 = 原生列数
TFT_HEIGHT = TFT_NATIVE_HEIGHT    # 传给驱动的高度 = 原生行数

# ===================== Step 1: SPI 总线 + 显示总线 =====================
if __debug__:
    logger.debug("mushbot.py: init SPI display")

try:
    spi_bus = machine.SPI.Bus(host=SPI_BUS, mosi=LCD_MOSI, miso=LCD_MISO, sck=LCD_SCLK)
except Exception as e:
    logger.error("Error initializing SPI bus: %s" % (e))
    if __debug__:
        logger.debug("Attempting hard reset in 3 sec...")
    time.sleep(3)
    machine.reset()

display_bus = lcd_bus.SPIBus(
    spi_bus=spi_bus,
    freq=SPI_FREQ,
    dc=LCD_DC,
    cs=LCD_CS,
    spi_mode=3,  # ST7789 用 SPI mode 3（CPOL=1,CPHA=1），匹配 Arduino_GFX 的 SPI_MODE3
)

# 局部刷新缓冲: 320(逻辑宽) * 30 行 * 2 字节 = 19200 B
#   注意 19200 同时整除 240*2（=40 原生行）与 320*2（=30 逻辑行），
#   所以旋转前后都不需要改这个数字；一帧完整画面 320*240*2 = 153600 B，
#   双缓冲不可能整屏放下（内部 RAM 不够），必须用局部缓冲。
_BUFFER_SIZE = const(320 * 30 * 2)  # 19200
fb1 = display_bus.allocate_framebuffer(_BUFFER_SIZE, lcd_bus.MEMORY_INTERNAL | lcd_bus.MEMORY_DMA)
fb2 = display_bus.allocate_framebuffer(_BUFFER_SIZE, lcd_bus.MEMORY_INTERNAL | lcd_bus.MEMORY_DMA)

mpos.ui.main_display = st7789.ST7789(
    data_bus=display_bus,
    frame_buffer1=fb1,
    frame_buffer2=fb2,
    display_width=TFT_WIDTH,
    display_height=TFT_HEIGHT,
    color_space=lv.COLOR_FORMAT.RGB565,
    color_byte_order=st7789.BYTE_ORDER_BGR,   # 颜色不对就试 st7789.BYTE_ORDER_RGB
    rgb565_byte_swap=True,                    # 颜色不对就改成 False
    backlight_pin=LCD_BL,
    backlight_on_state=st7789.STATE_PWM,
    reset_pin=LCD_RST,
    reset_state=0,                            # 屏全黑但引脚没错，就试 1
)

# 硬件复位脉冲：lvgl 的 init 只发软件 SWRESET，这里补 RST 拉低→拉高复位，
# 匹配 Arduino_GFX tftInit() 的复位时序（HIGH→LOW→HIGH）
_rst = machine.Pin(LCD_RST, machine.Pin.OUT)
_rst.value(1)              # 先释放（高）
time.sleep_ms(100)
_rst.value(0)              # 拉低复位（reset_state=0，低有效）
time.sleep_ms(120)
_rst.value(1)              # 释放复位
time.sleep_ms(120)

mpos.ui.main_display.init()
mpos.ui.main_display.set_power(True)
mpos.ui.main_display.set_backlight(100)

# 方向 / 镜像: 旋转成 320x240 横向
#   ⚠️ 必须在 init() 之后调用 —— set_rotation() 通过 _on_size_change 回调
#      重算 MADCTL，驱动未初始化时这次写不会生效（画面会停在竖屏 240x320）。
mpos.ui.main_display.set_rotation(DISPLAY_ROTATION)

# 若旋转对了但颜色/镜像仍不对，再手工调 MADCTL（0x36）。常用位：
#   0x00 正常 | 0x60 MV|MX | 0xC0 MY|MX | 0xA0 MV|MY
# mpos.ui.main_display.set_params(0x36, bytearray([0x60]))

# ===================== Step 2: 输入设备（可选，按需打开） =====================
# --- 电阻/电容触摸示例（CST816S）---
# import i2c
# from mpos import InputManager
# import drivers.indev.cst816s as cst816s
# i2c_bus = i2c.I2C.Bus(host=0, scl=48, sda=47, freq=400000, use_locks=False)
# touch_dev = i2c.I2C.Device(bus=i2c_bus, dev_id=0x15, reg_bits=8)
# indev = cst816s.CST816S(touch_dev)
# InputManager.register_indev(indev)

# --- PY32F002A 扩展按键板（UART1: G01/G02）---
# 4 键菜单导航 + 电池电压(÷3) + 芯片温度。详见 docs/MushOS-按键扩展方案.md
# 协议以 docs/MushPad_UART_Protocol.md（ICD v1.0 Rev B，冻结）为准。
# PY32 未烧录/未接线时不影响显示：按键位图保持 0，电量图标保持隐藏
# （电压未就绪时 _bridge_battery 抛异常，topmenu 会接住并跳过本次更新）。
from mpos.board import py32_keypad
py32_keypad.init(enable_battery=True)

# 把 PY32 的芯片温度接进系统顶栏：顶栏温度走 SensorManager.TYPE_SOC_TEMPERATURE
# （见 mpos/ui/topmenu.py），这里把该来源换成 PY32 上报的温度。
# 温度无效(ICD §8.3 的 0x8000)或尚未就绪时 get_temperature() 返回 None ->
# 顶栏显示 "--°C"；不会去显示 ESP32 自己的 MCU 温度，也不会显示占位值。
from mpos import SensorManager
SensorManager.register_soc_temperature_sensor(
    "PY32 Keypad Temperature", py32_keypad.get_temperature
)

# --- SmartPort PY32F002A-SOP8（I2C1: G04/G05）---
# 电机 PWM / ADC / GPIO 扩展端口。详见 docs/MushOS-SmartPort_I2C与IAP.md
# 协议兼容 docs/COMMUNICATION_PROTOCOL.md 第二部分。无端口时不影响系统。
from mpos.board import py32_smartport
smartport_manager = py32_smartport.init(start_poll=True)

if __debug__:
    logger.debug("mushbot.py finished")
