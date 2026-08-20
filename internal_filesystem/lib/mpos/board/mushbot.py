# internal_filesystem/lib/mpos/board/mushbot.py
#
# Hardware initialization for MushBot — a custom ESP32-S3 dev board running MushOS.
# MCU: ESP32-S3 (ESP32S3R8: 8MB Octal PSRAM), 8MB Flash
# Display: ST7789, 240x240, SPI
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

# 分辨率
TFT_WIDTH = const(240)
TFT_HEIGHT = const(240)

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

# 一帧 240*240*2 = 115200 字节；双缓冲各取一部分，按可用内存微调
_BUFFER_SIZE = const(240 * 40 * 2)  # 19200
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

# 方向 / 镜像：若画面方向不对，调整旋转或 MADCTL 位
# mpos.ui.main_display.set_rotation(lv.DISPLAY_ROTATION._0)
# mpos.ui.main_display.set_params(0x36, bytearray([0x00]))

# ===================== Step 2: 输入设备（可选，按需打开） =====================
# --- 电阻/电容触摸示例（CST816S）---
# import i2c
# from mpos import InputManager
# import drivers.indev.cst816s as cst816s
# i2c_bus = i2c.I2C.Bus(host=0, scl=48, sda=47, freq=400000, use_locks=False)
# touch_dev = i2c.I2C.Device(bus=i2c_bus, dev_id=0x15, reg_bits=8)
# indev = cst816s.CST816S(touch_dev)
# InputManager.register_indev(indev)

# --- 物理按键示例（KEYPAD）---
# from mpos import InputManager
# btn = Pin(0, Pin.IN, Pin.PULL_UP)
# def keypad_read_cb(indev, data):
#     data.key = lv.KEY.ENTER
#     data.state = lv.INDEV_STATE.PRESSED if btn.value() == 0 else lv.INDEV_STATE.RELEASED
# indev = lv.indev_create()
# indev.set_type(lv.INDEV_TYPE.KEYPAD)
# indev.set_read_cb(keypad_read_cb)
# indev.set_group(lv.group_get_default())
# indev.set_display(lv.display_get_default())
# indev.enable(True)
# InputManager.register_indev(indev)

if __debug__:
    logger.debug("mushbot.py finished")
