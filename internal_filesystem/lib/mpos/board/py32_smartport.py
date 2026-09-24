# lib/mpos/board/py32_smartport.py
#
# SmartPort 驱动（ESP32 侧 / MushOS 端）—— PY32F002A-SOP8 智能端口
#
# 硬件连接（参见 docs/HARDWARE_ARCHITECTURE.md）：
#   ESP32-S3  I2C1   G04(SCL) <->  PY32  PA6 (I2C_SCL)
#                     G05(SDA) <->  PY32  PA5 (I2C_SDA)
#   外部 4.7kΩ 上拉至 3.3V（I2C 总线必需）
#   每个端口通过 PA7(ADDR_SEL) 分压设定地址 0x10/0x11/0x12/0x13
#
# 协议（完全兼容 docs/COMMUNICATION_PROTOCOL.md 第二部分 I2C 寄存器映射）：
#   - 寄存器地址 8-bit，多字节寄存器采用「大端(MSB 先)」与 PY32 固件保持一致
#   - 0x00 DEVICE_ID / 0x02 FW_VERSION / 0x04 STATUS / 0x05 CONFIG
#   - 0x06 PWM0_DUTY / 0x07 PWM1_DUTY / 0x08 PWM0_FREQ / 0x0A PWM1_FREQ
#   - 0x0C ADC_VALUE / 0x0E GPIO_OUT / 0x0F GPIO_DIR / 0x10 INTERRUPT
#
# IAP（固件更新）相关寄存器为「扩展设计」，需 PY32 固件配合实现，
# 见 docs/MushOS-SmartPort_I2C与IAP.md。

import i2c

from mpos import TaskManager

# ============================ 配置 ============================
I2C_HOST = const(1)
SCL_PIN = const(4)      # ESP32 G04
SDA_PIN = const(5)      # ESP32 G05
I2C_FREQ = const(400000)

PORT_ADDRS = (0x10, 0x11, 0x12, 0x13)
EXPECTED_DEVICE_ID = const(0x3202)

# ============================ 寄存器地址 ============================
REG_DEVICE_ID = const(0x00)
REG_FW_VERSION = const(0x02)
REG_STATUS = const(0x04)
REG_CONFIG = const(0x05)
REG_PWM0_DUTY = const(0x06)
REG_PWM1_DUTY = const(0x07)
REG_PWM0_FREQ = const(0x08)
REG_PWM1_FREQ = const(0x0A)
REG_ADC_VALUE = const(0x0C)
REG_GPIO_OUT = const(0x0E)
REG_GPIO_DIR = const(0x0F)
REG_INTERRUPT = const(0x10)

# ---- IAP 扩展寄存器（设计稿，需固件支持）----
REG_IAP_CTRL = const(0x20)   # 1B: 命令触发
REG_IAP_ADDR = const(0x21)   # 4B: 页地址（大端）
REG_IAP_DATA = const(0x25)   # 1..64B: 页数据
REG_IAP_CRC = const(0x60)    # 4B: 整包 CRC32（大端）
REG_IAP_STAT = const(0x64)   # 1B: 引导状态

IAP_CMD_ENTER = const(0x01)
IAP_CMD_ERASE = const(0x02)
IAP_CMD_WRITE = const(0x03)
IAP_CMD_VERIFY = const(0x04)
IAP_CMD_COMMIT = const(0x05)   # 验收通过 + 跳转 App
IAP_CMD_ABORT = const(0xFF)

PAGE_SIZE = const(64)         # 与固件约定的写页大小


# ============================ 单个端口 ============================
class SmartPort:
    """代表一个已发现的 SmartPort 从机。"""

    def __init__(self, dev, addr):
        self.addr = addr
        self._dev = dev
        self.device_id = self._rd_u16(REG_DEVICE_ID)
        self.fw_version = self._rd_u16(REG_FW_VERSION)

    # ---------- 底层读写（带容错）----------
    def _rd(self, reg, n):
        try:
            return self._dev.read_mem(reg, n)
        except OSError:
            return None

    def _rd_u16(self, reg):
        b = self._rd(reg, 2)
        if not b or len(b) < 2:
            return None
        return (b[0] << 8) | b[1]   # 大端(MSB 先)

    def _rd_u8(self, reg):
        b = self._rd(reg, 1)
        return b[0] if b else None

    def _wr(self, reg, data):
        try:
            self._dev.write_mem(reg, data)
            return True
        except OSError:
            return False

    # ---------- 业务 API ----------
    def get_status(self):
        return self._rd_u8(REG_STATUS)

    def get_adc(self):
        """返回 12-bit ADC 原始值。"""
        return self._rd_u16(REG_ADC_VALUE)

    def set_pwm_duty(self, ch, duty):
        reg = REG_PWM0_DUTY if ch == 0 else REG_PWM1_DUTY
        return self._wr(reg, bytes([duty & 0xFF]))

    def get_pwm_duty(self, ch):
        reg = REG_PWM0_DUTY if ch == 0 else REG_PWM1_DUTY
        return self._rd_u8(reg)

    def set_pwm_freq(self, ch, hz):
        reg = REG_PWM0_FREQ if ch == 0 else REG_PWM1_FREQ
        # 大端 2 字节
        return self._wr(reg, bytes([(hz >> 8) & 0xFF, hz & 0xFF]))

    def get_pwm_freq(self, ch):
        reg = REG_PWM0_FREQ if ch == 0 else REG_PWM1_FREQ
        return self._rd_u16(reg)

    def set_gpio_out(self, val):
        return self._wr(REG_GPIO_OUT, bytes([val & 0xFF]))

    def get_gpio_out(self):
        return self._rd_u8(REG_GPIO_OUT)

    def set_gpio_dir(self, val):
        """bit: 0=输入 1=输出。"""
        return self._wr(REG_GPIO_DIR, bytes([val & 0xFF]))

    def config(self, val):
        """bit7 IRQ_EN / bit3-2 GPIO_MD / bit1-0 PWM_MD。"""
        return self._wr(REG_CONFIG, bytes([val & 0xFF]))

    # ---------- IAP 命令（需固件支持，见文档）----------
    def iap_enter(self):
        return self._wr(REG_IAP_CTRL, bytes([IAP_CMD_ENTER]))

    def iap_erase_page(self, addr):
        if not self._wr(REG_IAP_ADDR, addr.to_bytes(4, "big")):
            return False
        return self._wr(REG_IAP_CTRL, bytes([IAP_CMD_ERASE]))

    def iap_write_page(self, addr, data):
        if len(data) > PAGE_SIZE:
            return False
        if not self._wr(REG_IAP_ADDR, addr.to_bytes(4, "big")):
            return False
        if not self._wr(REG_IAP_DATA, bytes(data)):
            return False
        return self._wr(REG_IAP_CTRL, bytes([IAP_CMD_WRITE]))

    def iap_set_crc(self, crc32):
        return self._wr(REG_IAP_CRC, crc32.to_bytes(4, "big"))

    def iap_verify(self):
        return self._wr(REG_IAP_CTRL, bytes([IAP_CMD_VERIFY]))

    def iap_commit(self):
        """验收通过并跳转到 App。"""
        return self._wr(REG_IAP_CTRL, bytes([IAP_CMD_COMMIT]))

    def iap_abort(self):
        return self._wr(REG_IAP_CTRL, bytes([IAP_CMD_ABORT]))

    def iap_status(self):
        return self._rd_u8(REG_IAP_STAT)

    def __repr__(self):
        return "SmartPort(addr=0x%02x, id=%s, fw=%s)" % (
            self.addr, self.device_id, self.fw_version
        )


# ============================ 管理器 ============================
class SmartPortManager:
    """负责 I2C 总线扫描、端口发现、热插拔轮询与统一访问。"""

    def __init__(self):
        self.bus = i2c.I2C.Bus(
            host=I2C_HOST, scl=SCL_PIN, sda=SDA_PIN, freq=I2C_FREQ
        )
        self.ports = {}   # addr -> SmartPort

    def scan(self):
        """扫描 0x10~0x13，发现/移除端口。返回当前在线端口字典。"""
        try:
            found = self.bus.scan()
        except OSError:
            found = []

        for addr in PORT_ADDRS:
            if addr in found:
                if addr not in self.ports:
                    dev = i2c.I2C.Device(self.bus, addr, reg_bits=8)
                    sp = SmartPort(dev, addr)
                    if sp.device_id == EXPECTED_DEVICE_ID:
                        self.ports[addr] = sp
                    else:
                        # 地址响应但不是预期设备（罕见），忽略
                        pass
            else:
                self.ports.pop(addr, None)
        return self.ports

    def poll(self):
        """轻量轮询：重新扫描并刷新各端口状态（可由后台任务周期调用）。"""
        self.scan()
        for sp in self.ports.values():
            sp.get_status()
        return self.ports

    async def poll_loop(self, interval_ms=2000):
        while True:
            self.poll()
            await TaskManager.sleep_ms(interval_ms)

    # ---------- IAP 编排（需固件支持）----------
    def iap_update(self, addr, firmware, page_size=PAGE_SIZE):
        """把一个固件镜像(字节串)通过 I2C 写入指定端口。

        流程：enter -> 逐页 erase+write -> 设置 CRC -> verify -> commit。
        要求 PY32 固件实现了 REG_IAP_* 引导逻辑。返回 True/False。
        """
        sp = self.ports.get(addr)
        if sp is None:
            return False

        if not sp.iap_enter():
            return False

        # CRC32（与固件约定一致，这里用 zlib；固件侧需同算法）
        import binascii
        crc = binascii.crc32(firmware) & 0xFFFFFFFF

        for off in range(0, len(firmware), page_size):
            page = firmware[off:off + page_size]
            if len(page) < page_size:
                page = page + b"\xff" * (page_size - len(page))
            if not sp.iap_erase_page(0x08000000 + off):
                sp.iap_abort()
                return False
            if not sp.iap_write_page(0x08000000 + off, page):
                sp.iap_abort()
                return False

        sp.iap_set_crc(crc)
        sp.iap_verify()
        sp.iap_commit()
        return True


# ============================ 对外初始化 ============================
_manager = None


def get_manager():
    global _manager
    if _manager is None:
        _manager = SmartPortManager()
        _manager.scan()
    return _manager


def init(*, start_poll=True):
    """初始化 SmartPort 管理器并（可选）启动后台热插拔轮询。"""
    mgr = get_manager()
    if start_poll:
        TaskManager.create_task(mgr.poll_loop())
    return mgr
