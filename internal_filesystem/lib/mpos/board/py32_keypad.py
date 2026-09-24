# lib/mpos/board/py32_keypad.py
#
# PY32F002A-TSSOP20 扩展按键板驱动（ESP32 侧 / MushOS 端）
#
# 硬件连接（参见 docs/HARDWARE_ARCHITECTURE.md）：
#   ESP32-S3  UART1   G01(TX)  ->  PY32  PA3(RX)   （PY32 UART1_TX=PA2, RX=PA3, AF1）
#                     G02(RX)  ->  PY32  PA2(TX)
#   PY32(TSSOP20) 外接：
#             4 路按键 K1..K4 = PB0 / PB2 / PB1 / PB3（上拉，低电平=按下）
#             电池电压  ADC_IN0 = PA0（200K/100K 分压 ÷3）；扩展模拟输入 ADC_IN1 = PA1(悬空预留)
#             状态 LED = PA6（高电平点亮）；芯片温度 = 内部通道 CH11
#             BOOT0 = PB6（固件绝不驱动）；SWD = PA13/PA14；NRST = PF2
#
# 协议（以 docs/MushPad_UART_Protocol.md **ICD v1.0 Rev B** 为准）：
#   主机查询：0xAA 0x01 0x20 CRC8            -> GET_STATUS(0x20)
#   从机应答：0xAA 0x06 0xA0 K(1B) VH(1B) VL(1B) TH(1B) TL(1B) CRC8  -> STATUS_DATA(0xA0)
#             K    = 按键位图 bit0..3 = K1..K4，1=按下（bit4..7 保留=0，主机不使用）
#             VH/VL= 电压(mV)，大端；**0xFFFF = 无效/未就绪/采样失败**（ICD §8.2）
#             TH/TL= 芯片温度(0.1℃，int16 大端，二进制补码)；**0x8000 = 无效**（ICD §8.3）
#
#   ⚠️ 注意 docs/COMMUNICATION_PROTOCOL.md 只是**总览稿**（那里写 LEN 1~254、
#      DATA 0~253），本文件按冻结 ICD 的 LEN 1~64 实现；两者冲突以 ICD 为准。
#
# 设计要点：
#   - 后台 asyncio 任务每 ~40ms 查询一次 PY32，把「按键位图 + 电压 + 温度」缓存到模块变量；
#   - LVGL indev 回调只读取缓存（非阻塞），避免 UART I/O 卡住 UI 主循环；
#   - 复用 odroid_go.py 的 KEYPAD 导航范式（focus_direction / back_screen）。
#
# 协议有效性处理（对齐 ICD，避免"看起来正常的错值"）：
#   - LEN 必须等于 1 + len(DATA)（ICD §4），各响应码的 DATA 长度见 ICD §7 表。
#     不符即视为不可信，**丢一个字节重新同步**，绝不按它去等 / 去索引。
#   - 电压 0xFFFF（§8.2）视为"无效"：**保持上一次有效值**，不用它覆盖缓存
#     （ICD §12-10 对 ADC 超时的规定同样是"返回上次有效值或无效值"）。
#   - 温度 0x8000（§8.3 / §12-11）视为"无效"：缓存置 None，get_temperature()
#     返回 None。返回 None 而不是 0.0 的意义是让调用方能区分"未知"与"真的 0℃"；
#     框架对未知温度的既有表现就是顶栏显示 "--°C"。
#
# 顶栏接入：本模块只提供**读取接口**，接线在板级文件里做 ——
#   internal_filesystem/lib/mpos/board/mushbot.py:
#       SensorManager.register_soc_temperature_sensor(
#           "PY32 Keypad Temperature", py32_keypad.get_temperature)
#   顶栏温度走 SensorManager.TYPE_SOC_TEMPERATURE（mpos/ui/topmenu.py），
#   注册后该来源即被替换为 PY32 上报的温度（否则是 ESP32 自己的 MCU 温度，
#   或板子没有 IMU/温感时的占位值）。
#   - 从未收到有效值时 get_voltage_mv() / get_temperature() 返回 None。电池桥接对
#     BatteryManager 抛 RuntimeError —— 框架本来就允许这条路径抛异常：见
#     battery_manager.read_raw_adc 文档里的 "Raises: RuntimeError"，以及
#     topmenu.update_battery_icon() 的 try/except（它会保持电量图标隐藏而不是显示错值）。
#
# 解析统计：py32_keypad.get_stats() 返回各计数（frames/crc_err/len_err/resync/
#   invalid_v/invalid_t），排查"按键没反应"时先看它 —— 能区分"从机没回"与"回了但被我丢了"。

import time
import machine
import lvgl as lv
import asyncio

import mpos.ui
from mpos import InputManager, TaskManager
import mpos.battery_manager as bm

# ============================ 配置 ============================
UART_ID = const(1)
TX_PIN = const(1)     # ESP32 G01
RX_PIN = const(2)     # ESP32 G02
BAUD = const(115200)

# 轮询周期与帧解析缓冲上限
QUERY_MS = const(40)
# 收到但还没解出来的字节攒在这里。校验过 LEN 之后（见 _RSP_DATA_LEN），本协议的
# 任一响应最多也就 9 字节（SYNC+LEN+CMD+5+DATA+CRC），32 字节余量充足；
# 设上限只是为了"从机一直发、我们又一直解不出"时缓冲不会无限增长。
PARSE_BUF_MAX = const(32)

# PY32 位图 bit -> LVGL 按键的映射（4 键菜单导航方案）
#   K1=上移焦点  K2=下移焦点  K3=返回(ESC)  K4=确认(ENTER)
KEY_MAP = {
    0: lv.KEY.UP,      # K1
    1: lv.KEY.DOWN,    # K2
    2: lv.KEY.ESC,     # K3
    3: lv.KEY.ENTER,   # K4
}

# 自动重复（与 odroid_go.py 一致）
REPEAT_INITIAL_DELAY_MS = const(300)
REPEAT_RATE_MS = const(100)

# ============================ 协议常量 ============================
SYNC = const(0xAA)
CMD_PING = const(0x01)
CMD_GET_KEYS = const(0x10)
CMD_GET_VOLTAGE = const(0x11)
CMD_GET_TEMP = const(0x12)
CMD_GET_STATUS = const(0x20)
RSP_PONG = const(0x81)
RSP_KEYS = const(0x90)
RSP_VOLTAGE = const(0x91)
RSP_TEMP = const(0x92)
RSP_STATUS = const(0xA0)
RSP_ERROR = const(0xFE)

# 每个响应码声明的 DATA 长度（ICD v1.0 Rev B §7 响应表）。
# 用途：校验 LEN == 1 + 该长度（ICD §4）。"LEN 与响应码对不上"的帧一律不可信 ——
# 它典型的后果就是解析器把 CRC 字节当成最后一个数据字节解出来（例如把 STATUS 的
# CRC 当成 T_L、把 KEYS 的 CRC 当成键位图），而 CRC 字节本身还恰好能"通过"Crc 校验。
_RSP_DATA_LEN = {
    RSP_PONG: 0,
    RSP_KEYS: 1,
    RSP_VOLTAGE: 2,
    RSP_TEMP: 2,
    RSP_STATUS: 5,
    RSP_ERROR: 1,
}

# 无效标记（ICD §8.2 电压 / §8.3 温度）
INVALID_VOLTAGE_MV = const(0xFFFF)
INVALID_TEMP_DC = const(0x8000)

# ============================ 共享状态 ============================
_key_state = 0              # 缓存的按键位图（由轮询任务更新）
_voltage_mv = None          # 缓存的电压值(mV)；None = 还没收到过有效值
_temperature_dC = None      # 缓存的芯片温度(0.1℃)；None = 无效 / 还没收到过
_uart = None
_repeat_next = None         # 自动重复计时

# 解析统计（排障用）：get_stats() 会返回它的副本。
# 有了它才能区分"从机没回"和"回了但被我丢了" —— 前者 frames 不涨且 resync/crc_err
# 也不涨，后者会有一个计数器在涨。
_stats = {
    "frames": 0,        # 通过 CRC 且已分发的响应帧
    "crc_err": 0,       # CRC 校验失败（丢 1 字节重同步）
    "len_err": 0,       # LEN 与 ICD §7 声明不符（丢 1 字节重同步）
    "resync": 0,        # 命令码不属于本机响应集 = 疑似噪声（丢 1 字节重同步）
    "invalid_v": 0,     # 收到电压无效标记 0xFFFF（保持上次有效值）
    "invalid_t": 0,     # 收到温度无效标记 0x8000（缓存置 None）
    "error_frames": 0,  # 收到从机的 ERROR 响应(0xFE)
}


# ---------------------- CRC8（poly 0x07） ----------------------
def _crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _make_frame(cmd: int, data: bytes = b"") -> bytes:
    payload = bytes([cmd]) + data
    frame = bytes([SYNC, len(payload)]) + payload
    return frame + bytes([_crc8(frame)])


def _decode_temp(hi: int, lo: int):
    """把两字节大端温度解成 0.1℃；无效标记 0x8000 返回 None。

    注意 0x8000 不是一个真值：ICD §8.3 明确定义它是"无效/传感器异常"，而 §12-11
    要求"主机识别为无效温度"。若直接按二进制补码符号扩展会得到 -3276.8℃ ——
    一个看起来像"极冷"的错误读数，比"未知"更糟。
    """
    v = (hi << 8) | lo
    if v == INVALID_TEMP_DC:
        _stats["invalid_t"] += 1
        return None
    if v >= 0x8000:
        v -= 0x10000
    return v


# ============================ 后台轮询任务 ============================
async def _poll_loop():
    """周期性查询 PY32，解析 STATUS_DATA 帧，更新缓存。非阻塞。"""
    global _key_state, _voltage_mv, _temperature_dC
    buf = bytearray()
    last_q = 0
    q_frame = _make_frame(CMD_GET_STATUS)

    while True:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_q) >= QUERY_MS:
            try:
                _uart.write(q_frame)
            except Exception:
                pass
            last_q = now

        avail = _uart.any()
        if avail:
            buf.extend(_uart.read(avail))
            if len(buf) > PARSE_BUF_MAX:
                del buf[:-PARSE_BUF_MAX]

            # 解析所有完整帧
            #
            # 循环不变式：每一轮只会「前进」（del buf[:i+1]）或「等更多字节」（break）。
            # break 只有在 i 处**确实是本机认识的响应码、且 LEN 与 ICD §7 声明一致**
            # 时才允许发生。旧实现用的是 `ln >= 5` 这类宽松判断，于是一个恰好含
            # 0xAA 的噪声字节对（例如 AA 40）会被当成"一个 67 字节的帧"而在原地一直
            # 等它凑齐；缓冲区上限又只有 32 字节，于是解析器**永久停住** —— 表现是
            # 按键/电压/温度全部静默冻结（ICD §12-6 明确要求对虚假 0xAA 自然容错）。
            i = buf.find(SYNC)
            while i >= 0 and len(buf) - i >= 3:
                ln = buf[i + 1]
                exp_data = _RSP_DATA_LEN.get(buf[i + 2])
                if exp_data is None or ln != 1 + exp_data:
                    # 不是「已知响应码 + ICD 声明的长度」-> 这个 0xAA 不可信。
                    # 丢 1 个字节重新同步，是唯一不会再次跑偏的做法。
                    if exp_data is None:
                        _stats["resync"] += 1
                    else:
                        _stats["len_err"] += 1
                    del buf[:i + 1]
                    i = buf.find(SYNC)
                    continue

                end = i + 2 + ln + 1  # SYNC + LEN + CMD + DATA + CRC
                if len(buf) < end:
                    break
                frame = buf[i:end]
                if _crc8(frame[:-1]) != frame[-1]:
                    _stats["crc_err"] += 1
                    del buf[:i + 1]      # 同样只丢 1 字节：从头找下一个真 SYNC
                    i = buf.find(SYNC)
                    continue

                _stats["frames"] += 1
                cmd = frame[2]
                if cmd == RSP_STATUS:
                    # 0xA0: K(1B) VHI(1B) VLO(1B) THI(1B) TLO(1B)
                    v = (frame[4] << 8) | frame[5]
                    if v == INVALID_VOLTAGE_MV:
                        _stats["invalid_v"] += 1     # 保留上次有效值（§12-10）
                    else:
                        _voltage_mv = v
                    _key_state = frame[3]
                    _temperature_dC = _decode_temp(frame[6], frame[7])
                elif cmd == RSP_TEMP:
                    _temperature_dC = _decode_temp(frame[3], frame[4])
                elif cmd == RSP_VOLTAGE:
                    v = (frame[3] << 8) | frame[4]
                    if v == INVALID_VOLTAGE_MV:
                        _stats["invalid_v"] += 1     # 保留上次有效值（§12-10）
                    else:
                        _voltage_mv = v
                elif cmd == RSP_KEYS:
                    _key_state = frame[3]
                elif cmd == RSP_ERROR:
                    _stats["error_frames"] += 1     # 从机报错，交给上层按需处理
                del buf[:end]
                i = buf.find(SYNC)

        await asyncio.sleep_ms(10)


# ============================ indev 回调 ============================
def _current_pressed_key():
    """从缓存位图中返回当前应触发的 LVGL 键（多键同按取优先级最高者）。"""
    for bit, lvkey in KEY_MAP.items():
        if _key_state & (1 << bit):
            return lvkey
    return None


def _input_cb(indev, data):
    global _repeat_next
    current = _current_pressed_key()

    if current is None:
        # 无按键：释放上一键
        if data.key:
            data.key = 0
            data.state = lv.INDEV_STATE.RELEASED
            _repeat_next = None
        return

    now = time.ticks_ms()
    repeat = _repeat_next is not None and now > _repeat_next

    if repeat or current != data.key:
        data.key = current
        data.state = lv.INDEV_STATE.PRESSED

        # 导航动作（参考 odroid_go.py）
        if current == lv.KEY.ESC:
            mpos.ui.back_screen()
        elif current == lv.KEY.RIGHT:
            mpos.ui.focus_direction.move_focus_direction(90)
        elif current == lv.KEY.LEFT:
            mpos.ui.focus_direction.move_focus_direction(270)
        elif current == lv.KEY.UP:
            mpos.ui.focus_direction.move_focus_direction(0)
        elif current == lv.KEY.DOWN:
            mpos.ui.focus_direction.move_focus_direction(180)
        # ENTER 交给 LVGL group 默认处理（激活聚焦对象）

        if not repeat:
            _repeat_next = now + REPEAT_INITIAL_DELAY_MS
        else:
            _repeat_next = now + REPEAT_RATE_MS
    else:
        # 同一键持续按住：保持 PRESSED，不重复触发导航
        data.key = current
        data.state = lv.INDEV_STATE.PRESSED


# ============================ 电池桥接 ============================
def _bridge_battery():
    """把 PY32 上报的电压(mV)桥接给 BatteryManager，复用系统电量 UI。

    适用于：PY32 的 ADC 接的是电池分压（本板 PA0，200K/100K，÷3）。若只是普通
    模拟量，请改用 get_voltage_mv() 自行读取，不要启用本桥接。

    ⚠️ 电压未就绪时 read() 抛 RuntimeError，而不是返回 0：
      - battery_manager.read_raw_adc() 会对 10 次 read() 求和，返回 0 会被算成
        "0% 电量"（看起来像"电池没电了"，而实际是"根本没读到"）；
      - 返回 None 会在求和时 TypeError。
    抛异常是框架**既有**的约定（见 battery_manager.read_raw_adc 文档里的
    "Raises: RuntimeError"），topmenu.update_battery_icon() 会捕获它并且
    **保持电量图标隐藏** —— 这正是"未知"应有的表现（15 s 一次，不会刷日志）。
    """
    class _UartAdc:
        def read(self):
            if _voltage_mv is None:
                raise RuntimeError("PY32 电压未就绪（尚无有效 STATUS/VOLTAGE 帧）")
            return _voltage_mv

    bm._adc = _UartAdc()
    bm._conversion_func = lambda raw: raw / 1000.0  # mV -> V
    bm._adc_pin = 99                                # 非 ADC2，避免 WiFi 协调
    bm._cached_raw_adc = None


# ============================ 对外接口 ============================
def get_voltage_mv():
    """返回最近一次**有效**的电压值(mV)；从未收到过有效值时返回 None。

    不要把 None 当成 0 使用：0 mV 会被上层读成"电池没电"，而 None 的含义是
    "不知道"。从机报 0xFFFF（ICD §8.2 无效标记）时本函数保持上一次有效值。
    """
    return _voltage_mv


def get_temperature():
    """返回最近一次有效的芯片温度(℃)；无效或尚未收到时返回 None。

    温度来自 PY32 内部 TS 通道，上报单位为 0.1℃(int16 大端)。ICD §8.3 规定
    0x8000 表示"无效/传感器异常"，§12-11 要求主机识别为无效 —— 此时返回 None。

    MushOS 顶栏就是通过本函数取温度的（板级文件里注册为 SOC 温度来源）：
        SensorManager.register_soc_temperature_sensor(
            "PY32 Keypad Temperature", py32_keypad.get_temperature)
    返回 None 而不是 0.0，是为了让顶栏能区分"未知"（显示 "--°C"）与"真的 0℃"。
    """
    if _temperature_dC is None:
        return None
    return _temperature_dC / 10.0


def get_temperature_raw():
    """返回最近一次有效芯片温度的原始值(0.1℃，int16)；无效时返回 None。"""
    return _temperature_dC


def get_key_state():
    """返回最近一次缓存的按键位图（bit0..3 = K1..K4，1=按下）。"""
    return _key_state


def get_stats():
    """返回解析统计的副本（排障用）：frames / crc_err / len_err / resync /
    invalid_v / invalid_t / error_frames。

    用途：把"按键没反应"拆成可判定的两类 ——
      - frames 不涨、其余计数也不涨  -> 从机没回（查线/固件/波特率）；
      - len_err / crc_err / resync 在涨 -> 从机回了但被本机丢弃（查帧格式）。
    """
    return dict(_stats)


def init(*, enable_battery=False):
    """初始化 UART、注册 KEYPAD indev、启动后台轮询。

    enable_battery：是否把 PY32 上报的电压桥接给 BatteryManager。
        —— PY32 固件已上报**真实**电池电压（÷3 还原，见 ICD §8.2），
           因此按键板接上分压时应当开启；从机不在线时电量图标会保持隐藏，
           不会显示假电量（原理见 _bridge_battery 的说明）。
    """
    global _uart
    _uart = machine.UART(
        UART_ID, baudrate=BAUD, tx=TX_PIN, rx=RX_PIN, timeout=20
    )
    # 清空可能的上电垃圾数据
    try:
        _uart.read(_uart.any())
    except Exception:
        pass

    indev = lv.indev_create()
    indev.set_type(lv.INDEV_TYPE.KEYPAD)
    indev.set_read_cb(_input_cb)
    indev.set_group(lv.group_get_default())
    indev.set_display(lv.display_get_default())
    indev.enable(True)
    InputManager.register_indev(indev)

    if enable_battery:
        _bridge_battery()

    # 在 TaskManager 启动前注册后台任务（框架支持此用法，见 main.py）
    TaskManager.create_task(_poll_loop())
