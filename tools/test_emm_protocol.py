"""EmmStepper 协议帧主机侧测试 —— 对拍《Emm42_V5.0 步进闭环驱动说明书》命令例帧。

原理: 从 k230/main.py 中用 AST 提取 `EMM_CHECKSUM` 常量与 `EmmStepper` 类源码,
在 stub 掉 machine(UART/FPIOA) 与 time 的环境里执行, 捕获实际发出的字节流,
逐帧与说明书例帧对拍 —— 防止字节序/字段序写错(驱动器对错误帧会直接丢弃或误动)。

运行:  python tools/test_emm_protocol.py
说明:  纯主机侧运行, 不需要 K230 硬件; 不依赖任何第三方库(stdlib only)。
"""
import ast
import os
import sys
import time as _time
import types
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))


# ---------------------------------------------------------------------------
# 从 main.py 提取 EMM_CHECKSUM + EmmStepper, 并在 stub 环境里执行
# ---------------------------------------------------------------------------

def _extract_parts(src):
    """返回 (const_src, class_src); 找不到时抛 AssertionError"""
    tree = ast.parse(src)
    const_src = class_src = None
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
            if "EMM_CHECKSUM" in names:
                const_src = ast.get_source_segment(src, node)
        if isinstance(node, ast.ClassDef) and node.name == "EmmStepper":
            class_src = ast.get_source_segment(src, node)
    assert const_src is not None, "k230/main.py 中未找到 EMM_CHECKSUM 常量"
    assert class_src is not None, "k230/main.py 中未找到 class EmmStepper"
    return const_src, class_src


class _FakeTime(types.ModuleType):
    """stub MicroPython 的 time 接口"""
    def __init__(self):
        super().__init__("time")

    def ticks_ms(self):
        return int(_time.monotonic() * 1000)

    def ticks_diff(self, a, b):
        return a - b

    def sleep_ms(self, ms):
        _time.sleep(ms / 1000.0)


class FakeUART:
    """记录 write 字节流; RX 默认空(离线设备)。

    response: 预设"下一帧发送后注入 RX"的应答字节(模拟驱动器回包时序:
    实际驱动器是在收到请求帧后才应答, 因此方法开头的清残留 read() 不会吞掉它)。
    """
    def __init__(self, uart_id, baudrate=115200):
        self.uart_id = uart_id
        self.baudrate = baudrate
        self.rx = bytearray()
        self.response = None
        self.writes = []

    def write(self, b):
        self.writes.append(bytes(b))
        if self.response is not None:
            self.rx += self.response
            self.response = None
        return len(b)

    def any(self):
        return len(self.rx)

    def read(self, n=None):
        if n is None:
            d = bytes(self.rx)
            self.rx = bytearray()
            return d
        d = bytes(self.rx[:n])
        del self.rx[:n]
        return d


class FakeFPIOA:
    def set_function(self, pin, fn):
        self.last_map = (pin, fn)


for _u in range(5):
    setattr(FakeFPIOA, "UART%d_TXD" % _u, "UART%d_TXD" % _u)
    setattr(FakeFPIOA, "UART%d_RXD" % _u, "UART%d_RXD" % _u)


def _install_stubs():
    machine = types.ModuleType("machine")
    machine.UART = FakeUART
    machine.FPIOA = FakeFPIOA
    sys.modules["machine"] = machine


def load_emm_class():
    """加载并返回 main.py 里的 EmmStepper 类"""
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    const_src, class_src = _extract_parts(src)
    _install_stubs()
    env = {"time": _FakeTime()}
    exec(const_src, env)
    exec(class_src, env)
    return env["EmmStepper"]


def make_stepper():
    EmmStepper = load_emm_class()
    # timeout_ms 取小值, 让离线探测快速超时
    return EmmStepper(uart_id=2, tx_pin=5, rx_pin=6, addr=1, acc=200, timeout_ms=2)


# ---------------------------------------------------------------------------
# 说明书例帧对拍 + 位置模式接口断言
# ---------------------------------------------------------------------------

class TestEmmFrames(unittest.TestCase):
    def setUp(self):
        self.st = make_stepper()

    def last(self):
        self.assertTrue(self.st.uart.writes, "没有发出任何帧")
        return self.st.uart.writes[-1]

    # ---- 与说明书例帧逐字节对拍 ----

    def test_enable_frame(self):
        # 说明书: 发送 01 F3 AB 01 00 6B (使能, 不同步)
        self.st.enable(True)
        self.assertEqual(self.last(), bytes.fromhex("01 F3 AB 01 00 6B"))

    def test_disable_frame(self):
        # 说明书: 不使能发送 01 F3 AB 00 00 6B
        self.st.enable(False)
        self.assertEqual(self.last(), bytes.fromhex("01 F3 AB 00 00 6B"))

    def test_stop_frame(self):
        # 说明书: 发送 01 FE 98 00 6B (立即停止)
        self.st.stop()
        self.assertEqual(self.last(), bytes.fromhex("01 FE 98 00 6B"))

    def test_speed_frame(self):
        # 说明书: 发送 01 F6 01 05 DC 0A 00 6B
        # (方向=CCW, 速度 0x05DC=1500RPM, 加速度=10, 不同步)
        self.st.set_speed(1, 1500, acc=10)
        self.assertEqual(self.last(), bytes.fromhex("01 F6 01 05 DC 0A 00 6B"))

    def test_voltage_probe_frame(self):
        # 说明书: 发送 01 24 6B; 离线时返回 None
        v = self.st.read_bus_voltage()
        self.assertEqual(self.last(), bytes.fromhex("01 24 6B"))
        self.assertIsNone(v)

    # ---- 位置模式(备用接口 —— 当前控制流两轴均用速度模式; 此处对拍说明书例帧) ----

    def test_move_relative_matches_manual(self):
        # 说明书位置模式例帧: 01 FD 01 05 DC 00 00 00 7D 00 00 00 6B
        # (CCW, 1500RPM, acc=0, 32000 脉冲, 相对模式, 不同步)
        self.st.move(32000, 1500, acc=0, absolute=False)
        self.assertEqual(self.last(), bytes.fromhex("01 FD 01 05 DC 00 00 00 7D 00 00 00 6B"))

    def test_move_absolute_positive(self):
        # 绝对模式, +800 脉冲(0x320), 1200RPM(0x4B0), acc=200(0xC8)
        self.st.move(800, 1200, acc=200, absolute=True)
        self.assertEqual(self.last(), bytes.fromhex("01 FD 01 04 B0 C8 00 00 03 20 01 00 6B"))

    def test_move_absolute_negative_twos_complement(self):
        # 负目标 -800 脉冲 → 32 位补码 FF FF FC E0; 方向位=00
        self.st.move(-800, 1200, acc=200, absolute=True)
        self.assertEqual(self.last(), bytes.fromhex("01 FD 00 04 B0 C8 FF FF FC E0 01 00 6B"))

    def test_clear_position_frame(self):
        # 说明书: 发送 01 0A 6D 6B (当前位置角度清零) —— 上电 0 点
        self.st.clear_position()
        self.assertEqual(self.last(), bytes.fromhex("01 0A 6D 6B"))

    def test_home_frame(self):
        # 说明书: 01 9A 00 00 6B (单圈就近回零) / 01 9A 02 00 6B (多圈无限位碰撞回零)
        self.st.home(0)
        self.assertEqual(self.last(), bytes.fromhex("01 9A 00 00 6B"))
        self.st.home(2)
        self.assertEqual(self.last(), bytes.fromhex("01 9A 02 00 6B"))

    # ---- 通用不变量 ----

    def test_all_frames_checksum_always_6b(self):
        self.st.enable(True)
        self.st.stop()
        self.st.set_speed(1, 1500, acc=10)
        self.st.move(800, 1200, acc=200, absolute=True)
        self.st.move(-800, 1200, acc=200, absolute=True)
        self.st.clear_position()
        self.st.home(0)
        for w in self.st.uart.writes:
            self.assertEqual(w[-1], 0x6B, "帧 %s 校验字节不是 0x6B" % w.hex())
            self.assertEqual(w[0], 1, "帧 %s 地址字节不是 1" % w.hex())

    def test_move_clamps_rpm_and_acc(self):
        # 速度/加速度超界时按协议限幅(0~3000 / 0~255), 不应抛错
        self.st.move(100, 99999, acc=999, absolute=True)
        w = self.last()
        self.assertEqual(w[3:5], bytes.fromhex("0B B8"))  # 3000 = 0x0BB8
        self.assertEqual(w[5], 0xFF)


class TestEmmReadMethods(unittest.TestCase):
    """V8 读取类命令(转速/位置) + 清堵转:

    命令码来源: 张大头官方例程 X_V2.c 读取命令表(0x35 实时转速 / 0x36 实时位置 /
    0x0E 0x52 清堵转), 与读电压 0x24 同构(帧式 [addr][func][0x6B], 已实机验证)。
    响应字节段为**预期格式**, 首次上机须先 raw_probe(0x35/0x36) 捕获核对——
    本组用例保证: 请求帧正确 + 解析按预期格式正确 + 脏数据(校验错/短包/错地址)不误解析。
    """

    def setUp(self):
        self.st = make_stepper()

    def last(self):
        self.assertTrue(self.st.uart.writes, "没有发出任何帧")
        return self.st.uart.writes[-1]

    # ---- 请求帧(命令码 + 帧结构) ----

    def test_read_velocity_request_frame(self):
        v = self.st.read_velocity()
        self.assertEqual(self.last(), bytes.fromhex("01 35 6B"))
        self.assertIsNone(v)                        # 离线无应答

    def test_read_position_request_frame(self):
        p = self.st.read_position()
        self.assertEqual(self.last(), bytes.fromhex("01 36 6B"))
        self.assertIsNone(p)

    def test_reset_clog_frame(self):
        self.st.reset_clog()
        self.assertEqual(self.last(), bytes.fromhex("01 0E 52 6B"))

    # ---- 响应解析(模拟驱动器回包时序; 预期格式) ----

    def test_read_velocity_positive(self):
        self.st.uart.response = bytes.fromhex("01 35 00 03 E8 6B")
        self.assertEqual(self.st.read_velocity(), 1000)

    def test_read_velocity_negative(self):
        self.st.uart.response = bytes.fromhex("01 35 01 00 64 6B")
        self.assertEqual(self.st.read_velocity(), -100)

    def test_read_position_positive(self):
        self.st.uart.response = bytes.fromhex("01 36 00 00 00 03 E8 6B")
        self.assertEqual(self.st.read_position(), 1000)

    def test_read_position_negative(self):
        self.st.uart.response = bytes.fromhex("01 36 01 00 00 00 64 6B")
        self.assertEqual(self.st.read_position(), -100)

    def test_read_position_high_bytes_big_endian(self):
        # 高字节在前: 0x12345678
        self.st.uart.response = bytes.fromhex("01 36 00 12 34 56 78 6B")
        self.assertEqual(self.st.read_position(), 0x12345678)

    # ---- 鲁棒性: 脏数据不得误解析 ----

    def test_bad_checksum_returns_none(self):
        self.st.uart.response = bytes.fromhex("01 35 00 03 E8 00")
        self.assertIsNone(self.st.read_velocity())

    def test_short_reply_returns_none(self):
        self.st.uart.response = bytes.fromhex("01 35 00")
        self.assertIsNone(self.st.read_velocity())

    def test_wrong_addr_returns_none(self):
        self.st.uart.response = bytes.fromhex("02 35 00 03 E8 6B")
        self.assertIsNone(self.st.read_velocity())

    # ---- last_rpm 命令记录(堵转监控配套) ----

    def test_last_rpm_tracks_speed_cmd(self):
        self.assertEqual(self.st.last_rpm, 0)       # 初始 0
        self.st.set_speed(1, 30)
        self.assertEqual(self.st.last_rpm, 30)
        self.st.set_speed(-1, 20)
        self.assertEqual(self.st.last_rpm, -20)
        self.st.stop()
        self.assertEqual(self.st.last_rpm, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
