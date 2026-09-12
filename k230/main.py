"""
K230 物体追踪 — 双轴云台
========================
  左右(yaw)轴:  Emm42_V5.0 闭环步进驱动器  (UART3, 串口速度模式)
  上下(pitch)轴: FashionStar FSUS 总线舵机 (UART2, 增量位置控制)

═══════════════════════════════════════════════════════════════
  功能说明
═══════════════════════════════════════════════════════════════
  摄像头采集 → 物体检测 → 算目标中心与屏幕中心的偏移 → 驱动云台

  检测方式 (config.json → detection.mode):
    cv    : 矩形靶标检测(白心+黑边验证)   偏移 = 四边形中心 - 屏幕中心
    yolo  : KPU 目标检测                 偏移 = 识别框中心 - 屏幕中心

  执行器 (两轴独立, 各有死区, 分别由 config.json 的 motor / servo 段配置):
    左右轴(yaw)  : 按 dx 做「速度」控制 —— |dx| 越大转速越快; 死区内停转; 丢靶停转
    上下轴(pitch): 按 dy 做「增量位置」控制 —— 每帧角度 += pitch_k × dy, 带限位

  dx = 目标中心x - 屏幕中心x   (右偏为正)
  dy = 目标中心y - 屏幕中心y   (下偏为正)

═══════════════════════════════════════════════════════════════
  接线 (标准版 40Pin 排针; 引脚以 skill/LUSHAN-PI-K230-PINOUT-SKILL.md 第一节为准)
  !! 完整接线文档见 项目根目录 接线图.md
═══════════════════════════════════════════════════════════════
  yaw 步进 (Emm42_V5.0, TTL 串口 UART3):
      K230 TX = GPIO32(Pin37) → 驱动器 RXD
      K230 RX = GPIO33(Pin40) → 驱动器 TXD        (交叉接)
      共地; 驱动器 V+ 独立供电 7~32V(勿接 K230 的 5V)
      驱动器侧需预置: 校验方式=固定0x6B, 波特率=115200, 串口ID=motor.addr, 运动模式=UART_FUN

  pitch 舵机 (FSUS 总线, UART2):  接线图见 云台/舵机接线图.png
      K230 TX = GPIO5(Pin11) --串1kΩ--→ 舵机信号线 S
      K230 RX = GPIO6(Pin13) ---------→ 同一根信号线 S     (半双工单总线)
      共地; 舵机 V+ 独立供电; 总线 ID = servo.pitch_id (0)

  !! 一个 UART 控制器只能有一组引脚: 步进固定占 UART3、舵机固定占 UART2, 不可混用。
     两者若配成同一个 port, 程序启动时会打印 WARN。

═══════════════════════════════════════════════════════════════
  文件结构 (自上而下)
═══════════════════════════════════════════════════════════════
   一  配置加载          从 /sdcard/config.json 读参数(跳过 _ 开头的注释键)
   二  运行参数          把配置铺平成模块级常量
   三  FSUS 协议驱动     FSServoBus(总线) / FSServo(单舵机)        → pitch
   四  Emm 步进驱动      EmmStepper                               → yaw
   五  执行器初始化      gimbal_send(dx,dy,dist,status) —— 视觉与执行器之间的唯一接口
   六  CV 矩形检测       find_paper_box() 等
   七  YOLO 支持         _yolo_ok()
   八  CV 主循环         run_cv()
   九  YOLO 主循环       run_yolo()
   十  入口              test_mode 自检 → 按 mode 进对应主循环

  所有可调参数都在 config.json 内并附注释; 本文件不含硬编码阈值, 改完重启生效。
"""

import time, os, gc, math
import cv2
from media.sensor import *
from media.display import *
from media.media import *

# CanMV 工作目录切换到 /sdcard
try:
    os.chdir("/sdcard")
except Exception:
    pass

# ============================================================================
# 一、配置加载
# ============================================================================

def _load_json(paths):
    """从候选路径加载JSON, 跳过_开头的注释键"""
    try:
        import ujson as j
    except ImportError:
        import json as j
    for p in paths:
        try:
            with open(p, "r") as f:
                raw = j.load(f)
            cfg = {}
            for k, v in raw.items():
                if k.startswith("_"):
                    continue
                if isinstance(v, dict):
                    cfg[k] = {sk: sv for sk, sv in v.items() if not sk.startswith("_")}
                else:
                    cfg[k] = v
            print("config: %s loaded" % p)
            return cfg
        except Exception:
            pass
    print("config: using defaults")
    return {}

def _get(cfg, key, default):
    return cfg.get(key, default) if cfg else default

CFG = _load_json(["/sdcard/config.json", "config.json"])

# ============================================================================
# 二、运行参数 (全部来自 config.json)
# ============================================================================

# ---- 摄像头 ----
SENSOR_W  = _get(_get(CFG, "camera", {}), "sensor_width", 640)
SENSOR_H  = _get(_get(CFG, "camera", {}), "sensor_height", 480)
FPS       = _get(_get(CFG, "camera", {}), "fps", 90)

# ---- 显示屏 ----
DISPLAY_W = _get(_get(CFG, "display", {}), "lcd_width", 800)
DISPLAY_H = _get(_get(CFG, "display", {}), "lcd_height", 480)
SHOW_TO_IDE = _get(_get(CFG, "display", {}), "to_ide", False)
DISPLAY_X = _get(_get(CFG, "display", {}), "x_offset", 80)
DISPLAY_Y = _get(_get(CFG, "display", {}), "y_offset", 0)
SHOW_OSD  = _get(_get(CFG, "display", {}), "show_osd", True)

# ---- 检测模式 ----
MODE = _get(_get(CFG, "detection", {}), "mode", "cv")   # "cv" / "yolo"
DETECT_EVERY = _get(_get(CFG, "detection", {}), "detect_every", 1)

# ---- 调试输出节奏 ----
DBG = _get(CFG, "debug", {})
PRINT_EVERY = DBG.get("print_every", 60)   # 每 N 帧打印一次检测信息
GC_EVERY = DBG.get("gc_every", 30)         # 每 N 帧执行一次垃圾回收

# ---- 屏幕中心 (追踪目标点) ----
CENTER_CFG = _get(CFG, "center", {})
CENTER_X = CENTER_CFG.get("x", SENSOR_W // 2)
CENTER_Y = CENTER_CFG.get("y", SENSOR_H // 2)

# ---- CV 矩形检测阈值 ----
r = _get(CFG, "rectangle", {})
WHITE_LOW  = tuple(r.get("white_low",  [135, 135, 115]))
WHITE_HIGH = tuple(r.get("white_high", [255, 255, 255]))
BLACK_LOW  = tuple(r.get("black_low",  [0, 0, 0]))
BLACK_HIGH = tuple(r.get("black_high", [85, 85, 85]))
MIN_AREA   = r.get("min_area", 3500)
MAX_AREA   = r.get("max_area", 180000)
MIN_ASPECT = r.get("min_aspect", 1.05)
MAX_ASPECT = r.get("max_aspect", 2.80)
BORDER_EXPAND_X = r.get("border_expand_x", 16)
BORDER_EXPAND_Y = r.get("border_expand_y", 16)
BLACK_CHECK_STEP = r.get("black_check_step", 4)
MIN_BLACK_HITS   = r.get("min_black_hits", 3)
CENTER_WHITE_STEP = r.get("center_white_step", 3)
MAX_CENTER_JUMP = r.get("max_center_jump", 220)
MAX_AREA_RATIO  = r.get("max_area_ratio", 3)
MIN_RECT_FILL   = r.get("min_rect_fill", 45)
MAX_SIDE_RATIO  = r.get("max_side_ratio", 3)
APPROX_EPSILON  = r.get("approx_epsilon", 0.04)

# ---- YOLO 配置 ----
YOLO_CFG = _get(CFG, "yolo", {})

# ---- 执行器配置 ----
# 上下(pitch)轴: FSUS 总线舵机;  左右(yaw)轴: Emm42 闭环步进
sv = _get(CFG, "servo", {})                 # [servo] 段 → pitch
SERVO_ENABLED = sv.get("enabled", False)
mv = _get(CFG, "motor", {})                 # [motor] 段 → yaw
MOTOR_ENABLED = mv.get("enabled", False)

# ============================================================================
# 三、FSUS 协议驱动 —— 上下(pitch)轴总线舵机
#    协议: Fashion Star Uart Servo, 小端字节序, 角度单位 0.1°
#    请求帧: [0x12, 0x4C, cmd, size, content..., checksum]      (checksum = 累加和低8位)
#    应答帧: [0x05, 0x1C, cmd, size, content..., checksum]
#    半双工单总线: 总线上可挂多个舵机, 靠 ID 区分(本项目只挂 pitch 一个, ID=0)
# ============================================================================

# ---- 本程序用到的 FSUS 指令 ----
CMD_PING = 1        # 通信检测(用于启动时判断舵机在线)
CMD_READ = 3        # 读参数(用于读角度限位)
CMD_SET_ANGLE = 8   # 单圈设角度 ±180°, 带行程时间 —— pitch 用它
CMD_STOP = 24       # 停止: 0=卸力 1=锁力 2=阻尼

# ---- FSUS 参数地址(配合 CMD_READ 使用) ----
ADDR_ANGLE_LIMIT_SW = 48      # 角度限位开关 开/关
ADDR_ANGLE_LIMIT_HIGH = 51    # 角度上限(0.1°)
ADDR_ANGLE_LIMIT_LOW = 52     # 角度下限(0.1°)


class FSServoBus:
    """一条舵机总线 = 一个 UART(总线上可挂多个舵机, 靠 ID 区分)

    接线: TX 串 1kΩ 接信号线, RX 直连同一根信号线, 共地(见 云台/舵机接线图.png)
    默认 UART2: TX=GPIO5(Pin11), RX=GPIO6(Pin13)
    """

    def __init__(self, uart_id=2, baud=115200, tx_pin=5, rx_pin=6, timeout_ms=200):
        from machine import UART, FPIOA
        fp = FPIOA()
        fn_tx = getattr(FPIOA, "UART%d_TXD" % uart_id, FPIOA.UART2_TXD)
        fn_rx = getattr(FPIOA, "UART%d_RXD" % uart_id, FPIOA.UART2_RXD)
        fp.set_function(tx_pin, fn_tx)
        fp.set_function(rx_pin, fn_rx)
        self.uart = UART(uart_id, baudrate=baud)
        self.tmo = timeout_ms

    def _pack(self, cmd, content):
        """请求帧: [0x12,0x4C, cmd, size, content..., checksum]"""
        n = len(content)
        f = bytearray(4 + n + 1)
        f[0] = 0x12
        f[1] = 0x4C
        f[2] = cmd
        f[3] = n
        f[4:4 + n] = content
        chk = 0x12 + 0x4C + cmd + n
        for b in content:
            chk += b
        f[4 + n] = chk & 0xFF
        return f

    def _read_n(self, n):
        """读满 n 字节, 超时返回 None"""
        buf = bytearray()
        t0 = time.ticks_ms()
        while len(buf) < n:
            if time.ticks_diff(time.ticks_ms(), t0) > self.tmo:
                return None
            m = self.uart.any()
            if m:
                buf += self.uart.read(min(m, n - len(buf)))
        return buf

    def txrx(self, cmd, content, wait_resp=True, retry=1):
        """发一帧; wait_resp=False 纯写. 成功返回(cmdId, content), 失败 None"""
        for _ in range(retry + 1):
            try:
                self.uart.read()
                self.uart.write(self._pack(cmd, content))
                if not wait_resp:
                    return None
                time.sleep_ms(5)  # 等舵机开始回应
                hdr = self._read_n(4)
                if hdr is None or hdr[0] != 0x05 or hdr[1] != 0x1C:
                    continue
                size = hdr[3]
                body = self._read_n(size + 1)
                if body is None:
                    continue
                cont = body[:size]
                chk = (0x05 + 0x1C + hdr[2] + size + sum(cont)) & 0xFF
                if chk == body[size]:
                    return hdr[2], cont
            except Exception:
                continue
        return None


class FSServo:
    """单个总线舵机"""

    def __init__(self, bus, sid):
        self.bus = bus
        self.id = sid

    def ping(self):
        """通信检测, 成功返回 True"""
        r = self.bus.txrx(CMD_PING, bytes([self.id]), retry=2)
        return r is not None and len(r[1]) > 0 and r[1][0] == self.id

    def set_angle(self, angle_deg, interval_ms=0, power=0):
        """单圈设角度 ±180°; interval_ms=行程时间(ms)"""
        a = int(max(-180.0, min(180.0, angle_deg)) * 10)
        self.bus.txrx(CMD_SET_ANGLE, bytes([
            self.id,
            a & 0xFF, (a >> 8) & 0xFF,
            interval_ms & 0xFF, (interval_ms >> 8) & 0xFF,
            power & 0xFF, (power >> 8) & 0xFF]), wait_resp=False)

    def stop(self, mode=2, power=500):
        """停止: 0=卸力 1=锁力 2=阻尼(默认最安全)"""
        self.bus.txrx(CMD_STOP, bytes([
            self.id, (mode | 0x10) & 0xFF,
            power & 0xFF, (power >> 8) & 0xFF]), wait_resp=False)

    def read_param(self, addr):
        """读参数, 返回 int(小端解码); 失败 None"""
        r = self.bus.txrx(CMD_READ, bytes([self.id, addr]), retry=2)
        if r is None or len(r[1]) < 4:
            return None
        c = r[1]
        val = c[2] | (c[3] << 8)
        return val


# ============================================================================
# 四、Emm42_V5.0 闭环步进驱动 —— 左右(yaw)轴
#    协议来自 云台/Emm_V5.0步进闭环驱动说明书Rev1.3.pdf
# ============================================================================
#  帧格式: [地址][功能码][数据...][校验]
#    - 默认校验方式 = 固定值, 末字节恒为 0x6B;  串口 115200 8N1
#    - 地址: 1~255, 0=广播
#  本程序用到的命令:
#    使能       addr F3 AB <en> <sync> 6B     en: 01=使能 00=失能
#    速度模式   addr F6 <dir> <spdH> <spdL> <acc> <sync> 6B
#               dir: 00=CCW 01=CW;  spd=0~3000(RPM);  acc=0~255(越大加速越快)
#    立即停止   addr FE 98 <sync> 6B
#    读总线电压 addr 24 6B  →  回 addr 24 <VH> <VL> 6B  (mV, 兼作在线探测)
#  !! 注意: 本协议所有多字节字段均为"高字节在前"(大端), 与 Modbus 相反。
#     依据说明书标注: 速度 05 DC=0x05DC=1500RPM; 脉冲 00 00 7D 00=32000;
#     电压 5C 6A=23658mV
# ============================================================================

EMM_CHECKSUM = 0x6B   # 校验方式=固定值时, 末字节恒为该值


class EmmStepper:
    """Emm42_V5.0 闭环步进驱动器 (TTL 串口, 速度模式调速)

    接线(config.json [motor] 段可改 port/tx_pin/rx_pin/addr):
      K230 TX = GPIO32(Pin37) -> 驱动器 RXD
      K230 RX = GPIO33(Pin40) -> 驱动器 TXD      (交叉接)
      GND 必须共地;  驱动器 V+ 独立供电 7~32V (勿接 K230 5V)

    驱动器侧需预先设好(上位机/OLED 菜单): 校验=固定0x6B, 波特率=115200,
    串口ID=addr, 运动模式=UART_FUN

    本类只实现"速度模式"调速: 追踪时由 |dx| 算出目标转速, 由 dx 符号定方向。
    """

    def __init__(self, uart_id=3, baud=115200, tx_pin=32, rx_pin=33,
                 addr=1, acc=200, timeout_ms=50):
        from machine import UART, FPIOA
        fp = FPIOA()
        fn_tx = getattr(FPIOA, "UART%d_TXD" % uart_id, None)
        fn_rx = getattr(FPIOA, "UART%d_RXD" % uart_id, None)
        if fn_tx is None or fn_rx is None:
            raise ValueError("FPIOA.UART%d_TXD/RXD not found" % uart_id)
        fp.set_function(tx_pin, fn_tx)
        fp.set_function(rx_pin, fn_rx)
        self.uart = UART(uart_id, baudrate=baud)
        self.addr = addr & 0xFF
        self.acc = acc        # 默认加减速档位(set_speed 可覆盖)
        self.tmo = timeout_ms

    # ---- 底层收发 ----
    def _send(self, func, data=b""):
        """组帧并发送: [addr][func][data...][0x6B]"""
        f = bytearray(3 + len(data))
        f[0] = self.addr
        f[1] = func
        f[2:2 + len(data)] = data
        f[-1] = EMM_CHECKSUM
        self.uart.write(f)
        return f

    def _recv(self, n, timeout_ms=None):
        """读满 n 字节, 超时返回 None"""
        t = self.tmo if timeout_ms is None else timeout_ms
        buf = bytearray()
        t0 = time.ticks_ms()
        while len(buf) < n:
            if time.ticks_diff(time.ticks_ms(), t0) > t:
                return None
            m = self.uart.any()
            if m:
                buf += self.uart.read(min(m, n - len(buf)))
        return buf

    # ---- 高层动作 ----
    def enable(self, en=True, sync=False):
        """使能/失能电机 (F3)"""
        try:
            self.uart.read()               # 清空上次残留
        except Exception:
            pass
        self._send(0xF3, bytes([0xAB,
                                0x01 if en else 0x00,
                                0x01 if sync else 0x00]))

    def stop(self, sync=False):
        """立即停止 (FE 98)"""
        self._send(0xFE, bytes([0x98, 0x01 if sync else 0x00]))

    def set_speed(self, direction, rpm, acc=None, sync=False):
        """速度模式 (F6): direction>0=CW / <0=CCW; rpm=0~3000"""
        s = int(rpm)
        if s < 0:
            s = 0
        elif s > 3000:
            s = 3000
        a = self.acc if acc is None else acc
        if a < 0:
            a = 0
        elif a > 255:
            a = 255
        self._send(0xF6, bytes([0x01 if direction > 0 else 0x00,
                                (s >> 8) & 0xFF, s & 0xFF,    # 速度: 高字节在前
                                a & 0xFF,
                                0x01 if sync else 0x00]))

    def read_bus_voltage(self):
        """读总线电压 (24) → mV; 无应答/校验错返回 None (兼作在线探测)"""
        try:
            self.uart.read()
        except Exception:
            pass
        self._send(0x24)
        r = self._recv(5)
        if r is None or r[0] != self.addr or r[1] != 0x24 or r[4] != EMM_CHECKSUM:
            return None
        return (r[2] << 8) | r[3]                            # 高字节在前

    def raw_probe(self, func=0x24, timeout_ms=300):
        """诊断用: 发一帧后把 RX 上收到的原始字节全部 dump 出来(不做校验/解析)。
        返回 (sent, recv); recv 为空表示一个字节都没收到。
        - recv == sent        → 短接了 TX/RX 的回环, K230 收发链路正常
        - recv 非空且 != sent → 收到了, 但格式不符(多半驱动器校验方式不是固定0x6B)
        - recv 为空           → K230 没发出去 / 驱动器没上电 / 驱动器串口关了
        """
        try:
            self.uart.read()
        except Exception:
            pass
        sent = self._send(func)
        buf = bytearray()
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            m = self.uart.any()
            if m:
                buf += self.uart.read(m)
                t0 = time.ticks_ms()      # 收到就续命, 直到线路静默
        return bytes(sent), bytes(buf)


# ============================================================================
# 五、云台执行器初始化 + gimbal_send
#    gimbal_send(dx, dy, dist, status) 是「视觉」与「执行器」之间唯一的接口:
#      dx → 左右轴(yaw, 步进) : 速度模式 —— |dx| 决定转速, dx 符号决定方向
#      dy → 上下轴(pitch, 舵机): 增量位置 —— 每帧 角度 += pitch_k × dy, 带限位
#    status: "track"=追踪中 / "lost"=丢靶
#
#    !! 本段整体包在 try 里: 任何一条轴初始化失败都会打印
#       "!!! actuator init failed: ..." 并把 gimbal_send 降级成空操作(两轴都不动)。
#       所以上电后第一眼就看这行日志, 确认两条轴都正常。
# ============================================================================

_yaw = None        # yaw 轴: EmmStepper 实例(步进驱动器)
_pitch = None      # pitch 轴: FSServo 实例(总线舵机)
_pitch_ang = 0.0   # pitch 角度积分(增量位置控制用, 单位: 度)

try:
    # pitch 参数先给默认值, 即使舵机未启用也保证 gimbal_send 里变量存在
    S_INTERVAL = 40      # 舵机行程时间 ms
    S_POWER = 0          # 舵机功率上限 mW, 0=不限
    S_DEADBAND = 5       # pitch 死区 px
    S_LOST_HOLD = True   # 丢靶时舵机是否保持位置

    # ---------------- pitch 上下轴: FSUS 总线舵机(UART2) ----------------
    if SERVO_ENABLED:
        _sbus = FSServoBus(uart_id=sv.get("port", 2), baud=sv.get("baud", 115200),
                           tx_pin=sv.get("tx_pin", 5), rx_pin=sv.get("rx_pin", 6))
        _pitch = FSServo(_sbus, sv.get("pitch_id", 0))

        PITCH_K = sv.get("pitch_k", 0.02)                            # 每帧角度增量系数
        PITCH_REV = -1.0 if sv.get("pitch_reverse", False) else 1.0  # 方向翻转
        PITCH_MIN = sv.get("pitch_min", -30)                         # 角度下限(度)
        PITCH_MAX = sv.get("pitch_max", 30)                          # 角度上限(度)
        S_INTERVAL = sv.get("interval_ms", 40)
        S_POWER = sv.get("power", 0)
        S_DEADBAND = sv.get("deadband_px", 5)
        S_LOST_HOLD = sv.get("lost_hold", True)

    # ---------------- yaw 左右轴: Emm42 闭环步进(UART3) ----------------
    if MOTOR_ENABLED:
        M_PORT = mv.get("port", 3)          # UART 控制器编号
        M_BAUD = mv.get("baud", 115200)     # 波特率
        M_TX = mv.get("tx_pin", 32)         # K230 TX  -> 驱动器 RXD
        M_RX = mv.get("rx_pin", 33)         # K230 RX  -> 驱动器 TXD
        M_ADDR = mv.get("addr", 1)          # 驱动器串口 ID
        M_ACC = mv.get("acc", 200)          # 加减速档位 0~255
        M_MIN_RPM = mv.get("min_rpm", 1)    # 最低转速: |dx| 刚越过死区时
        M_MAX_RPM = mv.get("max_rpm", 30)   # 最高转速: |dx| >= dx_full_px 时
        M_DX_FULL = mv.get("dx_full_px", 200)
        M_DEADBAND = mv.get("deadband_px", 5)
        M_REV = -1 if mv.get("reverse", False) else 1

        # 一个 UART 控制器只能有一组引脚, 配成同一个 port 会互相覆盖
        if SERVO_ENABLED and sv.get("port", 2) == M_PORT:
            print("  WARN: pitch 舵机与 yaw 步进都配在 UART%d 上, 会互相覆盖!" % M_PORT)
            print("        → 把其中一条改到别的 UART, 或把 servo.enabled 置 false")

        _yaw = EmmStepper(M_PORT, M_BAUD, M_TX, M_RX, M_ADDR, M_ACC)
        if mv.get("enable_on_boot", True):
            _yaw.enable(True)       # 使能电机(不使能的话收到速度指令也不转)
        _yaw.stop()                 # 上电先停住, 防止误转

    def _s_clamp(v, lo, hi):
        """把 v 夹到 [lo, hi]"""
        return lo if v < lo else (hi if v > hi else v)

    def gimbal_send(dx, dy, dist, status):
        """每帧调用一次, 把视觉偏差送给两条轴

        dx, dy : 目标中心相对屏幕中心的像素偏差(右偏/下偏为正)
        dist   : 偏差欧氏距离(本实现未用到, 仅为接口完整保留)
        status : "track"=追踪中 / "lost"=丢靶
        """
        global _pitch_ang
        try:
            if status == "lost":
                if _yaw:
                    _yaw.stop()                    # 丢靶: yaw 一律停转
                if _pitch and not S_LOST_HOLD:
                    _pitch.stop(2)                 # 丢靶: 舵机是否卸力看配置
                return

            # ---- yaw: |dx| 线性映射到转速, 符号定方向, 死区内停 ----
            if _yaw:
                if abs(dx) > M_DEADBAND:
                    ratio = min(1.0, abs(dx) / float(M_DX_FULL))
                    rpm = M_MIN_RPM + (M_MAX_RPM - M_MIN_RPM) * ratio
                    _yaw.set_speed(M_REV * (1 if dx > 0 else -1), rpm)
                else:
                    _yaw.stop()

            # ---- pitch: 增量位置控制, 角度积分后限位 ----
            if _pitch and abs(dy) > S_DEADBAND:
                _pitch_ang = _s_clamp(_pitch_ang + PITCH_REV * PITCH_K * dy,
                                      PITCH_MIN, PITCH_MAX)
                _pitch.set_angle(_pitch_ang, S_INTERVAL, S_POWER)
        except Exception:
            pass

    # ---- 上电回中: 舵机 2s 缓慢回 0°(步进无位置概念, 需机械居中安装) ----
    if SERVO_ENABLED and sv.get("center_on_boot", False):
        _pitch.set_angle(0.0, 2000, S_POWER)

    # ---- 读一次舵机角度限位(仅打印, 不参与控制) ----
    if SERVO_ENABLED:
        try:
            lim_sw = _pitch.read_param(ADDR_ANGLE_LIMIT_SW)
            if lim_sw is None:
                print("  pitch limit: read fail")
            else:
                lim_hi = _pitch.read_param(ADDR_ANGLE_LIMIT_HIGH)
                lim_lo = _pitch.read_param(ADDR_ANGLE_LIMIT_LOW)
                hi = (lim_hi - 65536 if lim_hi >= 32768 else lim_hi) / 10.0
                lo = (lim_lo - 65536 if lim_lo >= 32768 else lim_lo) / 10.0
                print("  pitch limit: sw=%s hi=%.1f lo=%.1f" %
                      ("ON" if lim_sw else "OFF", hi, lo))
        except Exception as _e:
            print("  pitch limit read fail:", _e)

    # ---- 打印执行器状态(上电自检的第一手信息) ----
    _yaw_info = "off"
    if _yaw:
        _v = _yaw.read_bus_voltage()    # 兼作在线探测: 有回包说明驱动器和串口都通
        _yaw_info = "emm(addr=%d UART%d TX=GPIO%d RX=GPIO%d %d~%dRPM rev=%s)%s" % (
            M_ADDR, M_PORT, M_TX, M_RX, M_MIN_RPM, M_MAX_RPM,
            "Y" if M_REV < 0 else "N",
            (" online %dmV" % _v) if _v else " (no resp)")
    _pitch_info = "off"
    if _pitch:
        _pitch_info = "servo(id=%d k=%.3f rev=%s)%s" % (
            _pitch.id, PITCH_K, "Y" if PITCH_REV < 0 else "N",
            " online" if _pitch.ping() else " (no resp)")
    print("actuator: yaw=%s | pitch=%s" % (_yaw_info, _pitch_info))

except Exception as e:
    print("\n!!! actuator init failed: %s" % str(e))
    gimbal_send = lambda dx, dy, d, s: None


# ============================================================================
# 六、CV 矩形检测
# ============================================================================

def point_xy(point):
    try:
        return int(point[0][0]), int(point[0][1])
    except Exception:
        return int(point[0]), int(point[1])

def order_points(points):
    pts = [point_xy(p) for p in points]
    cx = sum(p[0] for p in pts) // 4
    cy = sum(p[1] for p in pts) // 4
    top = [p for p in pts if p[1] < cy]
    bottom = [p for p in pts if p[1] >= cy]
    if len(top) != 2:
        pts.sort(key=lambda p: p[1])
        top, bottom = pts[:2], pts[2:]
    top.sort(key=lambda p: p[0])
    bottom.sort(key=lambda p: p[0])
    return [top[0], top[1], bottom[1], bottom[0]]

def get_contours(result):
    return result[0] if len(result) == 2 else result[1]

def quad_center(box):
    """四边形对角线交点(精确中心)"""
    x1, y1 = box[0]; x2, y2 = box[2]; x3, y3 = box[1]; x4, y4 = box[3]
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        return (sum(p[0] for p in box) // 4, sum(p[1] for p in box) // 4)
    pre = x1 * y2 - y1 * x2
    post = x3 * y4 - y3 * x4
    cx = (pre * (x3 - x4) - (x1 - x2) * post) // den
    cy = (pre * (y3 - y4) - (y1 - y2) * post) // den
    return int(cx), int(cy)

def side_len2(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return dx * dx + dy * dy

def is_good_quad(quad, contour_area, box_area):
    if contour_area * 100 < box_area * MIN_RECT_FILL:
        return False
    sides = [side_len2(quad[i], quad[(i + 1) % 4]) for i in range(4)]
    if min(sides) <= 0 or max(sides) > min(sides) * MAX_SIDE_RATIO * MAX_SIDE_RATIO:
        return False
    h_avg = (sides[0] + sides[2]) // 2
    v_avg = (sides[1] + sides[3]) // 2
    if min(h_avg, v_avg) <= 0:
        return False
    ratio = max(h_avg, v_avg) / min(h_avg, v_avg)
    if ratio > MAX_ASPECT * MAX_ASPECT or ratio < MIN_ASPECT * MIN_ASPECT:
        return False
    return True

def contour_quad(cnt, box_area):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, APPROX_EPSILON * peri, True)
    if len(approx) != 4:
        return None
    quad = order_points(approx)
    if not is_good_quad(quad, cv2.contourArea(cnt), box_area):
        return None
    return quad

def has_black_border(frame_np, x, y, w, h):
    if w < 20 or h < 20:
        return False
    black = cv2.inRange(frame_np, BLACK_LOW, BLACK_HIGH)
    edge = max(2, min(w, h) // 18)
    top = black[y:y + edge, x:x + w]
    bottom = black[y + h - edge:y + h, x:x + w]
    left = black[y:y + h, x:x + edge]
    right = black[y:y + h, x + w - edge:x + w]
    hits = 0
    if cv2.countNonZero(top) > w // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(bottom) > w // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(left) > h // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(right) > h // BLACK_CHECK_STEP: hits += 1
    return hits >= MIN_BLACK_HITS

def has_white_center(white_mask, x, y, w, h):
    ix, iy = x + w // 4, y + h // 4
    iw, ih = w // 2, h // 2
    if iw <= 0 or ih <= 0:
        return False
    roi = white_mask[iy:iy + ih, ix:ix + iw]
    return cv2.countNonZero(roi) > (iw * ih) // CENTER_WHITE_STEP

def clamp_box(x, y, w, h):
    if x < 0: x = 0
    if y < 0: y = 0
    if x + w > SENSOR_W: w = SENSOR_W - x
    if y + h > SENSOR_H: h = SENSOR_H - y
    return x, y, w, h

def find_paper_box(frame_np, last_box):
    """检测矩形靶标, 返回 (box4点, 轮廓数, 面积候选数)"""
    mask = cv2.inRange(frame_np, WHITE_LOW, WHITE_HIGH)
    contours = get_contours(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    best, best_score = None, -1
    area_count = 0
    last_cx = last_cy = last_area = 0
    if last_box:
        last_cx, last_cy = quad_center(last_box)
        xs = [p[0] for p in last_box]; ys = [p[1] for p in last_box]
        last_area = (max(xs) - min(xs)) * (max(ys) - min(ys))
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            continue
        box_area = w * h
        if box_area < MIN_AREA or box_area > MAX_AREA:
            continue
        area_count += 1
        aspect = w / h
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue
        x -= BORDER_EXPAND_X; y -= BORDER_EXPAND_Y
        w += BORDER_EXPAND_X * 2; h += BORDER_EXPAND_Y * 2
        x, y, w, h = clamp_box(x, y, w, h)
        cx, cy = x + w // 2, y + h // 2
        if last_box and abs(cx - last_cx) + abs(cy - last_cy) > MAX_CENTER_JUMP:
            continue
        if last_area and (box_area > last_area * MAX_AREA_RATIO or last_area > box_area * MAX_AREA_RATIO):
            continue
        if not has_white_center(mask, x, y, w, h):
            continue
        if not has_black_border(frame_np, x, y, w, h):
            continue
        quad = contour_quad(cnt, box_area)
        if not quad:
            continue
        score = box_area - (abs(cx - last_cx) + abs(cy - last_cy)) * 8 if last_box else box_area
        if score > best_score:
            best_score = score
            best = quad
    return best, len(contours), area_count

def draw_box(frame_np, box):
    """绘制矩形框+中心十字, 返回 (cx, cy, dx, dy)"""
    for i in range(4):
        cv2.line(frame_np, box[i], box[(i + 1) % 4], (0, 255, 0), 2)
    cx, cy = quad_center(box)
    dx, dy = cx - CENTER_X, cy - CENTER_Y
    cv2.line(frame_np, (cx - 14, cy), (cx + 14, cy), (255, 255, 0), 2)
    cv2.line(frame_np, (cx, cy - 14), (cx, cy + 14), (255, 255, 0), 2)
    cv2.circle(frame_np, (cx, cy), 4, (0, 0, 255), 1)
    return cx, cy, dx, dy


# ============================================================================
# 七、YOLO 检测支持
# ============================================================================

def _yolo_ok():
    try:
        import nncase_runtime
    except ImportError:
        return False
    try:
        from libs.PipeLine import PipeLine
    except ImportError:
        return False
    try:
        from libs.YOLO import YOLOv8
    except ImportError:
        return False
    return True


# ============================================================================
# 八、CV 模式主循环
# ============================================================================

def run_cv():
    sensor = None
    frame_id = lost_count = 0
    last_box = None
    fps = fps_count = 0
    fps_tick = time.ticks_ms()

    try:
        print("boot")
        os.exitpoint(os.EXITPOINT_ENABLE)
        sensor = Sensor(width=1280, height=960, fps=FPS)
        sensor.reset()
        sensor.set_framesize(width=SENSOR_W, height=SENSOR_H, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
        print("sensor ok")
        Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H, to_ide=SHOW_TO_IDE)
        MediaManager.init()
        # sensor.run() 重试: 上次异常退出后 VICAP 硬件可能残留, 需多次尝试
        for _retry in range(3):
            try:
                sensor.run()
                break
            except Exception as _e:
                print("sensor.run retry %d: %s" % (_retry + 1, _e))
                time.sleep_ms(500)
                if _retry == 2:
                    raise
        print("run ok")

        while True:
            os.exitpoint()
            frame_id += 1
            frame = sensor.snapshot(chn=CAM_CHN_ID_0)
            frame_np = frame.to_numpy_ref()

            # 检测
            if frame_id % DETECT_EVERY == 0 or not last_box:
                box, contour_count, area_count = find_paper_box(frame_np, last_box)
            else:
                box, contour_count, area_count = last_box, 0, 0

            if box:
                last_box = box
                lost_count = 0
                cx, cy, dx, dy = draw_box(frame_np, box)
                dist = math.sqrt(dx * dx + dy * dy)
                gimbal_send(int(dx), int(dy), dist, "track")
                if frame_id % PRINT_EVERY == 0:
                    print("OK  center=(%d,%d)  d=(%d,%d)  dist=%.0f" % (cx, cy, dx, dy, dist))
            else:
                lost_count += 1
                if lost_count > 2:
                    last_box = None
                    gimbal_send(0, 0, 0, "lost")
                    if frame_id % PRINT_EVERY == 0:
                        print("LOST")

            # OSD
            if SHOW_OSD and box:
                cv2.putText(frame_np, "dx:%+d dy:%+d" % (dx, dy),
                            (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # FPS
            fps_count += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, fps_tick) >= 1000:
                fps = fps_count * 1000 // time.ticks_diff(now, fps_tick)
                fps_count = 0
                fps_tick = now
            cv2.putText(frame_np, "FPS:%d" % fps, (4, SENSOR_H - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            Display.show_image(frame, x=DISPLAY_X, y=DISPLAY_Y)
            if frame_id % GC_EVERY == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        print("error:", e)
    finally:
        if sensor:
            try:
                sensor.stop()
            except Exception:
                pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception:
            pass
        time.sleep_ms(100)
        try:
            MediaManager.deinit()
        except Exception:
            pass
        print("exit")


# ============================================================================
# 九、YOLO 模式主循环
# ============================================================================

def run_yolo():
    from libs.PipeLine import PipeLine, ScopedTiming
    from libs.YOLO import YOLOv8

    MP = YOLO_CFG.get("model_path", "/sdcard/model/model.kmodel")
    LABELS = YOLO_CFG.get("labels", ["target"])
    ISIZE = YOLO_CFG.get("input_size", [320, 320])
    CONF = YOLO_CFG.get("confidence", 0.5)
    NMS = YOLO_CFG.get("nms_threshold", 0.45)
    MB = YOLO_CFG.get("max_boxes", 50)
    SELECT = YOLO_CFG.get("select", "max_conf")
    TASK = YOLO_CFG.get("task", "detect")

    RGB888P_SIZE = [320, 320] if TASK == "segment" else [640, 360]

    print("YOLOv8  task=%s  model=%s  conf=%.2f" % (TASK, MP, CONF))

    pl = PipeLine(rgb888p_size=RGB888P_SIZE, display_mode="lcd")
    pl.create()
    ds = pl.get_display_size()
    disp_w = ds[0] if ds else DISPLAY_W
    disp_h = ds[1] if ds else DISPLAY_H

    yolo = YOLOv8(task_type=TASK, mode="video",
                  kmodel_path=MP, labels=LABELS,
                  rgb888p_size=RGB888P_SIZE, model_input_size=ISIZE,
                  display_size=ds, conf_thresh=CONF, nms_thresh=NMS,
                  max_boxes_num=MB, debug_mode=0)
    yolo.config_preprocess()

    lost_count = fps_count = fps = 0
    fps_tick = time.ticks_ms()

    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = yolo.run(img)
                yolo.draw_result(res, pl.osd_img)

            fps_count += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, fps_tick) >= 1000:
                fps = fps_count * 1000 // time.ticks_diff(now, fps_tick)
                fps_count = 0
                fps_tick = now

            if res and len(res[0]) > 0:
                boxes, cls_ids, confs = res[0], res[1], res[2]
                best_idx, best_score = 0, -1
                for i in range(len(boxes)):
                    x, y, w, h = [int(round(v, 0)) for v in boxes[i]]
                    if SELECT == "max_area":
                        score = w * h
                    elif SELECT == "nearest":
                        bcx, bcy = x + w // 2, y + h // 2
                        score = -((bcx - disp_w // 2) ** 2 + (bcy - disp_h // 2) ** 2)
                    else:
                        score = confs[i]
                    if score > best_score:
                        best_score = score
                        best_idx = i

                x, y, w, h = [int(round(v, 0)) for v in boxes[best_idx]]
                # 识别框中心 → 偏移
                cx, cy = x + w // 2, y + h // 2
                dx, dy = cx - disp_w // 2, cy - disp_h // 2
                dist = math.sqrt(dx * dx + dy * dy)
                gimbal_send(int(dx), int(dy), dist, "track")
                lost_count = 0

                label = LABELS[cls_ids[best_idx]] if cls_ids[best_idx] < len(LABELS) else "?"
                pl.osd_img.draw_string_advanced(4, 4, 24,
                    "%s %.2f | dx:%+d dy:%+d" % (label, confs[best_idx], dx, dy),
                    color=(0, 255, 0))
            else:
                lost_count += 1
                if lost_count > 2:
                    gimbal_send(0, 0, 0, "lost")
                pl.osd_img.draw_string_advanced(4, 4, 24, "LOST", color=(255, 0, 0))

            pl.osd_img.draw_string_advanced(4, disp_h - 28, 24, "FPS:%d" % fps, color=(0, 255, 0))
            pl.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("stopped")
    except BaseException as e:
        print("error:", e)
    finally:
        yolo.deinit()
        pl.destroy()


# ============================================================================
# 十、入口 — 模式切换
# ============================================================================

print("=" * 50)
print("mode: %s | pitch: %s | yaw: %s" %
      (MODE.upper(), SERVO_ENABLED, MOTOR_ENABLED))
print("=" * 50)

# 电机测试模式优先执行(不跑视觉)
if MOTOR_ENABLED and mv.get("test_mode", False) and _yaw:
    import time as _t

    _rpm = mv.get("test_rpm", 10)
    _dirn = M_REV if M_REV else 1
    print(">>> MOTOR TEST (Emm42 步进, 持续旋转自检)")
    print("  串口: UART%d @%dbps, ID=%d, 校验=固定0x6B" % (M_PORT, M_BAUD, M_ADDR))
    print("  接线: K230 TX=GPIO%d -> 驱动器RXD,  RX=GPIO%d -> 驱动器TXD" % (M_TX, M_RX))
    print("  共地 + 驱动器 V+ 独立供电 7~32V(勿接 K230 5V)")
    _v = _yaw.read_bus_voltage()
    if _v:
        print("  在线探测: OK, 母线电压 %dmV" % _v)
    else:
        print("  在线探测: 无应答")

        def _hx(b):
            return " ".join("%02X" % x for x in b)

        _sent, _raw = _yaw.raw_probe()
        if not _raw:
            print("  RX 原始字节: (无) —— 一个字节都没收到")
            print("    → K230 发出去了但没人回。依次查:")
            print("      1. 驱动器 OLED 是否正常 / 显示什么(Waiting V+ Power!?)")
            print("      2. 共地: K230 GND 与驱动器电源 GND 是否连通")
            print("      3. 交叉接线: TX GPIO%d -> 驱动器RXD, RX GPIO%d -> 驱动器TXD" %
                  (M_TX, M_RX))
            print("      4. 驱动器菜单: 串口功能=UART_FUN(不能是 P_Pul/RxTx_OFF),")
            print("         串口ID=%d, 波特率=%d, 校验方式=固定0x6B, 应答=Receive" %
                  (M_ADDR, M_BAUD))
        elif _raw == _sent:
            print("  RX 原始字节: %s   <- 正是我们自己发出去的帧" % _hx(_raw))
            print("    → 回环测试通过: K230 的 UART%d(GPIO%d/GPIO%d) 收发都正常" %
                  (M_PORT, M_TX, M_RX))
            print("      拔掉 TX/RX 之间的短接线, 接驱动器")
        else:
            print("  RX 原始字节: %s" % _hx(_raw))
            print("    (本机发出的是: %s)" % _hx(_sent))
            print("    → 收到字节但不是自身回环: 多半驱动器「校验方式」不是 固定0x6B")
    _yaw.enable(True)
    _t.sleep_ms(100)
    print("\n  持续%s %dRPM (reverse=%s; Ctrl+C 退出)" %
          ("正转" if _dirn > 0 else "反转", _rpm, mv.get("reverse", False)))
    print("  电机不动 → 先看上面'在线探测'是否 OK")
    while True:
        _yaw.set_speed(_dirn, _rpm)     # 持续重发, 防止丢帧后停住
        _t.sleep_ms(500)

# 舵机测试模式(不跑视觉): ping + 往返扫动
if SERVO_ENABLED and sv.get("test_mode", False) and _pitch:
    import time as _t
    _sid = sv.get("pitch_id", 0)
    print(">>> SERVO TEST (pitch id=%d)" % _sid)
    print("  接线: UART%d TX=GPIO%d(Pin11)串1kΩ接信号线, RX=GPIO%d(Pin13)直连" %
          (sv.get("port", 2), sv.get("tx_pin", 5), sv.get("rx_pin", 6)))
    print("  共地 + 舵机独立供电")
    if _pitch.ping():
        print("  ping OK")
    else:
        print("  ping FAIL —— 查 1kΩ串阻 / 共地 / 舵机供电 / 舵机ID(应=%d)" % _sid)

    _lo = sv.get("pitch_min", -30)
    _hi = sv.get("pitch_max", 30)
    _seq = [0, _hi, 0, _lo, 0]
    print("  往返扫动 %s 度, Ctrl+C 退出" % _seq)
    while True:
        for _a in _seq:
            print("  -> %.0f deg" % _a)
            _pitch.set_angle(_a, 800, 0)
            _t.sleep_ms(1500)

if MODE == "yolo":
    if _yolo_ok():
        run_yolo()
    else:
        print("YOLO libs missing, fallback to CV")
        run_cv()
else:
    run_cv()
