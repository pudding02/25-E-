"""
K230 物体追踪 — 双轴云台 (双步进)
================================
  左右(yaw)轴:  Emm42_V5.0 闭环步进驱动器  (UART3, 串口速度模式)
  上下(pitch)轴: Emm42_V5.0 闭环步进驱动器  (UART2, 串口速度模式)

═══════════════════════════════════════════════════════════════
  功能说明
═══════════════════════════════════════════════════════════════
  摄像头采集 → 物体检测 → 算目标中心与屏幕中心的偏移 → 驱动云台

  检测方式 (config.json → detection.mode):
    cv    : 矩形靶标检测(白心+黑边验证)   偏移 = 四边形中心 - 屏幕中心
    yolo  : KPU 目标检测                 偏移 = 识别框中心 - 屏幕中心

  执行器 (两轴都是 Emm42 闭环步进, 速度控制逻辑相同, 由 config.json 的 motor / pitch_motor 段配置):
    左右轴(yaw)  : 按 dx 做「速度」控制 —— |dx| 越大转速越快; 死区内停转;
                   丢靶预判: 目标从左右两侧丢失后, 朝最后方向续追一小段时间(可关, 见 coast_step)
    上下轴(pitch): 按 dy 做「速度」控制 —— |dy| 越大转速越快; 死区内停转; 丢靶停转

  dx = 目标中心x - 屏幕中心x   (右偏为正)
  dy = 目标中心y - 屏幕中心y   (下偏为正)

═══════════════════════════════════════════════════════════════
  接线 (标准版 40Pin 排针; 引脚以 skill/LUSHAN-PI-K230-PINOUT-SKILL.md 第一节为准)
  !! 完整接线文档见 项目根目录 接线图.md
═══════════════════════════════════════════════════════════════
  yaw 步进 (Emm42_V5.0, TTL 串口 UART3):
      K230 TX = GPIO32(Pin37) → 驱动器① RXD
      K230 RX = GPIO33(Pin40) → 驱动器① TXD        (交叉接)
      共地; 驱动器① V+ 独立供电 7~32V(勿接 K230 的 5V)
      驱动器侧需预置: 校验方式=固定0x6B, 波特率=115200, 串口ID=motor.addr, 运动模式=UART_FUN

  pitch 步进 (Emm42_V5.0, TTL 串口 UART2):   ← 原总线舵机方案已改为步进(第二台驱动器)
      K230 TX = GPIO5(Pin11) → 驱动器② RXD
      K230 RX = GPIO6(Pin13) → 驱动器② TXD        (交叉接, 全双工)
      共地; 驱动器② V+ 独立供电 7~32V(勿接 K230 的 5V)
      驱动器侧需预置: 校验方式=固定0x6B, 波特率=115200, 串口ID=pitch_motor.addr, 运动模式=UART_FUN

  !! 一个 UART 控制器只能有一组引脚: yaw 固定占 UART3、pitch 固定占 UART2, 不可混用。
     两者若配成同一个 port, 程序启动时会打印 WARN。

═══════════════════════════════════════════════════════════════
  文件结构 (自上而下)
═══════════════════════════════════════════════════════════════
   一  配置加载          从 /sdcard/config.json 读参数(跳过 _ 开头的注释键)
   二  运行参数          把配置铺平成模块级常量
   三  Emm 步进驱动      EmmStepper (速度模式 + 位置模式)          → yaw / pitch
   四  执行器初始化      gimbal_send(dx,dy,dist,status) —— 视觉与执行器之间的唯一接口
   五  CV 矩形检测       find_paper_box() 等
   六  YOLO 支持         _yolo_ok()
   六点五 触屏中心校准    calib_* —— 触摸按键移动绿点(追踪收敛点)/保存写回 config
   七  CV 主循环         run_cv()
   八  YOLO 主循环       run_yolo()
   九  入口              test_mode 自检 → 按 mode 进对应主循环

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

CFG_PATH = None    # 实际命中的 config.json 路径(触屏校准「保存」回写用)

def _load_json(paths):
    """从候选路径加载JSON, 跳过_开头的注释键"""
    global CFG_PATH
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
            CFG_PATH = p
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
CSI_ID    = _get(_get(CFG, "camera", {}), "csi_id", 2)   # CSI 端口(庐山派 GC2093=2; 画面视野偏移先查此项)
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
# 左右(yaw)轴: [motor] 段, Emm42 步进-速度模式;  上下(pitch)轴: [pitch_motor] 段, Emm42 步进-速度模式(同逻辑)
mv = _get(CFG, "motor", {})                 # [motor] 段 → yaw
MOTOR_ENABLED = mv.get("enabled", False)
pm = _get(CFG, "pitch_motor", {})           # [pitch_motor] 段 → pitch
PITCH_ENABLED = pm.get("enabled", False)

# ---- V7 红色目标(红环)检测 ----
rt = _get(CFG, "red_target", {})
RED_LOW  = tuple(rt.get("red_low", [130, 0, 0]))
RED_HIGH = tuple(rt.get("red_high", [255, 120, 120]))
RING_MIN_DIAM = rt.get("ring_min_diam_px", 40)
RING_MAX_DIAM = rt.get("ring_max_diam_px", 900)
RING_MAX_ASPECT = rt.get("ring_max_aspect", 1.6)
HOLLOW_SAMPLE_PCT = rt.get("hollow_sample_pct", 50)
HOLLOW_MAX_HITS = rt.get("hollow_max_hits", 2)
ROI_MARGIN_PX = rt.get("roi_margin_px", 12)
RADIUS_BIAS = rt.get("radius_bias_px", 0)

# ---- V7 激光偏移(激光点相对画面中心; 与光轴同轴则 0; 触屏校准运行时调整它) ----
LOX = CENTER_CFG.get("laser_offset_x", 0)
LOY = CENTER_CFG.get("laser_offset_y", 0)

# ---- V7.7 触屏中心校准(绿点=追踪收敛点=CENTER+LOX/LOY; 「保存」写回 [center] 的 laser_offset) ----
cb = _get(CFG, "center_calib", {})
CALIB_ENABLED   = cb.get("enabled", True)
CALIB_STEP      = cb.get("step_px", 4)          # 每按一次/每连发一拍移动的像素
CALIB_REPEAT    = cb.get("repeat_ms", 120)      # 按住连发间隔(ms)
CALIB_DOT_R     = cb.get("dot_radius", 6)       # 绿点半径(px)
CALIB_TOUCH_ROT = cb.get("touch_rotate", -1)    # 触摸坐标旋转(-1=设备默认; 方向不对时调 0-3)
CALIB_SHOW_DOT  = cb.get("show_dot", True)      # 启动时绿点+按钮是否显示
CALIB_XMAX = SENSOR_W // 2                      # 校准范围上限(防呆)
CALIB_YMAX = SENSOR_H // 2

# ---- V7 上电搜索 ----
srch = _get(CFG, "search", {})
SEARCH_EN = srch.get("enabled", True)
SEARCH_RPM = srch.get("rpm", 12)
SEARCH_DIR = srch.get("direction", 1)
SEARCH_MAX_MS = srch.get("max_ms", 0)

# ---- V7 画圈 ----
cc = _get(CFG, "circle", {})
CIRC_SPEED = cc.get("speed_deg_s", 60)
CIRC_DIR = cc.get("direction", 1)
CIRC_PAUSE = cc.get("pause_px", 120)

# ---- V7 模式引脚(GPIO49: 被供电=画圈) ----
mp = _get(CFG, "mode_pin", {})
PIN_ENABLED = mp.get("enabled", True)
PIN_GPIO = mp.get("gpio", 49)
PIN_ACTIVE = mp.get("active_level", 1)
PIN_DEBOUNCE = mp.get("debounce_n", 5)

# ============================================================================
# 三、Emm42_V5.0 闭环步进驱动 —— yaw / pitch(均为速度模式)
#    协议来自 云台/Emm_V5.0步进闭环驱动说明书Rev1.3.pdf
# ============================================================================
#  帧格式: [地址][功能码][数据...][校验]
#    - 默认校验方式 = 固定值, 末字节恒为 0x6B;  串口 115200 8N1
#    - 地址: 1~255, 0=广播
#  本程序用到的命令:
#    使能       addr F3 AB <en> <sync> 6B     en: 01=使能 00=失能
#    速度模式   addr F6 <dir> <spdH> <spdL> <acc> <sync> 6B          —— yaw 轴用
#               dir: 00=CCW 01=CW;  spd=0~3000(RPM);  acc=0~255(越大加速越快)
#    位置模式   addr FD <dir> <spdH> <spdL> <acc> <p3><p2><p1><p0> <abs> <sync> 6B   —— 备用(未接入)
#               16细分下 3200 脉冲/圈; abs: 00=相对 01=绝对; 目标为32位(负数用补码)
#    位置清零   addr 0A 6D 6B                  (把当前物理位置定义为 0° —— pitch 上电用)
#    触发回零   addr 9A <mode> <sync> 6B      (0=单圈就近 1=单圈方向 2=多圈碰撞 3=限位开关; 预留)
#    立即停止   addr FE 98 <sync> 6B
#    读总线电压 addr 24 6B  →  回 addr 24 <VH> <VL> 6B  (mV, 兼作在线探测)
#  !! 注意: 本协议所有多字节字段均为"高字节在前"(大端), 与 Modbus 相反。
#     依据说明书标注: 速度 05 DC=0x05DC=1500RPM; 脉冲 00 00 7D 00=32000;
#     电压 5C 6A=23658mV
# ============================================================================

EMM_CHECKSUM = 0x6B   # 校验方式=固定值时, 末字节恒为该值


class EmmStepper:
    """Emm42_V5.0 闭环步进驱动器 (TTL 串口; 速度模式 + 位置模式)

    接线(port/tx_pin/rx_pin/addr 由 config.json 各段提供, 如 [motor]/[pitch_motor]):
      K230 TX = tx_pin -> 驱动器 RXD
      K230 RX = rx_pin -> 驱动器 TXD      (交叉接)
      GND 必须共地;  驱动器 V+ 独立供电 7~32V (勿接 K230 5V)

    驱动器侧需预先设好(上位机/OLED 菜单): 校验=固定0x6B, 波特率=115200,
    串口ID=addr, 运动模式=UART_FUN

    速度模式: 追踪时由 |偏差| 算出目标转速, 由偏差符号定方向    (yaw / pitch 现用)
    位置模式: 下发绝对目标位置(脉冲), 配合 clear_position() 定义 0°;
              当前未接入控制流(备用, 见 move/home; 脉冲换算依赖细分, 16细分=3200/圈)。
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

    def move(self, pulses, rpm, acc=None, absolute=True, sync=False):
        """位置模式 (FD): 目标位置=脉冲数; absolute=True 时按绝对位置下发。

        16细分下 3200 脉冲/圈; 负目标用 32 位补码编码; dir 位按目标正负给
        (>=0 → CCW 0x01)。绝对模式下 dir 位的实际作用以实机验证为准。
        """
        p = int(pulses) & 0xFFFFFFFF
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
        self._send(0xFD, bytes([
            0x01 if pulses >= 0 else 0x00,
            (s >> 8) & 0xFF, s & 0xFF,
            a & 0xFF,
            (p >> 24) & 0xFF, (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF,
            0x01 if absolute else 0x00,
            0x01 if sync else 0x00]))

    def clear_position(self):
        """位置清零 (0A 6D): 把当前位置角度/误差/脉冲数全部清零。

        pitch 轴用它把「上电位置」定义为 0°(要求上电前云台位于行程中部)。
        """
        self._send(0x0A, bytes([0x6D]))

    def home(self, mode=0, sync=False):
        """触发回零 (9A): mode 0=单圈就近 1=单圈方向 2=多圈无限位碰撞 3=多圈限位开关。

        本项目当前未接入常规流程(预留); 回零参数需先在驱动器菜单里设好。
        """
        self._send(0x9A, bytes([mode & 0xFF, 0x01 if sync else 0x00]))

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
# 四、云台执行器初始化 + gimbal_send
#    gimbal_send(dx, dy, dist, status) 是「视觉」与「执行器」之间唯一的接口:
#      dx → 左右轴(yaw, 步进)  : 速度模式 —— |dx| 决定转速, dx 符号决定方向
#      dy → 上下轴(pitch, 步进): 速度模式 —— |dy| 决定转速, dy 符号决定方向(与 yaw 相同逻辑)
#    status: "track"=追踪中 / "lost"=丢靶
#    "lost" 时: yaw 支持「丢靶预判」—— 目标从左右两侧丢失后朝最后方向续转一段
#    (无目标追踪, 见 coast_step 与 config [motor].coast_* 键; 默认关, coast_test=true 可单独自检, 先于 test_mode 执行)
#    pitch 丢靶即停。
#
#    !! 本段整体包在 try 里: 任何一条轴初始化失败都会打印
#       "!!! actuator init failed: ..." 并把 gimbal_send 降级成空操作(两轴都不动)。
#       所以上电后第一眼就看这行日志, 确认两条轴都正常。
# ============================================================================

_yaw = None        # yaw 轴: EmmStepper 实例(速度模式)
_pitch = None      # pitch 轴: EmmStepper 实例(速度模式)
_mode_pin = None   # 模式引脚(GPIO49): 被供电=画圈; machine.Pin 实例(未启用/初始化失败=None)
_gimbal_ok = False # gimbal_send 是否已就绪(执行器初始化成功)——自检脚本用它防"静默假通过"

# 丢靶预判(coast)状态 —— 由 gimbal_send 维护(见 coast_step)
_last_dx = 0           # 最后一次追踪的 yaw 偏差(带符号); 丢靶期间保留, 供预判参考
_coast_active = False  # 是否正在丢靶预判中
_coast_start = 0       # 预判启动时刻(time.ticks_ms())

# 上电搜索状态 —— 由 gimbal_send 维护(见 search_step)
_acquired = False      # 启动后是否已捕获过目标; 一旦置位, 上电搜索永久退出
_search_spin = False   # 是否正在搜索旋转
_search_start = 0      # 搜索启动时刻(time.ticks_ms())
_search_done = False   # 搜索超时后不再重启


def coast_step(now_ms, active, start_ms, last_dx, edge_px, coast_ms):
    """丢靶预判状态机(纯函数 —— tools/test_coast_logic.py 抓取本函数做主机侧测试)。

    返回 (mode, new_active, new_start, new_last_dx):
      "start" = 本帧启动预判(开始朝最后方向续转)   "run" = 预判进行中
      "stop"  = 预判超时终止(停转)                 "idle"= 未达触发条件
    规则: 未激活且 |last_dx| >= edge_px → 启动(记 now_ms);
          已激活且未到 coast_ms → 续跑;
          已激活且到点 → 终止, 并把 last_dx 清 0(防止超时后反复重启)。
    """
    if not active:
        if abs(last_dx) >= edge_px:
            return ("start", True, now_ms, last_dx)
        return ("idle", False, 0, last_dx)
    if time.ticks_diff(now_ms, start_ms) < coast_ms:
        return ("run", True, start_ms, last_dx)
    return ("stop", False, 0, 0)


# ============================================================================
# 四点五、V7 视觉纯函数(主机可测) —— 画圈几何/上电搜索/引脚去抖
#   tools/test_circle_logic.py 抓取本节函数做主机侧测试(与 coast_step 同模式)
# ============================================================================

def point_in_quad(px, py, quad):
    """凸四边形包含判定(纯函数; 叉积同号法)。

    quad = order_points 输出序的 4 点 [TL, TR, BR, BL]; 点恰在边上视为在内。
    """
    sign = 0
    for i in range(4):
        x1, y1 = quad[i]
        x2, y2 = quad[(i + 1) % 4]
        cross = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
        if cross != 0:
            s = 1 if cross > 0 else -1
            if sign == 0:
                sign = s
            elif sign != s:
                return False
    return True

def classify_blob(w, h, hollow_hits):
    """红色块分流(纯函数): 大尺寸近方且空心 -> "ring"; 其余(小尺寸/实心/畸形) -> None。

    hollow_hits: 50% 外径半径处 8 点采样的红色命中数(由 find_red_ring 计算);
    阈值取自 config [red_target] 段。
    """
    mx = w if w > h else h
    mn = h if w > h else w
    if (mn > 0 and RING_MIN_DIAM <= mx <= RING_MAX_DIAM
            and mx <= mn * RING_MAX_ASPECT and hollow_hits <= HOLLOW_MAX_HITS):
        return "ring"
    return None

def circle_target_point(cx, cy, r, theta_deg):
    """画圈目标点(纯函数): 环心 + 半径在 θ 处的像素坐标。

    图像坐标 x 右 y 下; θ=0 指向 +x, θ 增大为视觉顺时针。
    """
    rad = math.radians(theta_deg)
    return int(round(cx + r * math.cos(rad))), int(round(cy + r * math.sin(rad)))

def circle_theta_step(theta_deg, deg_s, dt_ms, direction, err_px, pause_px):
    """画圈 θ 推进(纯函数): 误差门控 + 方向 + 360° 环绕。

    误差 > pause_px 或 dt<=0 时冻结不推进(等伺服追上); 返回 [0, 360) 区间。
    """
    if err_px > pause_px or dt_ms <= 0:
        return theta_deg
    return (theta_deg + direction * deg_s * dt_ms / 1000.0) % 360.0

def search_step(now_ms, searching, start_ms, enabled, acquired, max_ms):
    """上电搜索状态机(纯函数)。返回 (mode, searching, start_ms):
      "idle"    = 未启用 或 已捕获(不再进入搜索)
      "spin"    = 旋转中: 首次启动(记录 now_ms) / 续转(保持原 start_ms)
      "timeout" = max_ms>0 且已到点 -> 停转(不重启由调用方置 done 保证)
    max_ms=0 表示一直找, 不超时。
    """
    if not enabled or acquired:
        return "idle", False, 0
    if not searching:
        return "spin", True, now_ms
    if max_ms > 0 and time.ticks_diff(now_ms, start_ms) >= max_ms:
        return "timeout", False, 0
    return "spin", True, start_ms

def mode_pin_step(cur_active, cur_streak, raw_active, need):
    """模式引脚去抖(纯函数): 连续 need 次采样一致才切换; 中途弹回则计数清零。

    返回 (active, streak)。raw_active 的有效电平换算在调用方完成。
    """
    if raw_active == cur_active:
        return cur_active, 0
    cur_streak += 1
    if cur_streak >= need:
        return raw_active, 0
    return cur_active, cur_streak


try:
    # ---------------- pitch 上下轴: Emm42 闭环步进(UART2, 速度模式, 与 yaw 同逻辑) ----------------
    if PITCH_ENABLED:
        P_PORT = pm.get("port", 2)          # UART 控制器编号
        P_BAUD = pm.get("baud", 115200)     # 波特率
        P_TX = pm.get("tx_pin", 5)          # K230 TX  -> 驱动器② RXD
        P_RX = pm.get("rx_pin", 6)          # K230 RX  -> 驱动器② TXD
        P_ADDR = pm.get("addr", 1)          # 驱动器串口 ID
        P_ACC = pm.get("acc", 200)          # 加减速档位 0~255
        P_MIN_RPM = pm.get("min_rpm", 1)    # 最低转速: |dy| 刚越过死区时
        P_MAX_RPM = pm.get("max_rpm", 30)   # 最高转速: |dy| >= dy_full_px 时
        P_DY_FULL = pm.get("dy_full_px", 200)
        P_DEADBAND = pm.get("deadband_px", 5)          # pitch 死区 px
        P_REV = -1 if pm.get("reverse", False) else 1  # 方向翻转

        # 一个 UART 控制器只能有一组引脚, 两轴配成同一个 port 会互相覆盖
        if MOTOR_ENABLED and mv.get("port", 3) == P_PORT:
            print("  WARN: yaw 与 pitch 都配在 UART%d 上, 会互相覆盖!" % P_PORT)
            print("        → 把其中一条改到别的 UART, 或把对应轴 enabled 置 false")

        _pitch = EmmStepper(P_PORT, P_BAUD, P_TX, P_RX, P_ADDR, P_ACC)
        if pm.get("enable_on_boot", True):
            _pitch.enable(True)     # 使能电机(不使能的话收到速度指令也不转)
        _pitch.stop()               # 上电先停住, 防止误转

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
        M_COAST_EN = mv.get("coast_enabled", False)    # 丢靶预判开关(默认关, 先自检验证再启用)
        M_COAST_EDGE = mv.get("coast_edge_px", 160)    # 触发门槛: 最后 |dx| >= 此值
        M_COAST_MS = mv.get("coast_ms", 800)           # 预判持续时长(ms)

        _yaw = EmmStepper(M_PORT, M_BAUD, M_TX, M_RX, M_ADDR, M_ACC)
        if mv.get("enable_on_boot", True):
            _yaw.enable(True)       # 使能电机(不使能的话收到速度指令也不转)
        _yaw.stop()                 # 上电先停住, 防止误转

    def gimbal_send(dx, dy, dist, status):
        """每帧调用一次, 把视觉偏差送给两条轴

        dx, dy : 目标中心相对屏幕中心的像素偏差(右偏/下偏为正)
        dist   : 偏差欧氏距离(本实现未用到, 仅为接口完整保留)
        status : "track"=追踪中 / "lost"=丢靶 / "search"=上电搜索(yaw 匀速找靶)

        丢靶预判(coast): 丢靶且最后偏差 |_last_dx| >= coast_edge_px 时,
        不立即停, 而是朝最后方向续转 coast_ms 毫秒(无目标追踪)去找目标;
        超时停转, 期间一旦重新捕获(track)立即恢复常规追踪。
        pitch 轴不参与预判(丢靶即停, 与旧行为一致)。
        """
        global _last_dx, _coast_active, _coast_start
        global _acquired, _search_spin, _search_start, _search_done
        try:
            if status == "search":
                # 上电搜索(仅"启动后未捕获过目标"期间由 run_cv 调用): yaw 单方向匀速, pitch 停
                if _yaw:
                    if _search_done:
                        _yaw.stop()                     # 超时后不再重启
                    else:
                        _prev_spin = _search_spin
                        _smode, _search_spin, _search_start = search_step(
                            time.ticks_ms(), _search_spin, _search_start,
                            SEARCH_EN, False, SEARCH_MAX_MS)
                        if _smode == "spin":
                            if not _prev_spin:
                                print("SEARCH start")
                            _yaw.set_speed(SEARCH_DIR, SEARCH_RPM)
                        elif _smode == "timeout":
                            _search_done = True
                            print("SEARCH stop (timeout)")
                            _yaw.stop()
                        else:                               # idle: 未启用
                            _yaw.stop()
                if _pitch:
                    _pitch.stop()
                return
            if status == "lost":
                if _yaw and M_COAST_EN:
                    mode, _coast_active, _coast_start, _last_dx = coast_step(
                        time.ticks_ms(), _coast_active, _coast_start, _last_dx,
                        M_COAST_EDGE, M_COAST_MS)
                    if mode == "stop":
                        print("COAST stop (timeout)")
                        _yaw.stop()
                    elif mode == "idle":
                        _yaw.stop()
                    else:
                        # start / run: 朝最后方向续转, 转速=最后偏差的映射值
                        ratio = min(1.0, abs(_last_dx) / float(M_DX_FULL))
                        rpm = M_MIN_RPM + (M_MAX_RPM - M_MIN_RPM) * ratio
                        _yaw.set_speed(M_REV * (1 if _last_dx > 0 else -1), rpm)
                        if mode == "start":
                            print("COAST start (last dx=%+d)" % _last_dx)
                elif _yaw:
                    _yaw.stop()                    # 丢靶: yaw 停转(预判未启用)
                if _pitch:
                    _pitch.stop()                  # 丢靶: pitch 一律停转
                return

            # ---- 追踪中: 记录最后偏差(供丢靶预判), 复位预判状态, 标记已捕获 ----
            _last_dx = dx
            _coast_active = False
            _acquired = True     # 上电搜索: 一旦追踪过目标, 永久退出搜索

            # ---- yaw: |dx| 线性映射到转速, 符号定方向, 死区内停 ----
            if _yaw:
                if abs(dx) > M_DEADBAND:
                    ratio = min(1.0, abs(dx) / float(M_DX_FULL))
                    rpm = M_MIN_RPM + (M_MAX_RPM - M_MIN_RPM) * ratio
                    _yaw.set_speed(M_REV * (1 if dx > 0 else -1), rpm)
                else:
                    _yaw.stop()

            # ---- pitch: |dy| 线性映射到转速, 符号定方向, 死区内停(与 yaw 相同逻辑) ----
            if _pitch:
                if abs(dy) > P_DEADBAND:
                    ratio = min(1.0, abs(dy) / float(P_DY_FULL))
                    rpm = P_MIN_RPM + (P_MAX_RPM - P_MIN_RPM) * ratio
                    _pitch.set_speed(P_REV * (1 if dy > 0 else -1), rpm)
                else:
                    _pitch.stop()
        except Exception:
            pass

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
        _v = _pitch.read_bus_voltage()      # 兼作在线探测: 有回包说明驱动器和串口都通
        _pitch_info = "emm(addr=%d UART%d TX=GPIO%d RX=GPIO%d %d~%dRPM rev=%s)%s" % (
            P_ADDR, P_PORT, P_TX, P_RX, P_MIN_RPM, P_MAX_RPM,
            "Y" if P_REV < 0 else "N",
            (" online %dmV" % _v) if _v else " (no resp)")
    print("actuator: yaw=%s | pitch=%s" % (_yaw_info, _pitch_info))

    # ---------------- V7 模式引脚(GPIO49): 被供电=画圈, 其余=仅追踪 ----------------
    if PIN_ENABLED and PIN_GPIO > 0:
        try:
            from machine import Pin, FPIOA
            _fpio = FPIOA()
            _fn = getattr(FPIOA, "GPIO%d" % PIN_GPIO, None)
            if _fn is None:
                raise ValueError("FPIOA.GPIO%d not found" % PIN_GPIO)
            try:
                _fpio.set_function(PIN_GPIO, _fn, ie=1, oe=0)
            except TypeError:
                _fpio.set_function(PIN_GPIO, _fn)
            _mode_pin = Pin(PIN_GPIO, Pin.IN)
            _pv = _mode_pin.value()
            _pa = ((_pv != 0) if PIN_ACTIVE == 1 else (_pv == 0))
            print("mode pin: GPIO%d level=%d -> %s" % (PIN_GPIO, _pv,
                  "画圈" if _pa else "仅追踪"))
        except Exception as e:
            print("WARN: mode pin init failed: %s -> 永久仅追踪" % e)
            _mode_pin = None

    _gimbal_ok = True    # 执行器就绪: gimbal_send 为真实实现(供 coast_test 等自检判断)

except Exception as e:
    print("\n!!! actuator init failed: %s" % str(e))
    gimbal_send = lambda dx, dy, d, s: None


# ============================================================================
# 五、CV 矩形检测
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


# ---- V7 红色目标检测(与矩形检测同帧联合进行) ----

def _ring_geom_mec(cnt, x, y, bw, bh):
    """红环几何: 优先最小包围圆(cv2.minEnclosingCircle 可用时); 异常时兜底"""
    try:
        c, r = cv2.minEnclosingCircle(cnt)
        return int(c[0]), int(c[1]), int(r)
    except Exception:
        return x + bw // 2, y + bh // 2, (bw + bh) // 4

def _ring_geom_bbox(cnt, x, y, bw, bh):
    """红环几何兜底: 外接框中心 + (w+h)//4 作半径"""
    return x + bw // 2, y + bh // 2, (bw + bh) // 4

_RING_GEOM = _ring_geom_mec if hasattr(cv2, "minEnclosingCircle") else _ring_geom_bbox

def _count_red_at(mask, sx, sy, mw, mh):
    """3x3 窗口内是否有红色像素(sx/sy 已 clamp 在 mask 范围内)"""
    x0 = sx - 1 if sx > 0 else 0
    y0 = sy - 1 if sy > 0 else 0
    x1 = sx + 2 if sx + 2 <= mw else mw
    y1 = sy + 2 if sy + 2 <= mh else mh
    return cv2.countNonZero(mask[y0:y1, x0:x1]) > 0

def _hollow_hits(mask, cx, cy, sr, mw, mh):
    """空心验证采样: 以 (cx,cy) 为心、半径 sr 的 8 个 45° 方向点, 返回红色命中数"""
    hits = 0
    for k in range(8):
        rad = k * 0.7853981634      # k * 45° 的弧度
        sx = cx + int(sr * math.cos(rad))
        sy = cy + int(sr * math.sin(rad))
        if sx < 0:
            sx = 0
        elif sx >= mw:
            sx = mw - 1
        if sy < 0:
            sy = 0
        elif sy >= mh:
            sy = mh - 1
        if _count_red_at(mask, sx, sy, mw, mh):
            hits += 1
    return hits

def find_red_ring(frame_np, box):
    """红色侧检测: 找红环(画圈用), 返回 ring=(cx,cy,r) 或 None。

    box 有效 -> 红色在矩形外扩 ROI_MARGIN_PX 的区域内找(省算力);
    box 无效 -> 全帧找(保证"单独红环"可见)。
    ring 选取: 有矩形优先"环心在矩形内"(point_in_quad), 否则取半径最大者。
    """
    mw = SENSOR_W
    mh = SENSOR_H
    if box:
        xs = [p[0] for p in box]
        ys = [p[1] for p in box]
        ox = min(xs) - ROI_MARGIN_PX
        oy = min(ys) - ROI_MARGIN_PX
        ex = max(xs) + ROI_MARGIN_PX
        ey = max(ys) + ROI_MARGIN_PX
        if ox < 0: ox = 0
        if oy < 0: oy = 0
        if ex > mw: ex = mw
        if ey > mh: ey = mh
    else:
        ox = 0
        oy = 0
        ex = mw
        ey = mh
    rw = ex - ox
    rh = ey - oy
    if rw < 8 or rh < 8:
        return None
    mask = cv2.inRange(frame_np[oy:ey, ox:ex], RED_LOW, RED_HIGH)
    contours = get_contours(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    rings = []
    for cnt in contours:
        x, y, bw, bh = cv2.boundingRect(cnt)
        if bw <= 0 or bh <= 0:
            continue
        mx = bw if bw > bh else bh
        sr = (mx // 2) * HOLLOW_SAMPLE_PCT // 100
        cx = x + bw // 2
        cy = y + bh // 2
        hits = _hollow_hits(mask, cx, cy, sr, rw, rh) if sr > 0 else 0
        if classify_blob(bw, bh, hits) == "ring":
            rx, ry, rr = _RING_GEOM(cnt, x, y, bw, bh)
            rings.append((rx + ox, ry + oy, rr))
    # ---- 红环选取: 有矩形优先环心在矩形内; 否则取半径最大者 ----
    ring = None
    if rings:
        if box:
            for cand in rings:
                if point_in_quad(cand[0], cand[1], box):
                    if ring is None or cand[2] > ring[2]:
                        ring = cand
        if ring is None:
            for cand in rings:
                if ring is None or cand[2] > ring[2]:
                    ring = cand
    return ring


# ============================================================================
# 六、YOLO 检测支持
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
# 六点五、触屏中心校准 —— 触摸按键移动绿色校准点(追踪收敛点), 保存写回 config
# ============================================================================

def _calib_rects(ox, oy, sw, sh, bw):
    """生成 6 个触摸按钮的命中矩形(LCD 坐标): 上/下(左缘) 左/右(右缘) dot隐藏/S保存(顶中)"""
    m = 6
    y1 = sh // 2 - bw - 6
    y2 = y1 + bw + 12
    xl = ox + m
    xr = ox + sw - m - bw
    xc = ox + sw // 2
    return (
        ("up",    xl, oy + y1, xl + bw, oy + y1 + bw),
        ("down",  xl, oy + y2, xl + bw, oy + y2 + bw),
        ("left",  xr, oy + y1, xr + bw, oy + y1 + bw),
        ("right", xr, oy + y2, xr + bw, oy + y2 + bw),
        ("dot",   xc - bw - 4, oy + m, xc - 4, oy + m + bw),
        ("save",  xc + 4, oy + m, xc + bw + 4, oy + m + bw),
    )

def calib_hit(x, y, rects):
    """触摸点命中的按钮名; 未命中返回 None(边界视为命中)"""
    for name, x0, y0, x1, y1 in rects:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return None

def calib_step(vis, btn, hold_t0, now, pressed, was_pressed, hit, repeat_ms):
    """校准 UI 触摸状态机(每帧一次)。返回 (vis, btn, hold_t0, action)。

    规则: 隐藏态仅"新触摸边沿"唤出(本次触摸被消费, 不发动作; 防点 dot 未松手即弹回);
          可见态新按/换钮立即发一次; 方向键按住每 repeat_ms 连发; dot/save 单发。
    action ∈ None / "up" / "down" / "left" / "right" / "dot" / "save"
    """
    if not vis:
        if pressed and not was_pressed:
            return True, None, now, None
        return False, None, hold_t0, None
    if not pressed:
        return True, None, hold_t0, None
    if hit is None or hit != btn:
        return True, hit, now, hit
    if (hit == "up" or hit == "down" or hit == "left" or hit == "right") \
            and time.ticks_diff(now, hold_t0) >= repeat_ms:
        return True, hit, now, hit
    return True, hit, hold_t0, None

def _replace_json_int(line, val):
    """把一行 json 里第一个整数 token 换成 val(保留键/缩进/尾逗号); 无整数 token 返回 None"""
    i = line.find(":")
    if i < 0:
        return None
    j = i + 1
    n = len(line)
    while j < n and line[j] == " ":
        j += 1
    k = j
    if k < n and line[k] == "-":
        k += 1
    s = k
    while k < n and "0" <= line[k] <= "9":
        k += 1
    if k == s:
        return None
    return line[:j] + str(val) + line[k:]

def calib_save_text(src, lox, loy):
    """config.json 文本 → 替换 laser_offset_x/y 两行的整数值; 其余行逐字节保留。

    成功返回新文本; 两个键未同时找到返回 None(调用方按失败处理, 不写盘)。"""
    lines = src.split("\n")
    hit = 0
    for i in range(len(lines)):
        s = lines[i].strip()
        if s.startswith('"laser_offset_x"') or s.startswith('"laser_offset_y"'):
            val = lox if s.startswith('"laser_offset_x"') else loy
            nl = _replace_json_int(lines[i], val)
            if nl is None:
                return None
            lines[i] = nl
            hit += 1
    if hit != 2:
        return None
    return "\n".join(lines)

def calib_save_file(path, lox, loy):
    """读 config → 文本替换 → 备份(.bak) → 覆盖写 → 读回校验。返回 (ok, msg)。

    任何失败不动原文件(备份为上一版)。无 path(未从文件加载配置)时直接失败。"""
    if not path:
        return False, "no config path"
    try:
        with open(path, "r") as f:
            src = f.read()
        dst = calib_save_text(src, lox, loy)
        if dst is None:
            return False, "laser_offset keys not found"
        try:
            with open(path + ".bak", "w") as f:
                f.write(src)
        except Exception:
            pass
        with open(path, "w") as f:
            f.write(dst)
        try:
            import ujson as _j
        except ImportError:
            import json as _j
        with open(path, "r") as f:
            _j.load(f)
        return True, "saved (%d,%d)" % (lox, loy)
    except Exception as e:
        return False, str(e)

def _calib_arrow(frame_np, cx, cy, name):
    """方向按钮符号: 三角描边(上/下/左/右)"""
    s = 13
    if name == "up":
        p = ((cx, cy - s), (cx - s, cy + s // 2), (cx + s, cy + s // 2))
    elif name == "down":
        p = ((cx, cy + s), (cx - s, cy - s // 2), (cx + s, cy - s // 2))
    elif name == "left":
        p = ((cx - s, cy), (cx + s // 2, cy - s), (cx + s // 2, cy + s))
    else:
        p = ((cx + s, cy), (cx - s // 2, cy - s), (cx - s // 2, cy + s))
    cv2.line(frame_np, p[0], p[1], (0, 200, 0), 2)
    cv2.line(frame_np, p[1], p[2], (0, 200, 0), 2)
    cv2.line(frame_np, p[2], p[0], (0, 200, 0), 2)

def calib_draw(frame_np, rects, lox, loy, dot_r):
    """绘制校准 UI: 6 个按钮(框+符号) + 绿色校准点(画面坐标 CENTER+offset)"""
    for name, x0, y0, x1, y1 in rects:
        a0 = x0 - DISPLAY_X
        b0 = y0 - DISPLAY_Y
        a1 = x1 - DISPLAY_X
        b1 = y1 - DISPLAY_Y
        cv2.line(frame_np, (a0, b0), (a1, b0), (0, 140, 0), 2)
        cv2.line(frame_np, (a1, b0), (a1, b1), (0, 140, 0), 2)
        cv2.line(frame_np, (a1, b1), (a0, b1), (0, 140, 0), 2)
        cv2.line(frame_np, (a0, b1), (a0, b0), (0, 140, 0), 2)
        mx = (a0 + a1) // 2
        my = (b0 + b1) // 2
        if name == "dot":
            cv2.circle(frame_np, (mx, my), 12, (0, 200, 0), 2)
            cv2.circle(frame_np, (mx, my), 3, (0, 200, 0), -1)
        elif name == "save":
            cv2.putText(frame_np, "S", (mx - 8, my + 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        else:
            _calib_arrow(frame_np, mx, my, name)
    gx = CENTER_X + lox
    gy = CENTER_Y + loy
    cv2.circle(frame_np, (gx, gy), dot_r, (0, 255, 0), -1)
    cv2.circle(frame_np, (gx, gy), dot_r + 2, (255, 255, 255), 1)

CALIB_BTNS = _calib_rects(DISPLAY_X, DISPLAY_Y, SENSOR_W, SENSOR_H, 56)

# ============================================================================
# 七、CV 模式主循环
# ============================================================================

def run_cv():
    global LOX, LOY                  # 触屏中心校准运行时更新激光偏移
    sensor = None
    frame_id = lost_count = 0
    last_box = None
    last_ring = None                 # 红环检测帧间复用
    _circle_on = False               # 画圈模式(模式引脚去抖后)
    _pin_streak = 0
    _theta = None                    # 画圈 θ(None=未起始; 见环时按当前方位角起转)
    _theta_tick = time.ticks_ms()    # θ 推进的帧间计时基准
    last_dist = 0.0                  # 上帧瞄准偏差(画圈误差门控用)
    _calib_tp = None                 # 触摸句柄(None=不可用/未启用)
    _calib_vis = CALIB_SHOW_DOT      # 绿点+按钮 显隐(dot按钮切换; 隐藏后触摸任意处唤出)
    _calib_btn = None                # 按住中的按钮名
    _calib_hold = 0                  # 按住起始时刻(连发计时)
    _calib_pre = False               # 上帧是否有触点(唤醒边沿判定)
    fps = fps_count = 0
    fps_tick = time.ticks_ms()

    try:
        print("boot")
        os.exitpoint(os.EXITPOINT_ENABLE)
        # 采集端对齐官方例程(rgb888_find_circles_1.py): 显式 CSI 端口 + 1280x720@90(16:9) 模式提示。
        # 旧写法(无 id + 1280x960)实测画面视野偏移, 官方例程同板对照正常 —— 勿回退。
        sensor = Sensor(id=CSI_ID, width=1280, height=720, fps=FPS)
        sensor.reset()
        sensor.set_framesize(width=SENSOR_W, height=SENSOR_H, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
        print("sensor ok")
        # ---- 触屏中心校准: 触摸初始化(失败自动禁用, 不影响主程序) ----
        if CALIB_ENABLED:
            try:
                from machine import TOUCH
                if CALIB_TOUCH_ROT >= 0:
                    _calib_tp = TOUCH(0, rotate=CALIB_TOUCH_ROT)
                else:
                    _calib_tp = TOUCH(0)
                print("calib: touch ok")
            except Exception as _e:
                print("calib: touch unavailable (%s) -- disabled" % _e)
                _calib_tp = None
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

            # ---- V7.7 触屏中心校准: 触摸扫描 → 移动绿点/显隐/保存(先于本帧瞄准生效) ----
            if _calib_tp is not None:
                try:
                    _pts = _calib_tp.read()
                    if _pts is None:
                        _pts = ()
                except Exception as _e:
                    print("calib: touch read fail (%s) -- disabled" % _e)
                    _calib_tp = None
                    _pts = ()
                _pressed = len(_pts) > 0
                _hit = calib_hit(_pts[0].x, _pts[0].y, CALIB_BTNS) if _pressed else None
                _calib_vis, _calib_btn, _calib_hold, _act = calib_step(
                    _calib_vis, _calib_btn, _calib_hold, time.ticks_ms(),
                    _pressed, _calib_pre, _hit, CALIB_REPEAT)
                _calib_pre = _pressed
                if _act == "up":
                    LOY -= CALIB_STEP
                elif _act == "down":
                    LOY += CALIB_STEP
                elif _act == "left":
                    LOX -= CALIB_STEP
                elif _act == "right":
                    LOX += CALIB_STEP
                elif _act == "dot":
                    _calib_vis = False
                elif _act == "save":
                    _ok, _msg = calib_save_file(CFG_PATH, LOX, LOY)
                    print("calib: save %s %s" % ("OK" if _ok else "FAIL", _msg))
                if LOX < -CALIB_XMAX:
                    LOX = -CALIB_XMAX
                elif LOX > CALIB_XMAX:
                    LOX = CALIB_XMAX
                if LOY < -CALIB_YMAX:
                    LOY = -CALIB_YMAX
                elif LOY > CALIB_YMAX:
                    LOY = CALIB_YMAX

            # ---- V7 模式引脚采样 + 去抖: 被供电=画圈, 其余=仅追踪 ----------------
            if _mode_pin is not None:
                _raw = _mode_pin.value()
                _raw_on = ((_raw != 0) if PIN_ACTIVE == 1 else (_raw == 0))
                _prev_on = _circle_on
                _circle_on, _pin_streak = mode_pin_step(_circle_on, _pin_streak,
                                                        _raw_on, PIN_DEBOUNCE)
                if _circle_on != _prev_on:
                    _theta = None                    # 进出画圈沿重置起始角
                    print("MODE->%s" % ("circle" if _circle_on else "track"))

            # ---- 检测(每检测帧): 矩形 + 红色 联合 ----------------
            if frame_id % DETECT_EVERY == 0 or not last_box:
                box, contour_count, area_count = find_paper_box(frame_np, last_box)
                ring = find_red_ring(frame_np, box)
            else:
                box, ring = last_box, last_ring

            # ---- 目标选择: 画圈(环优先) > 追踪(矩形中心) ----------------
            now2 = time.ticks_ms()
            dtm = time.ticks_diff(now2, _theta_tick)
            _theta_tick = now2
            kind, tx, ty = "none", 0, 0
            if _circle_on and ring is not None:
                rcx, rcy, rr = ring
                if box is not None:
                    draw_box(frame_np, box)          # 仅绘制(目标=环上点)
                if _theta is None:
                    _theta = math.degrees(math.atan2(CENTER_Y - rcy, CENTER_X - rcx)) % 360.0
                _theta = circle_theta_step(_theta, CIRC_SPEED, dtm, CIRC_DIR, last_dist, CIRC_PAUSE)
                tx, ty = circle_target_point(rcx, rcy, rr + RADIUS_BIAS, _theta)
                kind = "ring"
            elif box is not None:
                rcx, rcy, _dcx, _dcy = draw_box(frame_np, box)   # 画框 + 矩形中心
                tx, ty = rcx, rcy
                kind = "rect"

            # ---- 执行(保持既有 gimbal_send track 语义) ----------------
            if kind != "none":
                lost_count = 0
                if box is not None:
                    last_box = box
                last_ring = ring
                dx, dy = tx - CENTER_X - LOX, ty - CENTER_Y - LOY
                dist = math.sqrt(dx * dx + dy * dy)
                last_dist = dist
                gimbal_send(int(dx), int(dy), dist, "track")
                # OSD: 红环/最终瞄准点 标记 + 目标源文字
                if SHOW_OSD:
                    if ring is not None:
                        cv2.circle(frame_np, (ring[0], ring[1]), ring[2], (255, 128, 0), 1)
                    cv2.circle(frame_np, (tx, ty), 4, (0, 255, 255), 1)
                    cv2.putText(frame_np, "[%s] dx:%+d dy:%+d" % (kind, dx, dy),
                                (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                if frame_id % PRINT_EVERY == 0:
                    print("OK [%s] d=(%d,%d) dist=%.0f" % (kind, dx, dy, dist))
            else:
                lost_count += 1
                if lost_count > 2:
                    last_box = None
                    if SEARCH_EN and not _acquired:
                        gimbal_send(0, 0, 0, "search")   # 上电搜索: 未捕获期 yaw 匀速旋转找靶
                    else:
                        gimbal_send(0, 0, 0, "lost")
                    if frame_id % PRINT_EVERY == 0:
                        print("LOST")

            # FPS
            fps_count += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, fps_tick) >= 1000:
                fps = fps_count * 1000 // time.ticks_diff(now, fps_tick)
                fps_count = 0
                fps_tick = now
            cv2.putText(frame_np, "FPS:%d" % fps, (4, SENSOR_H - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # 触屏校准 UI(绿点+按钮, 可隐藏)
            if _calib_vis:
                calib_draw(frame_np, CALIB_BTNS, LOX, LOY, CALIB_DOT_R)

            Display.show_image(frame, x=DISPLAY_X, y=DISPLAY_Y)
            if frame_id % GC_EVERY == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        print("error:", e)
    finally:
        if _calib_tp is not None:
            try:
                _calib_tp.deinit()
            except Exception:
                pass
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
# 八、YOLO 模式主循环
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
# 九、入口 — 模式切换
# ============================================================================

print("=" * 50)
print("mode: %s | pitch: %s | yaw: %s" %
      (MODE.upper(), PITCH_ENABLED, MOTOR_ENABLED))
print("=" * 50)

# 丢靶预判自检(不跑视觉): 模拟左右两侧丢靶 + 中途重捕获, 单独验证续追行为
# 本块先于 test_mode 判断执行: 与 motor/pitch_motor 的 test_mode 不要同时开(同开时本项优先)。
if mv.get("coast_test", False):
    if not (MOTOR_ENABLED and _yaw and _gimbal_ok):
        print(">>> coast_test=true, 但 yaw 电机未启用 / 初始化未成功 → 跳过丢靶预判自检")
    else:
        M_COAST_EN = True    # 测试模式强制启用预判(不受 coast_enabled 影响)
        print(">>> COAST TEST (丢靶预判自检, 不跑视觉; Ctrl+C 退出)")
        print("  预期: 每侧丢靶后 yaw 朝该侧续转约 %dms 再停; 中途'重捕获'应立即停" % M_COAST_MS)
        for _side, _dx in (("右", 250), ("左", -250)):
            print("---- [%s] 丢靶续追 (最后偏差 dx=%+d) ----" % (_side, _dx))
            gimbal_send(_dx, 0, _dx, "track")      # 模拟"最后看到目标"的一帧
            time.sleep_ms(100)
            _c0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), _c0) < M_COAST_MS + 1200:
                gimbal_send(0, 0, 0, "lost")       # 连续丢靶 → 观察 COAST start / COAST stop 打印
                time.sleep_ms(20)
            time.sleep_ms(1200)                    # 停顿, 确认已停稳
        print("---- [重捕获] 续追中途目标回到中心 → 应立即停 ----")
        gimbal_send(250, 0, 250, "track")
        time.sleep_ms(100)
        _c0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), _c0) < M_COAST_MS // 2:
            gimbal_send(0, 0, 0, "lost")
            time.sleep_ms(20)
        gimbal_send(0, 0, 0, "track")              # 目标回到画面中心(死区内) → 停转, 同时复位预判
        time.sleep_ms(100)
        _c0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), _c0) < 1000:
            gimbal_send(0, 0, 0, "lost")           # 继续丢靶: 若复位失效, 这里会暴露(继续转/打印 COAST stop)
            time.sleep_ms(20)
        print("  验证点: 上一步后云台应保持静止, 且无 COAST start/stop 打印")
        print(">>> COAST TEST done (Ctrl+C 退出)")
        while True:
            time.sleep_ms(1000)

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

# pitch 步进测试模式(不跑视觉): 在线探测 + 持续旋转自检(与 yaw 相同逻辑)
if PITCH_ENABLED and pm.get("test_mode", False) and _pitch:
    import time as _t

    _rpm = pm.get("test_rpm", 10)
    _dirn = P_REV if P_REV else 1
    print(">>> PITCH TEST (Emm42 步进, 持续旋转自检)")
    print("  串口: UART%d @%dbps, ID=%d, 校验=固定0x6B" % (P_PORT, P_BAUD, P_ADDR))
    print("  接线: K230 TX=GPIO%d -> 驱动器RXD,  RX=GPIO%d -> 驱动器TXD" % (P_TX, P_RX))
    print("  共地 + 驱动器 V+ 独立供电 7~32V(勿接 K230 5V)")
    _v = _pitch.read_bus_voltage()
    if _v:
        print("  在线探测: OK, 母线电压 %dmV" % _v)
    else:
        print("  在线探测: 无应答")

        def _hx(b):
            return " ".join("%02X" % x for x in b)

        _sent, _raw = _pitch.raw_probe()
        if not _raw:
            print("  RX 原始字节: (无) —— 一个字节都没收到")
            print("    → K230 发出去了但没人回。依次查:")
            print("      1. 驱动器 OLED 是否正常 / 显示什么(Waiting V+ Power!?)")
            print("      2. 共地: K230 GND 与驱动器电源 GND 是否连通")
            print("      3. 交叉接线: TX GPIO%d -> 驱动器RXD, RX GPIO%d -> 驱动器TXD" % (P_TX, P_RX))
            print("      4. 驱动器菜单: 串口功能=UART_FUN, 串口ID=%d, 波特率=%d," % (P_ADDR, P_BAUD))
            print("         校验方式=固定0x6B, 应答=Receive")
        elif _raw == _sent:
            print("  RX 原始字节: %s   <- 正是我们自己发出去的帧" % _hx(_raw))
            print("    → 回环测试通过: K230 的 UART%d(GPIO%d/GPIO%d) 收发都正常" % (P_PORT, P_TX, P_RX))
            print("      拔掉 TX/RX 之间的短接线, 接驱动器")
        else:
            print("  RX 原始字节: %s" % _hx(_raw))
            print("    (本机发出的是: %s)" % _hx(_sent))
            print("    → 收到字节但不是自身回环: 多半驱动器「校验方式」不是 固定0x6B")
    _pitch.enable(True)
    _t.sleep_ms(100)
    print("\n  持续%s %dRPM (reverse=%s; Ctrl+C 退出)" %
          ("正转" if _dirn > 0 else "反转", _rpm, pm.get("reverse", False)))
    print("  电机不动 → 先看上面'在线探测'是否 OK")
    while True:
        _pitch.set_speed(_dirn, _rpm)     # 持续重发, 防止丢帧后停住
        _t.sleep_ms(500)

if MODE == "yolo":
    if _yolo_ok():
        run_yolo()
    else:
        print("YOLO libs missing, fallback to CV")
        run_cv()
else:
    run_cv()
