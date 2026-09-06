"""
K230 物体追踪 — 双轴舵机云台
============================

═══════════════════════════════════════════════════════════════
  功能说明
═══════════════════════════════════════════════════════════════
  摄像头采集 → 物体检测 → 计算物体中心与屏幕中心偏移 → 控制舵机
  - CV 模式:  矩形检测, 偏移 = 矩形中心 - 屏幕中心
  - YOLO 模式: 目标检测, 偏移 = 识别框中心 - 屏幕中心

═══════════════════════════════════════════════════════════════
  配置速查 (config.json, 修改后重启生效)
═══════════════════════════════════════════════════════════════
  [detection.mode]        →  "cv"(矩形) / "yolo"(KPU目标检测)
  [camera.sensor_width]   →  摄像头宽度
  [camera.sensor_height]  →  摄像头高度
  [center.x / center.y]   →  屏幕中心像素坐标(追踪目标点)
  [rectangle.*]           →  CV矩形检测阈值
  [yolo.*]                →  YOLO模型/置信度等
  [servo.enabled]         →  true=K230直控FashionStar总线舵机
  [servo.yaw_id]          →  左右轴舵机ID
  [servo.pitch_id]        →  上下轴舵机ID
  [servo.yaw_vel_scale]   →  每帧角度增量系数(越大转越快)
  [servo.pitch_k]         →  上下轴每帧增量系数
  [servo.yaw_reverse]     →  左右方向翻转(true/false)
  [servo.pitch_reverse]   →  上下方向翻转(true/false)
  [servo.yaw_min/max]     →  左右轴角度限位
  [servo.pitch_min/max]   →  上下轴角度限位
  [servo.deadband_px]     →  死区(像素偏差小于此值不动作)
  [servo.center_on_boot]  →  上电回中0°
═══════════════════════════════════════════════════════════════

接线: K230 UART2 TX(GPIO5/Pin11)串1kΩ接舵机信号线, RX(GPIO6/Pin13)直连, 共地, 舵机独立供电
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

# ---- 舵机配置 ----
sv = _get(CFG, "servo", {})
SERVO_ENABLED = sv.get("enabled", False)

# ============================================================================
# 三、FSUS 协议驱动 (整合自 fs_servo.py)
#    协议: Fashion Star Uart Servo, 小端字节序, 角度单位 0.1°
# ============================================================================

# ---- FSUS 指令 ID ----
CMD_PING = 1
CMD_READ = 3
CMD_WRITE = 4
CMD_SET_ANGLE = 8
CMD_DAMPING = 9
CMD_QUERY_ANGLE = 10
CMD_SET_BY_INTERVAL = 11
CMD_SET_BY_VELOCITY = 12
CMD_SET_MTURN = 13
CMD_QUERY_MTURN = 16
CMD_RESET_MTURN = 17
CMD_MONITOR = 22
CMD_ORIGIN = 23
CMD_STOP = 24
CMD_SYNC = 25

# ---- 参数地址 ----
ADDR_SERVO_ID = 34
ADDR_BAUDRATE = 36
ADDR_ANGLE_LIMIT_SW = 48
ADDR_ANGLE_LIMIT_HIGH = 51
ADDR_ANGLE_LIMIT_LOW = 52


class FSServoBus:
    """一条舵机总线 = 一个 UART, 总线上可挂多个舵机(靠ID区分)"""

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
        """发一帧; wait_resp=False 纯写。成功返回(cmdId, content), 失败 None"""
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

    def set_angle_mturn(self, angle_deg, interval_ms=0, power=0):
        """多圈设角度; angle_deg 支持 ±360° 及更大范围"""
        a = int(angle_deg * 10)
        self.bus.txrx(CMD_SET_MTURN, bytes([
            self.id,
            a & 0xFF, (a >> 8) & 0xFF, (a >> 16) & 0xFF, (a >> 24) & 0xFF,
            interval_ms & 0xFF, (interval_ms >> 8) & 0xFF,
            (interval_ms >> 16) & 0xFF, (interval_ms >> 24) & 0xFF,
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

    def write_param(self, addr, data):
        """写参数, data 为 list/bytes"""
        self.bus.txrx(CMD_WRITE,
                      bytes([self.id, addr, len(data)]) + bytes(data),
                      wait_resp=False)


def sync_set_angles(bus, targets, interval_ms=0, power=0):
    """同步指令: 一帧驱动多舵机同时动作。targets=[(id, angle), ...]"""
    n = len(targets)
    c = bytearray([8, 7, n])  # 子cmd=8, 每舵机7B
    for sid, ang in targets:
        a = int(max(-180.0, min(180.0, ang)) * 10)
        c += bytes([
            sid & 0xFF,
            a & 0xFF, (a >> 8) & 0xFF,
            interval_ms & 0xFF, (interval_ms >> 8) & 0xFF,
            power & 0xFF, (power >> 8) & 0xFF])
    bus.txrx(CMD_SYNC, c, wait_resp=False)


# ============================================================================
# 四、舵机控制 — 初始化 + servo_send
# ============================================================================

if SERVO_ENABLED:
    _servo_inited = False
    try:
        _sport = sv.get("port", 2)
        _sbus = FSServoBus(uart_id=_sport, baud=sv.get("baud", 115200),
                           tx_pin=sv.get("tx_pin", 5), rx_pin=sv.get("rx_pin", 6))
        _yaw = FSServo(_sbus, sv.get("yaw_id", 1))      # 左右轴
        _pitch = FSServo(_sbus, sv.get("pitch_id", 0))  # 上下轴
        _servo_inited = True

        # ---- 控制参数 ----
        YAW_VEL_SCALE = sv.get("yaw_vel_scale", 0.08)   # 左右: 每帧角度 += scale × dx
        PITCH_K = sv.get("pitch_k", 0.02)               # 上下: 每帧角度 += k × dy
        YAW_REV = -1.0 if sv.get("yaw_reverse", False) else 1.0
        PITCH_REV = -1.0 if sv.get("pitch_reverse", False) else 1.0
        YAW_MIN = sv.get("yaw_min", -360)
        YAW_MAX = sv.get("yaw_max", 360)
        PITCH_MIN = sv.get("pitch_min", -30)
        PITCH_MAX = sv.get("pitch_max", 30)
        S_INTERVAL = sv.get("interval_ms", 40)          # 舵机行程周期(ms)
        S_POWER = sv.get("power", 0)                     # 功率上限mW, 0=不限
        S_DEADBAND = sv.get("deadband_px", 5)            # 死区像素
        S_LOST_HOLD = sv.get("lost_hold", True)          # 丢靶保持 or 阻尼

        # 角度积分状态 (上电时云台应居中=0°)
        _yaw_ang = 0.0
        _pitch_ang = 0.0

        def _s_clamp(v, lo, hi):
            return lo if v < lo else (hi if v > hi else v)

        def servo_send(dx, dy, dist, status):
            """视觉追踪控制 — 增量式
               dx=左右偏移(像素), dy=上下偏移(像素)
               yaw(左右轴) 用 dx 控制, pitch(上下轴) 用 dy 控制
               偏差越大, 每帧增量越大 → 转越快"""
            global _yaw_ang, _pitch_ang
            try:
                if status == "lost":
                    if not S_LOST_HOLD:
                        _yaw.stop(2)   # 丢靶→阻尼
                        _pitch.stop(2)
                    return

                # ---- yaw 左右轴: 用 dx 增量控制 ----
                if abs(dx) > S_DEADBAND:
                    incr = YAW_VEL_SCALE * dx
                    if abs(dx) > 50:
                        incr *= 1 + (abs(dx) - 50) / 80  # 大偏差加速
                    _yaw_ang = _s_clamp(_yaw_ang + YAW_REV * incr, YAW_MIN, YAW_MAX)
                    _yaw.set_angle_mturn(_yaw_ang, S_INTERVAL, S_POWER)
                else:
                    _yaw.stop(1)   # 死区内锁力保持

                # ---- pitch 上下轴: 用 dy 增量控制 ----
                if abs(dy) > S_DEADBAND:
                    _pitch_ang = _s_clamp(_pitch_ang + PITCH_REV * PITCH_K * dy, PITCH_MIN, PITCH_MAX)
                    _pitch.set_angle(_pitch_ang, S_INTERVAL, S_POWER)

            except Exception:
                pass

        # 上电回中
        if sv.get("center_on_boot", False):
            sync_set_angles(_sbus, [(_yaw.id, 0.0), (_pitch.id, 0.0)], 2000, S_POWER)

        # 读取并解除角度限制
        try:
            lim_sw = _yaw.read_param(48)
            lim_hi = _yaw.read_param(51)
            lim_lo = _yaw.read_param(52)
            if lim_sw is None:
                print("  yaw limit: 读取失败")
            else:
                hi = (lim_hi - 65536 if lim_hi >= 32768 else lim_hi) / 10.0
                lo = (lim_lo - 65536 if lim_lo >= 32768 else lim_lo) / 10.0
                print("  yaw limit: sw=%s hi=%.1f° lo=%.1f°" % ("ON" if lim_sw else "OFF", hi, lo))
                if lim_sw:
                    _yaw.write_param(48, [0])
                    _yaw.write_param(51, [0x10, 0x0E])   # +360° = 3600 = 0x0E10
                    _yaw.write_param(52, [0xF0, 0xF1])   # -360° = -3600 = 0xF1F0
                    print("  -> 已解除角度限制, 设为±360°(断电重启生效)")
        except Exception as _e:
            print("  读取限制参数失败:", _e)

        print("servo: yaw_id=%d pitch_id=%d scale=%.3f k=%.3f rev=%s/%s range=%d~%d port=%d%s" %
              (_yaw.id, _pitch.id, YAW_VEL_SCALE, PITCH_K,
               "Y" if YAW_REV < 0 else "N", "Y" if PITCH_REV < 0 else "N",
               YAW_MIN, YAW_MAX, _sport,
               " online" if _yaw.ping() else " (ping no resp)"))
    except Exception as e:
        print("\n!!! servo init failed: %s" % str(e))
        servo_send = lambda dx, dy, d, s: None
else:
    servo_send = lambda dx, dy, d, s: None


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
# 七、CV 模式主循环
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
        sensor.run()
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
                servo_send(int(dx), int(dy), dist, "track")
                if frame_id % 60 == 0:
                    print("OK  center=(%d,%d)  d=(%d,%d)  dist=%.0f" % (cx, cy, dx, dy, dist))
            else:
                lost_count += 1
                if lost_count > 2:
                    last_box = None
                    servo_send(0, 0, 0, "lost")
                    if frame_id % 60 == 0:
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
            if frame_id % 30 == 0:
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
                servo_send(int(dx), int(dy), dist, "track")
                lost_count = 0

                label = LABELS[cls_ids[best_idx]] if cls_ids[best_idx] < len(LABELS) else "?"
                pl.osd_img.draw_string_advanced(4, 4, 24,
                    "%s %.2f | dx:%+d dy:%+d" % (label, confs[best_idx], dx, dy),
                    color=(0, 255, 0))
            else:
                lost_count += 1
                if lost_count > 2:
                    servo_send(0, 0, 0, "lost")
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
print("mode: %s  servo: %s" % (MODE.upper(), SERVO_ENABLED))
print("=" * 50)

if MODE == "yolo":
    if _yolo_ok():
        run_yolo()
    else:
        print("YOLO libs missing, fallback to CV")
        run_cv()
else:
    run_cv()
