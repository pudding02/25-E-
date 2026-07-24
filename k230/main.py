"""
K230 矩形靶标追踪
================

═══════════════════════════════════════════════════════════════
  功能开关速查（在 config.json 中切换，无需改代码）
═══════════════════════════════════════════════════════════════
  [kalman.enabled]      →  卡尔曼滤波(EMA)，平滑去抖
  [uart.enabled]        →  串口输出 dx,dy,dist,status → 云台
  [detect_resolution.enabled] → 降采样检测(320x240)提速
  [detection.mode]      → "cv" / "yolo" / "hybrid"
  [camera.sensor_width/height] → 摄像头分辨率
  [rectangle.*]         → 矩形检测阈值，现场调参入口
═══════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════
  性能设计原则（K230 MicroPython 优化经验）
═══════════════════════════════════════════════════════════════
  1. 热路径零分支 —— 可选功能用 lambda 注入，避免每帧 if
  2. 全局变量直读 —— 模块级变量比对象属性/字典快5-10倍
  3. 返回元组不用dict —— 避免热路径内存分配触发GC
  4. 检测算法纯函数 —— 不封装成类，函数直接读全局参数
═══════════════════════════════════════════════════════════════

配置文件: config.json (与main.py同目录)
"""

import time, os, gc, math
import cv2
from media.sensor import *
from media.display import *
from media.media import *

# ============================================================================
# 一、配置加载 — import阶段执行一次，不影响热路径速度
# ============================================================================

def _load_json(paths):
    """从多个候选路径加载JSON配置，返回dict或{}"""
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
                if not k.startswith("_"):
                    cfg[k] = {sk: sv for sk, sv in v.items()
                              if isinstance(v, dict) and not sk.startswith("_")}
                    if not isinstance(v, dict):
                        cfg[k] = v
            print("config: %s loaded" % p)
            return cfg
        except Exception:
            pass
    print("config: using defaults")
    return {}

def _get(cfg, key, default):
    """安全获取配置值，cfg为空时返回默认值"""
    return cfg.get(key, default) if cfg else default

CFG = _load_json(["/sdcard/config.json", "config.json"])

# ============================================================================
# 二、运行参数 — 全部从config.json读取，模块级全局直接赋值
#    修改方式: 改config.json → 重启main.py → 即刻生效
#    不要在此处直接改值，config.json是唯一的"调参面板"
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
DETECT_EVERY  = _get(_get(CFG, "detection", {}), "detect_every", 1)
# 跳帧检测: 1=每帧, 2=隔帧(帧率翻倍但响应变慢), 3=每3帧
MODE          = _get(_get(CFG, "detection", {}), "mode", "cv")
# 模式: "cv"=经典视觉, "yolo"=KPU推理, "hybrid"=CV优先

# ---- 调试 ----
PRINT_EVERY = _get(_get(CFG, "debug", {}), "print_every", 60)
# 每N帧打印一次检测信息到串口终端，60 ≈ 每2秒(30fps)
GC_EVERY    = _get(_get(CFG, "debug", {}), "gc_every", 30)
# 每N帧执行一次垃圾回收，太频繁拖慢帧率、太稀疏内存堆积

# ---- 矩形检测阈值（现场调参主要入口） ----
r = _get(CFG, "rectangle", {})
WHITE_LOW  = tuple(r.get("white_low",  [135, 135, 115]))
WHITE_HIGH = tuple(r.get("white_high", [255, 255, 255]))
# 白色掩膜RGB范围 — 检测靶标白色内部区域
BLACK_LOW  = tuple(r.get("black_low",  [0, 0, 0]))
BLACK_HIGH = tuple(r.get("black_high", [85, 85, 85]))
# 黑色边框RGB范围 — 验证矩形四边是否为黑色
MIN_AREA   = r.get("min_area", 3500)
MAX_AREA   = r.get("max_area", 180000)
# 矩形面积范围(像素²) — 过滤太小(噪点)和太大(全画面)
MIN_ASPECT = r.get("min_aspect", 1.05)
MAX_ASPECT = r.get("max_aspect", 2.80)
TARGET_ASPECT = r.get("target_aspect", 1.55)
# 宽高比: 正方形≈1.0, 横长方形>1.0, 竖长方形<1.0
BORDER_EXPAND_X = r.get("border_expand_x", 16)
BORDER_EXPAND_Y = r.get("border_expand_y", 16)
# 检测框向外扩展像素，用于黑边/白心验证
BLACK_CHECK_STEP = r.get("black_check_step", 4)
MIN_BLACK_HITS   = r.get("min_black_hits", 3)
# 黑边验证: 四边中至少3边有足够黑色像素
CENTER_WHITE_STEP = r.get("center_white_step", 3)
# 白心验证: 中心区域白色像素占比阈值
MAX_CENTER_JUMP = r.get("max_center_jump", 220)
# 帧间中心跳跃上限(像素) — 超过此值的候选框被忽略
MAX_AREA_RATIO  = r.get("max_area_ratio", 3)
# 帧间面积变化比例上限
MIN_RECT_FILL   = r.get("min_rect_fill", 45)
# 轮廓面积/外接矩形面积 最小百分比
MAX_SIDE_RATIO  = r.get("max_side_ratio", 3)
# 四边形对边长度比上限
APPROX_EPSILON  = r.get("approx_epsilon", 0.04)
# approxPolyDP精度: 0.04=轮廓周长的4%作为逼近误差

# ---- 跟踪 ----
t = _get(CFG, "tracking", {})
SMOOTH_NUM = t.get("smooth_num", 0)
SMOOTH_DEN = t.get("smooth_den", 1)
# 指数平滑: new = (old*SMOOTH_NUM + new)/SMOOTH_DEN
# 0/1=不平滑, 1/2=一半旧一半新, 2/3=偏旧
LOST_KEEP_FRAMES = t.get("lost_keep_frames", 2)
# 目标丢失后保留上一帧坐标的帧数 — 遮挡短暂恢复

# ---- 中心/激光偏移 ----
CENTER_CFG = _get(CFG, "center", {})
CENTER_X = CENTER_CFG.get("x", SENSOR_W // 2)
CENTER_Y = CENTER_CFG.get("y", SENSOR_H // 2)
# 画面中心像素坐标 — 分辨率改变时需同步更新
LASER_OFFSET_X = CENTER_CFG.get("laser_offset_x", 0)
LASER_OFFSET_Y = CENTER_CFG.get("laser_offset_y", 0)
# 激光光斑相对摄像头光轴的像素偏移(标定值)

# ---- 降采样检测 ----
DRES = _get(CFG, "detect_resolution", {})
USE_DOWNSCALE = DRES.get("enabled", False)
# 开启后检测在低分辨率上运行: 320x240仅1/4像素量
# 注意: cv2.resize有开销，实际帧率提升需实测。默认关闭
DETECT_W = DRES.get("width",  320)
DETECT_H = DRES.get("height", 240)
SCALE_X = SENSOR_W / DETECT_W
SCALE_Y = SENSOR_H / DETECT_H

# ---- YOLO KPU ----
YOLO_CFG = _get(CFG, "yolo", {})

# ---- 卡尔曼(EMA)开关 ----
k = _get(CFG, "kalman", {})
KALMAN_ENABLED = k.get("enabled", False)
KALMAN_ALPHA   = k.get("smooth_factor", 0.3)
# alpha越大→越跟手但越抖; alpha越小→越平滑但越延迟
# 0.2=很平滑(适合静态对准), 0.6=跟手(适合快速追踪), 0.3=平衡

# ---- 串口开关 ----
u = _get(CFG, "uart", {})
UART_ENABLED = u.get("enabled", False)
# 开启后向云台发送 dx,dy,dist,status

# ============================================================================
# 三、检测算法 — 纯函数，直接读全局参数（热路径优化）
#    不改函数签名，所有阈值通过模块全局变量传递
# ============================================================================

def point_xy(point):
    """统一解析OpenCV point类型 → (int(x), int(y))"""
    try:                     return int(point[0][0]), int(point[0][1])
    except Exception:        return int(point[0]), int(point[1])

def order_points(points):
    """四点排序: 左上→右上→右下→左下"""
    pts = [point_xy(p) for p in points]
    cx = sum([p[0] for p in pts]) // 4
    cy = sum([p[1] for p in pts]) // 4
    top, bottom = [], []
    for p in pts:
        if p[1] < cy: top.append(p)
        else:         bottom.append(p)
    if len(top) != 2 or len(bottom) != 2:
        pts.sort(key=lambda p: p[1])
        top, bottom = pts[:2], pts[2:]
    top.sort(key=lambda p: p[0])
    bottom.sort(key=lambda p: p[0])
    return [top[0], top[1], bottom[1], bottom[0]]

def center_of(points):
    return (sum(point_xy(p)[0] for p in points) // 4,
            sum(point_xy(p)[1] for p in points) // 4)

def get_contours(result):
    """兼容不同OpenCV版本的findContours返回值"""
    return result[0] if len(result) == 2 else result[1]

def has_black_border(frame_np, x, y, w, h):
    """验证矩形四边是否有足够黑色像素（黑边框检测）"""
    if w < 20 or h < 20: return False
    black = cv2.inRange(frame_np, BLACK_LOW, BLACK_HIGH)
    edge = max(2, min(w, h) // 18)
    top    = black[y:y+edge,    x:x+w]
    bottom = black[y+h-edge:y+h, x:x+w]
    left   = black[y:y+h,       x:x+edge]
    right  = black[y:y+h,       x+w-edge:x+w]
    hits = 0
    if cv2.countNonZero(top)    > w // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(bottom) > w // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(left)   > h // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(right)  > h // BLACK_CHECK_STEP: hits += 1
    return hits >= MIN_BLACK_HITS

def clamp_box(x, y, w, h):
    """裁切检测框到画面范围内"""
    if x < 0: x = 0
    if y < 0: y = 0
    if x + w > SENSOR_W: w = SENSOR_W - x
    if y + h > SENSOR_H: h = SENSOR_H - y
    return x, y, w, h

def has_white_center(white_mask, x, y, w, h):
    """验证矩形中心区域是否有足够白色像素（白心检测）"""
    ix, iy = x+w//4, y+h//4
    iw, ih = w//2, h//2
    if iw <= 0 or ih <= 0: return False
    roi = white_mask[iy:iy+ih, ix:ix+iw]
    return cv2.countNonZero(roi) > (iw * ih) // CENTER_WHITE_STEP

def quad_bounds(box):
    """四边形外接矩形"""
    xs = [p[0] for p in box]; ys = [p[1] for p in box]
    return min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys)

def quad_average_center(box):
    """四边形顶点均值中心"""
    return (sum(p[0] for p in box)//4, sum(p[1] for p in box)//4)

def quad_center(box):
    """四边形对角线交点（精确中心）"""
    x1,y1 = box[0]; x2,y2 = box[2]; x3,y3 = box[1]; x4,y4 = box[3]
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if den == 0: return quad_average_center(box)
    pre  = x1*y2 - y1*x2
    post = x3*y4 - y3*x4
    cx = (pre*(x3-x4) - (x1-x2)*post) // den
    cy = (pre*(y3-y4) - (y1-y2)*post) // den
    return int(cx), int(cy)

def score_box(x, y, w, h, box_area, aspect, last_box):
    """候选框评分: 面积大 + 接近目标宽高比 + 靠近上一帧位置 = 高分"""
    cx, cy = x + w//2, y + h//2
    aspect_bias = abs(aspect - TARGET_ASPECT) * 1000
    if last_box:
        lx, ly = quad_center(last_box)
        return box_area - (abs(cx-lx)+abs(cy-ly))*8 - aspect_bias
    return box_area - (abs(cx-SENSOR_W//2)+abs(cy-SENSOR_H//2))*2 - aspect_bias

def side_len2(a, b):
    dx, dy = a[0]-b[0], a[1]-b[1]
    return dx*dx + dy*dy

def is_good_quad(quad, contour_area, box_area):
    """多维度验证四边形质量: 填充率/边长比/对边比"""
    if contour_area * 100 < box_area * MIN_RECT_FILL: return False
    top, right = side_len2(quad[0], quad[1]), side_len2(quad[1], quad[2])
    bottom, left = side_len2(quad[2], quad[3]), side_len2(quad[3], quad[0])
    sides = [top, right, bottom, left]
    if min(sides) <= 0: return False
    if max(sides) > min(sides) * MAX_SIDE_RATIO * MAX_SIDE_RATIO: return False
    long_side  = max((top+bottom)//2, (left+right)//2)
    short_side = min((top+bottom)//2, (left+right)//2)
    if short_side <= 0: return False
    if long_side > short_side * MAX_ASPECT * MAX_ASPECT: return False
    if long_side * 100 < short_side * MIN_ASPECT * MIN_ASPECT * 100: return False
    return True

def contour_quad(cnt, box_area):
    """轮廓→四边形，含多边形逼近和质量验证"""
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, APPROX_EPSILON * peri, True)
    if len(approx) != 4: return None
    quad = order_points(approx)
    if not is_good_quad(quad, cv2.contourArea(cnt), box_area): return None
    return quad

def find_paper_box(frame_np, last_box):
    """核心检测: 白色掩膜→轮廓筛选→黑边验证→白心验证→评分→最优框"""
    mask = cv2.inRange(frame_np, WHITE_LOW, WHITE_HIGH)
    contours = get_contours(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    best, best_score = None, -1
    area_count = 0
    last_cx = last_cy = last_area = 0
    if last_box:
        last_cx, last_cy = quad_center(last_box)
        lx, ly, lw, lh = quad_bounds(last_box)
        last_area = lw * lh
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0: continue
        box_area = w * h
        if box_area < MIN_AREA or box_area > MAX_AREA: continue
        area_count += 1
        aspect = w / h
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT: continue
        x -= BORDER_EXPAND_X; y -= BORDER_EXPAND_Y
        w += BORDER_EXPAND_X*2; h += BORDER_EXPAND_Y*2
        x, y, w, h = clamp_box(x, y, w, h)
        cx, cy = x + w//2, y + h//2
        if last_box and abs(cx-last_cx)+abs(cy-last_cy) > MAX_CENTER_JUMP: continue
        if last_area and (box_area > last_area*MAX_AREA_RATIO or last_area > box_area*MAX_AREA_RATIO): continue
        if not has_white_center(mask, x, y, w, h): continue
        if not has_black_border(frame_np, x, y, w, h): continue
        quad = contour_quad(cnt, box_area)
        if not quad: continue
        score = score_box(x, y, w, h, box_area, aspect, last_box)
        if score > best_score: best_score = score; best = quad
    return best, len(contours), area_count

def smooth_box(last_box, box):
    """四边形顶点指数平滑: 减少帧间抖动"""
    if not last_box: return box
    return [((last_box[i][0]*SMOOTH_NUM + box[i][0]) // SMOOTH_DEN,
             (last_box[i][1]*SMOOTH_NUM + box[i][1]) // SMOOTH_DEN)
            for i in range(4)]

def draw_box(frame_np, box, source):
    """绘制检测框+十字线+中心点, 返回 (cx, cy, dx, dy)"""
    for i in range(4):
        cv2.line(frame_np, box[i], box[(i+1)%4], (0, 255, 0), 2)
    cx, cy = quad_center(box)
    dx, dy = cx - CENTER_X, cy - CENTER_Y
    cv2.line(frame_np, (cx-14, cy), (cx+14, cy), (255, 255, 0), 2)
    cv2.line(frame_np, (cx, cy-14), (cx, cy+14), (255, 255, 0), 2)
    cv2.circle(frame_np, (cx, cy), 4, (0, 0, 255), 1)
    return cx, cy, dx, dy

# ============================================================================
# 四、可选功能注入 — lambda模式, 热路径零分支
#    开启: 创建真实函数对象并绑定
#    关闭: 绑定为lambda identity / no-op
#    主循环中无感调用, 无 if 判断开销
# ============================================================================

# ---- 卡尔曼滤波器(EMA) ----
# 功能: 对dx,dy做一阶指数平滑, 减少检测抖动
# 配置: kalman.enabled / kalman.smooth_factor
# 关闭时: kf_update = 透传(float转换), kf_reset = no-op
class _Kalman:
    def __init__(self, a): self.a, self.x, self.y = a, None, None
    def update(self, mx, my):
        if self.x is None: self.x, self.y = float(mx), float(my)
        else:
            a = self.a
            self.x = a*float(mx) + (1-a)*self.x
            self.y = a*float(my) + (1-a)*self.y
        return self.x, self.y
    def reset(self): self.x = self.y = None

if KALMAN_ENABLED:
    _kf = _Kalman(KALMAN_ALPHA)
    kf_update = _kf.update      # 真实EMA
    kf_reset  = _kf.reset
else:
    kf_update = lambda mx, my: (float(mx), float(my))  # 透传
    kf_reset  = lambda: None                            # 空操作

# ---- 串口输出 ----
# 功能: 通过UART向云台STM32发送 dx,dy,dist,status
# 协议: "dx,dy,dist,status\n"  status: 0=追踪 1=对准 404=丢失
# 配置: uart.enabled / uart.port / uart.baud
# 关闭时: uart_send = no-op
if UART_ENABLED:
    try:
        from machine import UART
        _uart_port = u.get("port", 2)
        _uart_baud = u.get("baud", 115200)
        _uart = UART(_uart_port, baudrate=_uart_baud)
        print("uart: port=%d baud=%d" % (_uart_port, _uart_baud))
        def uart_send(dx, dy, dist, status):
            """发送偏差到云台 — 协议: dx,dy,dist,status\\n"""
            try:
                if status == "lost":
                    _uart.write("404,404,0,0\n")
                elif status == "aligned":
                    _uart.write("%d,%d,%.0f,1\n" % (dx, dy, dist))
                else:
                    _uart.write("%d,%d,%.0f,0\n" % (dx, dy, dist))
            except Exception:
                pass
    except Exception as e:
        print("uart init failed:", e)
        uart_send = lambda dx, dy, d, s: None
else:
    uart_send = lambda dx, dy, d, s: None

# ============================================================================
# 五、CV模式主循环
#    热路径结构与初版main.py一致 → 保证帧率
#    新增: kf_update/uart_send已在上方注入, 此处直接调用
# ============================================================================

def run_cv():
    global SENSOR_W, SENSOR_H  # 降采样时会临时改写

    sensor = None
    frame_id = lost_count = 0
    last_box = None
    fps = fps_count = 0
    fps_tick = time.ticks_ms()
    sw_orig, sh_orig = SENSOR_W, SENSOR_H

    try:
        # ---- 初始化 ----
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

            # ================================================================
            # 检测阶段
            #   detect_every: 跳帧检测 — 非检测帧直接复用last_box
            #   downscale:    降采样 — 在320x240上检测, 坐标映射回640x480
            # ================================================================
            if frame_id % DETECT_EVERY == 0 or not last_box:
                if USE_DOWNSCALE:
                    # --- 降采样路径 ---
                    detect_frame = cv2.resize(frame_np, (DETECT_W, DETECT_H))
                    SENSOR_W, SENSOR_H = DETECT_W, DETECT_H          # 临时切换尺寸
                    box, contour_count, area_count = find_paper_box(detect_frame, last_box)
                    SENSOR_W, SENSOR_H = sw_orig, sh_orig             # 恢复
                    if box:
                        box = [[int(p[0]*SCALE_X), int(p[1]*SCALE_Y)] for p in box]
                else:
                    # --- 原版路径(默认) ---
                    box, contour_count, area_count = find_paper_box(frame_np, last_box)
            else:
                box, contour_count, area_count = last_box, 0, 0

            # ================================================================
            # 结果处理阶段
            #   OK:    检测到 → 平滑 → 绘图 → 卡尔曼 → 距离 → 串口
            #   HOLD:  短暂丢失 → 复用上一帧坐标
            #   LOST:  完全丢失 → 重置卡尔曼 → 通知云台
            # ================================================================
            if box:
                # ---- 检测成功 ----
                box = smooth_box(last_box, box)
                last_box = box
                lost_count = 0
                cx, cy, dx, dy = draw_box(frame_np, box, "OK")

                # 卡尔曼滤波(或透传)
                fdx, fdy = kf_update(dx, dy)

                # 欧氏距离: sqrt(dx² + dy²) — 目标到画面中心的像素距离
                dist = math.sqrt(fdx*fdx + fdy*fdy)

                # 串口输出: dx,dy,dist,status
                uart_send(int(fdx), int(fdy), dist, "track")

                if frame_id % PRINT_EVERY == 0:
                    print("OK  xy=(%d,%d)  d=(%d,%d)  dist=%.0f  c=%d a=%d" %
                          (cx, cy, int(fdx), int(fdy), dist, contour_count, area_count))

            else:
                # ---- 检测失败 ----
                lost_count += 1
                if last_box and lost_count <= LOST_KEEP_FRAMES:
                    # 短暂丢失 → 保持上一帧位置
                    cx, cy, dx, dy = draw_box(frame_np, last_box, "HOLD")
                    fdx, fdy = kf_update(dx, dy)
                    dist = math.sqrt(fdx*fdx + fdy*fdy)
                    uart_send(int(fdx), int(fdy), dist, "track")
                    if frame_id % PRINT_EVERY == 0:
                        print("HOLD xy=(%d,%d)  d=(%d,%d)  dist=%.0f  c=%d a=%d" %
                              (cx, cy, int(fdx), int(fdy), dist, contour_count, area_count))
                else:
                    # 完全丢失 → 重置状态
                    last_box = None
                    kf_reset()
                    uart_send(0, 0, 0, "lost")
                    if frame_id % PRINT_EVERY == 0:
                        print("LOST c=%d a=%d" % (contour_count, area_count))

            # ================================================================
            # OSD叠加 — 左上角显示实时偏差和距离
            # ================================================================
            if SHOW_OSD and box:
                cv2.putText(frame_np, "dx:%+d dy:%+d dist:%.0f" % (int(fdx), int(fdy), dist),
                            (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # ================================================================
            # FPS统计
            # ================================================================
            fps_count += 1
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, fps_tick)
            if elapsed >= 1000:
                fps = fps_count * 1000 // elapsed
                fps_count = 0; fps_tick = now
            cv2.putText(frame_np, "FPS:%d" % fps, (4, SENSOR_H - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            Display.show_image(frame, x=DISPLAY_X, y=DISPLAY_Y)

            # ================================================================
            # 垃圾回收 — 定时清理, 防止内存堆积导致周期性卡顿
            # ================================================================
            if frame_id % GC_EVERY == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        print("error:", e)
    finally:
        if sensor:
            try: sensor.stop()
            except Exception: pass
        try: Display.deinit()
        except Exception: pass
        try: os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception: pass
        time.sleep_ms(100)
        try: MediaManager.deinit()
        except Exception: pass
        print("exit")

# ============================================================================
# 六、YOLO KPU模式
#    使用K230 PipeLine框架进行KPU推理
#    功能: 目标检测 → 中心偏差 → 距离 → 串口
#    前提: SD卡上有model.kmodel, KPU库可用
# ============================================================================

def _yolo_ok():
    """检测KPU库是否可用 — 不可用时自动回退CV模式"""
    try:
        import nncase_runtime, aidemo
        from libs.PipeLine import PipeLine
        from libs.AIBase import AIBase
        from libs.Ai2d import Ai2d
        return True
    except ImportError:
        return False

def run_yolo():
    from libs.PipeLine import PipeLine, ScopedTiming
    from libs.AIBase import AIBase
    from libs.Ai2d import Ai2d
    from libs.Utils import ALIGN_UP, letterbox_pad_param, get_colors
    import nncase_runtime as nn
    import ulab.numpy as np
    import aidemo

    MP = YOLO_CFG.get("model_path", "/sdcard/model/model.kmodel")
    LABELS = YOLO_CFG.get("labels", ["target"])
    ISIZE  = YOLO_CFG.get("input_size", [320, 320])
    CONF   = YOLO_CFG.get("confidence", 0.3)
    NMS    = YOLO_CFG.get("nms_threshold", 0.4)
    MB     = YOLO_CFG.get("max_boxes", 30)

    class YOLOv8App(AIBase):
        def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num,
                     confidence_threshold, nms_threshold, rgb888p_size, display_size, debug_mode=0):
            super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
            self.labels = labels
            self.model_input_size = model_input_size
            self.confidence_threshold = confidence_threshold
            self.nms_threshold = nms_threshold
            self.max_boxes_num = max_boxes_num
            self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
            self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
            self.debug_mode = debug_mode
            self.color_four = get_colors(len(labels))
            self.ai2d = Ai2d(debug_mode)
            self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

        def config_preprocess(self, input_image_size=None):
            with ScopedTiming("set preprocess config", self.debug_mode > 0):
                ai2d_input_size = input_image_size or self.rgb888p_size
                top, bottom, left, right, self.scale = letterbox_pad_param(self.rgb888p_size, self.model_input_size)
                self.ai2d.pad([0,0,0,0, top,bottom,left,right], 0, [128,128,128])
                self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                                [1,3,self.model_input_size[1],self.model_input_size[0]])

        def preprocess(self, input_np):
            with ScopedTiming("preprocess", self.debug_mode > 0):
                return [nn.from_numpy(input_np)]

        def postprocess(self, results):
            with ScopedTiming("postprocess", self.debug_mode > 0):
                nr = results[0][0].transpose()
                return aidemo.yolov8_det_postprocess(nr.copy(),
                    [self.rgb888p_size[1],self.rgb888p_size[0]],
                    [self.model_input_size[1],self.model_input_size[0]],
                    [self.display_size[1],self.display_size[0]],
                    len(self.labels), self.confidence_threshold,
                    self.nms_threshold, self.max_boxes_num)

        def draw_result(self, pl, dets):
            with ScopedTiming("display_draw", self.debug_mode > 0):
                if dets:
                    pl.osd_img.clear()
                    for i in range(len(dets[0])):
                        x,y,w,h = map(lambda v: int(round(v,0)), dets[0][i])
                        pl.osd_img.draw_rectangle(x,y,w,h, color=self.color_four[dets[1][i]], thickness=4)
                        pl.osd_img.draw_string_advanced(x,y-50,32,
                            " %s %.2f " % (self.labels[dets[1][i]], dets[2][i]),
                            color=self.color_four[dets[1][i]])
                else:
                    pl.osd_img.clear()

    print("=" * 60)
    print("YOLOv8 KPU mode")
    print("model: %s  labels: %s" % (MP, LABELS))
    print("=" * 60)

    pl = PipeLine(rgb888p_size=ISIZE, display_mode="lcd", display_size=None)
    pl.create()
    ds = pl.get_display_size()
    det = YOLOv8App(MP, labels=LABELS, model_input_size=ISIZE,
                     max_boxes_num=MB, confidence_threshold=CONF,
                     nms_threshold=NMS, rgb888p_size=ISIZE,
                     display_size=ds, debug_mode=0)
    det.config_preprocess()
    print("running...")
    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = det.run(img)
                det.draw_result(pl, res)
                pl.show_image()
                if res and len(res[0]) > 0:
                    x,y,w,h = [int(round(v,0)) for v in res[0][0]]
                    cx, cy = x+w//2, y+h//2
                    fdx, fdy = kf_update(cx-CENTER_X, cy-CENTER_Y)
                    dist = math.sqrt(fdx*fdx + fdy*fdy)
                    uart_send(int(fdx), int(fdy), dist, "track")
                else:
                    uart_send(0, 0, 0, "lost")
                gc.collect()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        det.deinit(); pl.destroy()

# ============================================================================
# 七、入口 — 根据 config.json 中 detection.mode 选择运行模式
# ============================================================================

if MODE == "yolo":
    if _yolo_ok():
        run_yolo()
    else:
        print("YOLO libs missing, fallback to CV")
        run_cv()
else:
    # MODE == "cv" 或 "hybrid" → 都走CV路径
    run_cv()
