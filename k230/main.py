"""
    K230 CanMV 靶标追踪 — 电赛E题复刻版
    =====================================
    基于原 MaixCam 项目移植，支持三种检测模式:
      - cv:   find_rects  硬件矩形检测（精确，有角点）
      - blob: find_blobs  硬件色块检测（更快，无需矩形特征）
      - yolo: KPU YOLO推理（需 .kmodel 模型）

    核心优化:
      - ROI 追踪：首次全图搜索，追踪期仅在目标周围搜索（面积缩减 75%）
      - 三通道传感器：chn0(DMA显示) + chn1(检测) + chn2(OSD叠加)
      - 检测分辨率 320×192，缩放比 2.5× → 全分辨率 800×480

    通信协议:
      检测到目标: "dx,dy,0,0\n"    (像素偏差)
      目标丢失:   "404,404,0,0\n"

    @version 3.0 — 多模式 + ROI追踪
    @date    2025.8
"""

import time, gc, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import Pin

# =========================== YOLO 模块导入 ============================
try:
    from libs.PipeLine import PipeLine, ScopedTiming
    from libs.AIBase import AIBase
    from libs.AI2D import Ai2d
    from libs.Utils import *
    import nncase_runtime as nn
    import ulab.numpy as np
    import aidemo
    YOLO_LIBS_AVAILABLE = True
except Exception:
    YOLO_LIBS_AVAILABLE = False

# =========================== 配置区 ===================================

# ★ 检测模式: "cv" / "blob" / "cascade" / "yolo"
#   cv:      find_rects 直接矩形检测
#   blob:    find_blobs 色块检测 + 形状约束
#   cascade: find_blobs 粗筛 → find_rects 精验（两级硬件级联）
#   yolo:    KPU 神经网络推理
DETECTION_MODE = "blob"

# 摄像头
CAM_WIDTH  = 800
CAM_HEIGHT = 480
SENSOR_ID  = 2

# 检测分辨率
DET_WIDTH  = 320
DET_HEIGHT = 192

# ---- find_rects 参数（cv 模式）----
FIND_RECTS_THRESHOLD = 15000
RECT_AREA_MIN = 200
ASPECT_MIN = 1.0
ASPECT_MAX = 3.0
ANGLE_TOL  = 25

# ---- find_blobs 参数（blob 模式）----
# LAB 色彩空间阈值 — 找暗色/黑色区域
#   L:   0 ~ 50   (亮度低 → 暗色)
#   A: -30 ~ 30   (任意色相)
#   B: -30 ~ 30   (任意色相)
BLOB_L_THRESHOLD  = (0, 50)       # L 亮度阈值
BLOB_A_THRESHOLD  = (-30, 30)     # A 色相
BLOB_B_THRESHOLD  = (-30, 30)     # B 色相
BLOB_PIXELS_MIN   = 100           # 最小像素数
BLOB_AREA_MIN     = 200           # 最小面积
BLOB_MERGE        = True          # 合并相邻色块

# ---- 背景白底约束（blob 模式防误判）----
# 检测到的暗色块周边必须是亮色（白色背景），否则视为误判丢弃
BG_WHITE_MIN   = 130             # 背景采样点 RGB 各通道最低值 (0~255)
BG_SAMPLE_GAP  = 6               # 采样点距边界距离 (像素)
BG_SAMPLE_OK   = 5               # 最少几个采样点通过才算有效（共8点）

# ---- 靶标形状约束（blob 模式）----
# A4 黑框: 外框 297×210，长宽比 ≈ 1.4；空心框密度低（边框/外接矩形 ≈ 0.1~0.4）
BLOB_ASPECT_MIN = 1.1            # 长宽比下限
BLOB_ASPECT_MAX = 2.0            # 长宽比上限
BLOB_DENSITY_MAX = 0.55          # 密度上限：低于此值才是空心框（实心块→1.0）

# ---- YOLO 参数（yolo 模式）----
KMODEL_PATH = "/sdcard/model/best001.kmodel"
YOLO_LABELS = ["black_frame"]
YOLO_CONF   = 0.5
YOLO_NMS    = 0.45
YOLO_INPUT  = [320, 320]

# ---- ROI 追踪 ----
ROI_MARGIN   = 80                # 窗口半边长（检测分辨率）
ROI_LOST_MAX = 30                # 连续丢失多少帧恢复全图

# ---- 串口 ----
UART_PORT = 2
UART_BAUD = 115200

# ---- 显示 ----
debug_draw_rect     = True
debug_draw_crosshair = True
print_fps_terminal  = True
OSD_EVERY_N = 2                  # OSD 每 N 帧刷新一次（1=每帧, 2=隔帧）
DET_SKIP_N  = 2                  # 检测每 N 帧跑一次（1=每帧, 2=隔帧，用上次结果填充）

# =====================================================================

SCALE_X = float(CAM_WIDTH)  / float(DET_WIDTH)   # 2.5
SCALE_Y = float(CAM_HEIGHT) / float(DET_HEIGHT)  # 2.5

_angle_limit_sq = math.cos(math.radians(90 - ANGLE_TOL)) ** 2

# 全局
yolo_app = None
uart = None

# ROI 追踪状态（检测分辨率坐标）
_roi_last_x = None
_roi_last_y = None
_roi_lost_count = 0


# =========================== CV: find_rects ===========================

def poly_area(p0, p1, p2, p3):
    a  = p0[0]*p1[1] - p1[0]*p0[1]
    a += p1[0]*p2[1] - p2[0]*p1[1]
    a += p2[0]*p3[1] - p3[0]*p2[1]
    a += p3[0]*p0[1] - p0[0]*p3[1]
    if a < 0: a = -a
    return a * 0.5


def aspect_ok(p0, p1, p2, p3):
    xs = [p0[0], p1[0], p2[0], p3[0]]
    ys = [p0[1], p1[1], p2[1], p3[1]]
    w = max(xs) - min(xs)
    h = max(ys) - min(ys)
    if w < 1 or h < 1:
        return False
    r = float(w)/float(h) if w > h else float(h)/float(w)
    return ASPECT_MIN <= r <= ASPECT_MAX


def angle_ok(p0, p1, p2, p3):
    pts = (p0, p1, p2, p3)
    for i in range(4):
        prev = pts[(i-1)%4]; curr = pts[i]; nxt = pts[(i+1)%4]
        v1x = float(prev[0]) - float(curr[0])
        v1y = float(prev[1]) - float(curr[1])
        v2x = float(nxt[0])  - float(curr[0])
        v2y = float(nxt[1])  - float(curr[1])
        m1 = v1x*v1x + v1y*v1y
        m2 = v2x*v2x + v2y*v2y
        if m1 < 0.001 or m2 < 0.001:
            return False
        dot = v1x*v2x + v1y*v2y
        if dot * dot > _angle_limit_sq * m1 * m2:
            return False
    return True


def centroid(corners):
    sx = sum(c[0] for c in corners)
    sy = sum(c[1] for c in corners)
    return (int(sx / 4), int(sy / 4))


def detect_cv(img):
    """find_rects 矩形检测 + ROI追踪"""
    global _roi_last_x, _roi_last_y, _roi_lost_count

    # ROI 搜索
    if _roi_last_x is not None and _roi_lost_count < ROI_LOST_MAX:
        rx = max(0, _roi_last_x - ROI_MARGIN)
        ry = max(0, _roi_last_y - ROI_MARGIN)
        rw = min(DET_WIDTH  - rx, ROI_MARGIN * 2)
        rh = min(DET_HEIGHT - ry, ROI_MARGIN * 2)
        try:
            rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD,
                                   roi=(rx, ry, rw, rh))
            found_in_roi = True
        except Exception:
            rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)
            found_in_roi = False
    else:
        rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)
        found_in_roi = False

    if not rects:
        if found_in_roi:
            rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)
        if not rects:
            _roi_lost_count += 1
            return None

    best = None
    best_score = 0
    for r in rects:
        c = r.corners()
        if len(c) != 4:
            continue
        p0, p1, p2, p3 = c[0], c[1], c[2], c[3]
        area = poly_area(p0, p1, p2, p3)
        if area < RECT_AREA_MIN:
            continue
        if not aspect_ok(p0, p1, p2, p3):
            continue
        if not angle_ok(p0, p1, p2, p3):
            continue
        cx, cy = centroid(c)
        if cx < 0 or cx >= DET_WIDTH or cy < 0 or cy >= DET_HEIGHT:
            continue
        if area > best_score:
            best_score = area
            best = (cx, cy, c)

    if best is None:
        _roi_lost_count += 1
        return None

    cx_det, cy_det, corners = best
    _roi_last_x = cx_det
    _roi_last_y = cy_det
    _roi_lost_count = 0
    return (int(cx_det * SCALE_X), int(cy_det * SCALE_Y),
            corners, (cx_det, cy_det))


# =========================== Blob: find_blobs ===========================

def bg_white_check(img, blob):
    """
    检查色块周边是否为白色背景。
    在色块边界框外围采样 8 个点，如果大多数是亮色则通过。
    """
    x = blob.x()
    y = blob.y()
    w = blob.w()
    h = blob.h()
    gap = BG_SAMPLE_GAP

    # 8 个采样点：四边中点 + 四角
    samples = [
        (x + w // 2, y - gap),           # 上边中点
        (x + w // 2, y + h + gap),       # 下边中点
        (x - gap,      y + h // 2),       # 左边中点
        (x + w + gap,  y + h // 2),       # 右边中点
        (x - gap,      y - gap),          # 左上角
        (x + w + gap,  y - gap),          # 右上角
        (x - gap,      y + h + gap),      # 左下角
        (x + w + gap,  y + h + gap),      # 右下角
    ]

    ok = 0
    for sx, sy in samples:
        if sx < 2 or sy < 2 or sx >= DET_WIDTH - 2 or sy >= DET_HEIGHT - 2:
            ok += 1  # 出界也算通过（可能是画面边缘的靶标）
            continue
        try:
            r, g, b = img.get_pixel(sx, sy)
            if r >= BG_WHITE_MIN and g >= BG_WHITE_MIN and b >= BG_WHITE_MIN:
                ok += 1
        except Exception:
            ok += 1  # 读不到也算通过，不断在边界情况

    return ok >= BG_SAMPLE_OK



def detect_blob(img):
    """
    硬件色块检测 + ROI追踪 + 几何约束。
    收紧 LAB 阈值 + 长宽比 + 密度 + 白底约束，过滤误判。
    """
    global _roi_last_x, _roi_last_y, _roi_lost_count

    thresholds = [BLOB_L_THRESHOLD, BLOB_A_THRESHOLD, BLOB_B_THRESHOLD]

    # ROI 搜索
    if _roi_last_x is not None and _roi_lost_count < ROI_LOST_MAX:
        rx = max(0, _roi_last_x - ROI_MARGIN)
        ry = max(0, _roi_last_y - ROI_MARGIN)
        rw = min(DET_WIDTH  - rx, ROI_MARGIN * 2)
        rh = min(DET_HEIGHT - ry, ROI_MARGIN * 2)
        try:
            blobs = img.find_blobs(thresholds, roi=(rx, ry, rw, rh),
                                   pixels_threshold=BLOB_PIXELS_MIN,
                                   area_threshold=BLOB_AREA_MIN,
                                   merge=BLOB_MERGE)
            found_in_roi = True
        except Exception:
            blobs = img.find_blobs(thresholds,
                                   pixels_threshold=BLOB_PIXELS_MIN,
                                   area_threshold=BLOB_AREA_MIN,
                                   merge=BLOB_MERGE)
            found_in_roi = False
    else:
        blobs = img.find_blobs(thresholds,
                               pixels_threshold=BLOB_PIXELS_MIN,
                               area_threshold=BLOB_AREA_MIN,
                               merge=BLOB_MERGE)
        found_in_roi = False

    if not blobs:
        if found_in_roi:
            blobs = img.find_blobs(thresholds,
                                   pixels_threshold=BLOB_PIXELS_MIN,
                                   area_threshold=BLOB_AREA_MIN,
                                   merge=BLOB_MERGE)
        if not blobs:
            _roi_lost_count += 1
            return None

    # 选面积最大且通过全部约束的色块
    best = None
    best_area = 0
    for b in blobs:
        if b.area() <= best_area:
            continue
        # ① 长宽比：必须接近矩形靶标
        bw, bh = b.w(), b.h()
        if bw < 8 or bh < 8:
            continue
        aspect = float(bw) / float(bh) if bw > bh else float(bh) / float(bw)
        if aspect < BLOB_ASPECT_MIN or aspect > BLOB_ASPECT_MAX:
            continue
        # ② 密度：空心框密度低，实心噪点→1.0
        try:
            density = b.density()
        except Exception:
            density = 1.0
        if density > BLOB_DENSITY_MAX:
            continue

        # ③ 背景白底检查
        if not bg_white_check(img, b):
            continue

        best_area = b.area()
        best = b

    if best is None:
        _roi_lost_count += 1
        return None

    cx_det = best.cx()
    cy_det = best.cy()
    _roi_last_x = cx_det
    _roi_last_y = cy_det
    _roi_lost_count = 0

    # blob 的角点（最小外接矩形）
    corners = None
    try:
        corners = best.corners()
        if len(corners) != 4:
            corners = None
    except Exception:
        pass

    return (int(cx_det * SCALE_X), int(cy_det * SCALE_Y),
            corners, (cx_det, cy_det))


# =========================== YOLO: KPU ================================

def detect_yolo(img):
    global yolo_app
    try:
        res = yolo_app.run(img)
        if res and len(res) > 0:
            x, y, w, h = map(lambda v: int(round(v, 0)), res[0][0])
            cx = x + w // 2
            cy = y + h // 2
            return (int(cx * SCALE_X), int(cy * SCALE_Y), None, (cx, cy))
    except Exception:
        pass
    return None


# ===================== Cascade: blob粗筛 → rect精验 ======================

CASCADE_BLOB_PIXELS = 60         # blob 粗筛：宽松，多收候选
CASCADE_BLOB_AREA   = 60
CASCADE_RECT_THRESH = 10000      # rect 精验阈值
CASCADE_ROI_PAD     = 20         # blob 边界框外扩
CASCADE_RECT_AREA_MIN = 150

def detect_cascade(img):
    """
    两级硬件级联：S1: find_blobs 全图快扫 → S2: find_rects 候选区内精验。
    blob 负责速度（不漏），rect 负责精度（不误判）。
    """
    global _roi_last_x, _roi_last_y, _roi_lost_count

    thresholds = [BLOB_L_THRESHOLD, BLOB_A_THRESHOLD, BLOB_B_THRESHOLD]

    # S1: blob 粗筛
    if _roi_last_x is not None and _roi_lost_count < ROI_LOST_MAX:
        rx = max(0, _roi_last_x - ROI_MARGIN)
        ry = max(0, _roi_last_y - ROI_MARGIN)
        rw = min(DET_WIDTH  - rx, ROI_MARGIN * 2)
        rh = min(DET_HEIGHT - ry, ROI_MARGIN * 2)
        try:
            blobs = img.find_blobs(thresholds, roi=(rx, ry, rw, rh),
                                   pixels_threshold=CASCADE_BLOB_PIXELS,
                                   area_threshold=CASCADE_BLOB_AREA, merge=True)
        except Exception:
            blobs = img.find_blobs(thresholds,
                                   pixels_threshold=CASCADE_BLOB_PIXELS,
                                   area_threshold=CASCADE_BLOB_AREA, merge=True)
    else:
        blobs = img.find_blobs(thresholds,
                               pixels_threshold=CASCADE_BLOB_PIXELS,
                               area_threshold=CASCADE_BLOB_AREA, merge=True)

    if not blobs:
        _roi_lost_count += 1
        return None

    # 按面积降序 — 优先验证最大候选
    blobs.sort(key=lambda b: b.area(), reverse=True)

    # S2: rect 精验 — 在候选 ROI 内找矩形
    for b in blobs:
        bx, by, bw, bh = b.x(), b.y(), b.w(), b.h()
        pad = CASCADE_ROI_PAD
        rx = max(0, bx - pad)
        ry = max(0, by - pad)
        rw = min(DET_WIDTH  - rx, bw + pad * 2)
        rh = min(DET_HEIGHT - ry, bh + pad * 2)

        try:
            rects = img.find_rects(threshold=CASCADE_RECT_THRESH,
                                   roi=(rx, ry, rw, rh))
        except Exception:
            continue

        if not rects:
            continue

        best_r = None
        best_area = 0
        for r in rects:
            c = r.corners()
            if len(c) != 4:
                continue
            area = poly_area(c[0], c[1], c[2], c[3])
            if area < CASCADE_RECT_AREA_MIN:
                continue
            if area > best_area:
                best_area = area
                best_r = (c, area)

        if best_r is not None:
            corners, _ = best_r
            cx_det, cy_det = centroid(corners)
            _roi_last_x = cx_det
            _roi_last_y = cy_det
            _roi_lost_count = 0
            return (int(cx_det * SCALE_X), int(cy_det * SCALE_Y),
                    corners, (cx_det, cy_det))

    _roi_lost_count += 1
    return None


# =========================== 统一入口 =================================

def detect(img):
    if DETECTION_MODE == "cv":
        return detect_cv(img)
    elif DETECTION_MODE == "blob":
        return detect_blob(img)
    elif DETECTION_MODE == "cascade":
        return detect_cascade(img)
    else:
        return detect_yolo(img)


# =========================== 串口通信 =================================

def send_tracking_data(dx, dy, lost=False):
    if uart is None:
        return
    try:
        if lost:
            uart.write("404,404,0,0\n")
        else:
            uart.write("%d,%d,0,0\n" % (dx, dy))
    except Exception:
        pass


# =========================== 画图工具 =================================

def draw_crosshair(img, cx, cy, size=20, color=(255, 255, 0), thickness=2):
    img.draw_line(cx - size, cy, cx + size, cy, color, thickness)
    img.draw_line(cx, cy - size, cx, cy + size, color, thickness)


# =========================== 初始化 ===================================

def init_yolo():
    global yolo_app
    if not YOLO_LIBS_AVAILABLE:
        print("[YOLO] KPU库不可用")
        return False
    try:
        class YOLOv8App(AIBase):
            def __init__(self, kmodel_path, labels, model_input_size,
                         max_boxes_num=30, confidence_threshold=0.3,
                         nms_threshold=0.4, rgb888p_size=None,
                         display_size=None, debug_mode=0):
                if rgb888p_size is None:
                    rgb888p_size = model_input_size
                if display_size is None:
                    display_size = [1920, 1080]
                super().__init__(kmodel_path, model_input_size,
                                 rgb888p_size, debug_mode)
                self.labels = labels
                self.model_input_size = model_input_size
                self.confidence_threshold = confidence_threshold
                self.nms_threshold = nms_threshold
                self.max_boxes_num = max_boxes_num
                self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16),
                                     rgb888p_size[1]]
                self.display_size = [ALIGN_UP(display_size[0], 16),
                                     display_size[1]]
                self.debug_mode = debug_mode
                self.color_four = get_colors(len(self.labels))
                self.ai2d = Ai2d(debug_mode)
                self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                         nn.ai2d_format.NCHW_FMT,
                                         np.uint8, np.uint8)

            def config_preprocess(self, input_image_size=None):
                ai2d_input_size = (input_image_size
                                   if input_image_size else self.rgb888p_size)
                top, bottom, left, right, self.scale = letterbox_pad_param(
                    self.rgb888p_size, self.model_input_size)
                self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [128,128,128])
                self.ai2d.resize(nn.interp_method.tf_bilinear,
                                 nn.interp_mode.half_pixel)
                self.ai2d.build(
                    [1,3,ai2d_input_size[1],ai2d_input_size[0]],
                    [1,3,self.model_input_size[1],self.model_input_size[0]])

            def preprocess(self, input_np):
                return [nn.from_numpy(input_np)]

            def postprocess(self, results):
                new_result = results[0][0].transpose()
                det_res = aidemo.yolov8_det_postprocess(
                    new_result.copy(),
                    [self.rgb888p_size[1], self.rgb888p_size[0]],
                    [self.model_input_size[1], self.model_input_size[0]],
                    [self.display_size[1], self.display_size[0]],
                    len(self.labels), self.confidence_threshold,
                    self.nms_threshold, self.max_boxes_num)
                return det_res

        yolo_app = YOLOv8App(KMODEL_PATH, YOLO_LABELS, YOLO_INPUT,
                             confidence_threshold=YOLO_CONF,
                             nms_threshold=YOLO_NMS,
                             rgb888p_size=[DET_WIDTH, DET_HEIGHT])
        yolo_app.config_preprocess()
        print("[YOLO] 初始化成功: %s" % KMODEL_PATH)
        return True
    except Exception as e:
        print("[YOLO] 初始化失败: %s" % e)
        return False


# =========================== 主程序 ===================================

def main():
    global yolo_app, uart, DETECTION_MODE
    global _roi_last_x, _roi_last_y, _roi_lost_count

    print("=" * 60)
    print("K230 靶标追踪 v3.0 — %s 模式" % DETECTION_MODE.upper())
    print("=" * 60)
    print("分辨率:  %dx%d (检测 %dx%d)" %
          (CAM_WIDTH, CAM_HEIGHT, DET_WIDTH, DET_HEIGHT))
    print("ROI窗口: %dpx | 丢失阈值: %d帧" % (ROI_MARGIN, ROI_LOST_MAX))
    if DETECTION_MODE == "blob":
        print("色块阈值: L%s A%s B%s" %
              (BLOB_L_THRESHOLD, BLOB_A_THRESHOLD, BLOB_B_THRESHOLD))
    print("=" * 60)

    sensor_obj = None
    yolo_app = None
    uart = None

    try:
        # ---- 摄像头 ----
        sensor_obj = Sensor(id=SENSOR_ID)
        sensor_obj.reset()

        sensor_obj.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT,
                                 chn=CAM_CHN_ID_0)
        sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        bind_info = sensor_obj.bind_info(chn=CAM_CHN_ID_0)
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

        sensor_obj.set_framesize(width=DET_WIDTH, height=DET_HEIGHT,
                                 chn=CAM_CHN_ID_1)
        sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

        # chn2: 全分辨率 → OSD 叠加
        sensor_obj.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT,
                                 chn=CAM_CHN_ID_2)
        sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_2)

        Display.init(Display.ST7701, width=800, height=480,
                     to_ide=True, osd_num=1)
        sensor_obj.run()
        print("HW   OK | CAM+Display")

        # ---- YOLO ----
        if DETECTION_MODE == "yolo":
            if not init_yolo():
                print("[WARN] YOLO失败，回退blob")
                DETECTION_MODE = "blob"

        # ---- 串口 ----
        try:
            uart = UART(UART_PORT, UART_BAUD)
            print("UART OK | UART%d @ %d" % (UART_PORT, UART_BAUD))
        except Exception as e:
            print("UART -- | %s" % e)

        # ---- 状态 ----
        center_pos = (CAM_WIDTH // 2, CAM_HEIGHT // 2)
        last_det_info = None
        last_dx, last_dy = 0, 0

        clock = time.clock()
        frame = 0
        lost_ms = 0
        lost_on = False

        # 丢弃前 10 帧
        for _ in range(10):
            sensor_obj.snapshot(chn=CAM_CHN_ID_1)

        print("LOOP...")

        while True:
            os.exitpoint()
            clock.tick()
            frame += 1

            # ==== 检测 ====
            img_det = sensor_obj.snapshot(chn=CAM_CHN_ID_1)

            if frame % DET_SKIP_N == 0:
                det_info = detect(img_det)
                if det_info is not None:
                    cx_full, cy_full, corners, (cx_det, cy_det) = det_info
                    dx = center_pos[0] - cx_full
                    dy = center_pos[1] - cy_full
                    last_det_info = det_info
                    last_dx, last_dy = dx, dy
                    lost_on = False
                    lost_ms = 0
                else:
                    if not lost_on:
                        lost_ms = time.ticks_ms()
                        lost_on = True

            # UART 每帧都发（用最新偏差值）
            if last_det_info is not None and not lost_on:
                if uart is not None:
                    uart.write("%d,%d,0,0\n" % (last_dx, last_dy))
            elif lost_on and time.ticks_diff(time.ticks_ms(), lost_ms) > 2000:
                if uart is not None:
                    uart.write("404,404,0,0\n")

            # ==== OSD ====
            if OSD_EVERY_N > 0 and frame % OSD_EVERY_N == 0:
                osd_img = sensor_obj.snapshot(chn=CAM_CHN_ID_2)

                if debug_draw_crosshair:
                    draw_crosshair(osd_img, center_pos[0], center_pos[1])

                if last_det_info is not None:
                    cx_full, cy_full, corners, _ = last_det_info

                    # 矩形框
                    if debug_draw_rect and corners and len(corners) == 4:
                        scaled = [(int(p[0] * SCALE_X), int(p[1] * SCALE_Y))
                                  for p in corners]
                        for i in range(4):
                            osd_img.draw_line(scaled[i][0], scaled[i][1],
                                              scaled[(i+1)%4][0], scaled[(i+1)%4][1],
                                              color=(255, 0, 0), thickness=2)

                    # 中心点 + 连线 + 偏差值
                    osd_img.draw_circle(cx_full, cy_full, 5,
                                        color=(0, 255, 0), thickness=1, fill=True)
                    osd_img.draw_line(center_pos[0], center_pos[1],
                                      cx_full, cy_full,
                                      color=(0, 255, 255), thickness=2)
                    osd_img.draw_string_advanced(
                        cx_full + 8, cy_full - 8, 14,
                        "(%+d,%+d)" % (last_dx, last_dy),
                        color=(0, 255, 0))

                # 状态标识
                osd_img.draw_string_advanced(
                    2, 2, 14, "%s | %dFPS" % (DETECTION_MODE.upper(), int(clock.fps())),
                    color=(0, 255, 255))

                Display.show_image(osd_img, layer=Display.LAYER_OSD1)

            # FPS 终端
            if print_fps_terminal and frame % 30 == 0:
                print("[FPS] %d" % int(clock.fps()))
                gc.collect()

    except KeyboardInterrupt:
        print("STOP")
    except Exception as e:
        print("ERR: " + str(e))
        import sys
        sys.print_exception(e)
    finally:
        if sensor_obj is not None:
            sensor_obj.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        gc.collect()
        print("END")


if __name__ == "__main__":
    main()
