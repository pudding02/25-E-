"""
K230 CanMV 矩形靶标识别与追踪 — 电赛E题复刻版
==============================================
基于 Simple_Tracking_Device 原项目，升级为 K230 + STM32F407 PID 闭环追踪。
支持经典CV (find_rects) 和 YOLO KPU 双模式，现场可切换。

部署: 复制此文件到 SD 卡根目录重命名为 main.py
运行: CanMV IDE 中打开运行，或上电自动执行

通信协议 (K230 → STM32):
  正常: "dx,dy,0,0\n"     像素偏差
  丢失: "404,404,0,0\n"   目标丢失
  对准: "dx,dy,1,0\n"     flag1=1 表示已对准

@version 2.0 — K230复刻版
@date 2025.8
"""

import time, gc, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import Pin

# YOLO 相关模块 — 模块顶层导入（MicroPython 不允许函数内 import *）
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

# =========================== 配置区 ============================

# ★ 检测模式: "cv" / "yolo"
#   cv:   经典CV矩形检测（无需模型, ~45fps, 推荐）
#   yolo: KPU YOLO推理（需预训练kmodel）
DETECTION_MODE = "cv"

# 摄像头分辨率
CAM_WIDTH  = 800
CAM_HEIGHT = 480
SENSOR_ID  = 2           # K230 MIPI CSI 传感器ID

# 检测用低分辨率（加速用）
DET_WIDTH  = 400
DET_HEIGHT = 240

# YOLO模型路径（仅 yolo 模式使用）
KMODEL_PATH = "/sdcard/model/model.kmodel"
YOLO_LABELS = ["target"]          # 必须与 data.yaml 中 names 顺序一致
YOLO_CONF   = 0.3                 # 置信度阈值
YOLO_NMS    = 0.4                 # NMS阈值
YOLO_INPUT  = [320, 320]          # 模型输入尺寸

# 矩形检测参数（cv模式）
FIND_RECTS_THRESHOLD = 10000      # 矩形检测阈值 (5000-20000), 越低越敏感
AREA_MIN   = 100                  # 最小面积
ASPECT_MIN = 1.0                  # 长宽比下限
ASPECT_MAX = 3.0                  # 长宽比上限
ANGLE_TOL  = 25                   # 角度容差(度)

# 画面中心（自动计算 = 分辨率/2）
CENTER_X = CAM_WIDTH // 2         # 400
CENTER_Y = CAM_HEIGHT // 2        # 240

# ★ 激光偏移补偿（像素）— 激光安装点与摄像头光轴不重合时填入
LASER_OFFSET_X = 0                # 正值=激光在摄像头右侧
LASER_OFFSET_Y = 0                # 正值=激光在摄像头下方

# ★ 对准容差
TOLERANCE_X = 5                   # X方向像素容差
TOLERANCE_Y = 5                   # Y方向像素容差
CONFIRM_FRAMES = 12               # 连续对准帧数确认

# 目标丢失超时 (ms)
LOST_TIMEOUT_MS = 2000

# 串口配置
UART_PORT = 2                     # K230 UART2
UART_BAUD = 115200

# 显示配置
SHOW_OSD = True                   # 显示OSD叠加信息
PRINT_INFO = True                 # 终端打印检测信息

# ★ 单通道OSD模式 — 直接在检测分辨率(400×240)上绘制，4x加速
#   开启后省掉 chn2 的 800×480 snapshot + 大图绘制
SINGLE_CHANNEL_OSD = True

# OSD刷新间隔（帧），仅在单通道模式下生效
#   1=每帧刷新, 2=隔帧刷新, 3=每3帧
OSD_DRAW_EVERY_N = 2

# ===============================================================

SCALE_X = float(CAM_WIDTH)  / float(DET_WIDTH)   # 2.0
SCALE_Y = float(CAM_HEIGHT) / float(DET_HEIGHT)  # 2.0

# 检测分辨率下的中心和偏移
DET_CENTER_X = DET_WIDTH // 2    # 200
DET_CENTER_Y = DET_HEIGHT // 2   # 120
DET_EFF_CX = DET_CENTER_X + int(LASER_OFFSET_X / SCALE_X)
DET_EFF_CY = DET_CENTER_Y + int(LASER_OFFSET_Y / SCALE_Y)

# cos^2(90° - ANGLE_TOL), 用于快速角度检查
_angle_limit_sq = math.cos(math.radians(90 - ANGLE_TOL)) ** 2

# 环形缓冲区（处理短暂丢失）
RING_SIZE = 16
_ring_buf = [(0, 0, None)] * RING_SIZE
_ring_head = 0
_ring_count = 0


def ring_push(cx, cy, corners):
    global _ring_head, _ring_count
    _ring_buf[_ring_head] = (cx, cy, corners)
    _ring_head = (_ring_head + 1) % RING_SIZE
    if _ring_count < RING_SIZE:
        _ring_count += 1


def ring_latest():
    global _ring_count
    if _ring_count == 0:
        return None
    return _ring_buf[(_ring_head - 1) % RING_SIZE]


def ring_clear():
    global _ring_head, _ring_count
    _ring_head = 0
    _ring_count = 0


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
    """纯乘法角度检查 — 无 sqrt/acos，比原版快 ~10x"""
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


def scale_corners(c, sx, sy):
    return [(int(p[0] * sx), int(p[1] * sy)) for p in c]


def init_yolo():
    """初始化 YOLO KPU 推理。需要配合 K230 KPU API。"""
    global yolo_app
    if not YOLO_LIBS_AVAILABLE:
        print("[YOLO] KPU库不可用，回退到CV模式")
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
                ai2d_input_size = (input_image_size if input_image_size
                                   else self.rgb888p_size)
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
        return True
    except Exception as e:
        print("[YOLO] 初始化失败: %s" % e)
        print("[YOLO] 回退到CV模式")
        return False


def detect_rect_cv(img):
    """经典CV矩形检测 — 使用 find_rects 硬件加速"""
    rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)
    if not rects:
        return None

    for r in rects:
        c = r.corners()
        if len(c) != 4:
            continue
        p0, p1, p2, p3 = c[0], c[1], c[2], c[3]
        if poly_area(p0, p1, p2, p3) < AREA_MIN:
            continue
        if not aspect_ok(p0, p1, p2, p3):
            continue
        if not angle_ok(p0, p1, p2, p3):
            continue
        icx, icy = centroid(c)
        if icx < 0 or icx >= DET_WIDTH or icy < 0 or icy >= DET_HEIGHT:
            continue
        cx = int(icx * SCALE_X)
        cy = int(icy * SCALE_Y)
        return (cx, cy, c)
    return None


def detect_rect_yolo(img):
    """YOLO KPU 检测 — 返回中心坐标"""
    global yolo_app
    try:
        res = yolo_app.run(img)
        if res and len(res[0]) > 0:
            x, y, w, h = map(lambda v: int(round(v, 0)), res[0][0])
            cx = x + w // 2
            cy = y + h // 2
            return (cx, cy, None)
    except Exception as e:
        pass
    return None


def main():
    global _ring_count, yolo_app, DETECTION_MODE
    print("=" * 60)
    print("K230 矩形靶标追踪 — 电赛E题复刻版 v2.0")
    print("=" * 60)
    print("检测模式: %s" % DETECTION_MODE)
    print("分辨率:   %dx%d (检测 %dx%d)" % (CAM_WIDTH, CAM_HEIGHT,
              DET_WIDTH, DET_HEIGHT))
    print("画面中心: (%d, %d)" % (CENTER_X, CENTER_Y))
    print("激光偏移: (%+d, %+d)" % (LASER_OFFSET_X, LASER_OFFSET_Y))
    print("串口:     UART%d @ %d" % (UART_PORT, UART_BAUD))
    print("=" * 60)

    sensor_obj = None
    yolo_app = None

    try:
        # ---- 摄像头初始化 ----
        sensor_obj = Sensor(id=SENSOR_ID)
        sensor_obj.reset()

        # chn0: 全分辨率 → 硬件DMA显示
        sensor_obj.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT,
                                 chn=CAM_CHN_ID_0)
        sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        bind_info = sensor_obj.bind_info(chn=CAM_CHN_ID_0)
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

        # chn1: 低分辨率 → 检测 + OSD绘制（单通道模式）
        sensor_obj.set_framesize(width=DET_WIDTH, height=DET_HEIGHT,
                                 chn=CAM_CHN_ID_1)
        sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

        if not SINGLE_CHANNEL_OSD:
            # 传统3通道模式: chn2 全分辨率 → OSD
            sensor_obj.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT,
                                     chn=CAM_CHN_ID_2)
            sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_2)
            ch_info = "三通"
        else:
            ch_info = "单通OSD"

        Display.init(Display.ST7701, width=800, height=480,
                     to_ide=True, osd_num=1)
        print("DISP OK | ST7701 800x480")

        sensor_obj.run()
        print("CAM  OK | %sd" % ch_info)

        # ---- YOLO初始化（如需要） ----
        if DETECTION_MODE == "yolo":
            if not init_yolo():
                print("[WARN] YOLO初始化失败，使用CV模式")
                # 回退到CV
                DETECTION_MODE = "cv"

        # ---- 串口初始化 ----
        uart = None
        try:
            uart = UART(UART_PORT, UART_BAUD)
            print("UART OK | UART%d @ %d" % (UART_PORT, UART_BAUD))
        except Exception as e:
            print("UART NONE | %s" % e)

        # ---- 计算有效中心（含激光偏移） ----
        eff_cx = CENTER_X + LASER_OFFSET_X
        eff_cy = CENTER_Y + LASER_OFFSET_Y

        # ---- 状态变量 ----
        clock  = time.clock()
        frame  = 0
        lost_ms = 0
        lost_on = False
        last_dx = 0
        last_dy = 0
        last_cx = None
        last_cy = None
        last_crn = None
        last_crn_raw = None
        fb_count = 0
        aligned_count = 0

        # 丢弃前几帧
        for _ in range(10):
            sensor_obj.snapshot(chn=CAM_CHN_ID_1)

        print("LOOP...")
        while True:
            os.exitpoint()
            clock.tick()
            frame += 1

            # ==== chn1: 低分辨率检测 ====
            img_det = sensor_obj.snapshot(chn=CAM_CHN_ID_1)

            cx = None; cy = None; corners_raw = None

            if DETECTION_MODE == "cv":
                result = detect_rect_cv(img_det)
            else:
                result = detect_rect_yolo(img_det)

            if result is not None:
                cx, cy, corners_raw = result

            # ==== 追踪状态处理 ====
            if cx is not None:
                # 保存检测分辨率的角点（OSD用），cx/cy是全分辨率（UART用）
                last_crn_raw = corners_raw
                last_crn = (scale_corners(corners_raw, SCALE_X, SCALE_Y)
                            if corners_raw and not SINGLE_CHANNEL_OSD else corners_raw)
                last_cx, last_cy = cx, cy
                dx = eff_cx - cx
                dy = eff_cy - cy
                last_dx, last_dy = dx, dy
                lost_on = False
                lost_ms = 0
                fb_count = 0
                ring_push(cx, cy, last_crn)

                # 对准判断
                if abs(dx) <= TOLERANCE_X and abs(dy) <= TOLERANCE_Y:
                    aligned_count += 1
                else:
                    aligned_count = 0

                aligned = 1 if aligned_count >= CONFIRM_FRAMES else 0

                # 串口发送
                if uart is not None:
                    uart.write("%d,%d,%d,0\n" % (dx, dy, aligned))
            else:
                # 目标丢失 — 环形缓冲区回退
                rl = ring_latest()
                if rl is not None and fb_count < RING_SIZE:
                    last_cx, last_cy, last_crn = rl
                    fb_count += 1
                    dx = eff_cx - last_cx
                    dy = eff_cy - last_cy
                    last_dx, last_dy = dx, dy
                    aligned_count = 0
                    if uart is not None:
                        uart.write("%d,%d,0,0\n" % (dx, dy))
                else:
                    ring_clear()
                    fb_count = 0
                    aligned_count = 0

                    if not lost_on:
                        lost_ms = time.ticks_ms()
                        lost_on = True
                    elif time.ticks_diff(time.ticks_ms(), lost_ms) > LOST_TIMEOUT_MS:
                        if uart is not None:
                            uart.write("404,404,0,0\n")
                    else:
                        if uart is not None:
                            uart.write("%d,%d,0,0\n" % (last_dx, last_dy))

            # ==== OSD显示 ====
            if SHOW_OSD and frame % OSD_DRAW_EVERY_N == 0:
                if SINGLE_CHANNEL_OSD:
                    # 单通道: 直接在检测图(400×240)上绘制
                    osd_img = img_det
                    ocx = DET_EFF_CX
                    ocy = DET_EFF_CY
                    d_cx = int(last_cx / SCALE_X) if last_cx else None
                    d_cy = int(last_cy / SCALE_Y) if last_cy else None
                    d_crn = last_crn_raw  # 检测分辨率，无需缩放
                else:
                    # 传统: chn2 全分辨率(800×480)
                    osd_img = sensor_obj.snapshot(chn=CAM_CHN_ID_2)
                    ocx = eff_cx
                    ocy = eff_cy
                    d_cx = last_cx
                    d_cy = last_cy
                    d_crn = last_crn  # 已缩放至全分辨率

                YLW = (255, 255, 0)
                GRN = (0, 255, 0)
                RED = (255, 0, 0)
                CYN = (0, 255, 255)

                # 十字线
                osd_img.draw_line(ocx - 15, ocy, ocx + 15, ocy,
                                  color=YLW, thickness=1)
                osd_img.draw_line(ocx, ocy - 15, ocx, ocy + 15,
                                  color=YLW, thickness=1)

                # 检测框 + 误差线
                if d_cx is not None:
                    aligned = aligned_count >= CONFIRM_FRAMES
                    color = GRN if aligned else RED
                    osd_img.draw_line(ocx, ocy, d_cx, d_cy,
                                      color=color, thickness=1)
                    if d_crn and len(d_crn) == 4:
                        for i in range(4):
                            x1 = int(d_crn[i][0])
                            y1 = int(d_crn[i][1])
                            x2 = int(d_crn[(i+1)%4][0])
                            y2 = int(d_crn[(i+1)%4][1])
                            osd_img.draw_line(x1, y1, x2, y2,
                                              color=color, thickness=2)
                    osd_img.draw_circle(d_cx, d_cy, 3, color=color,
                                        thickness=1, fill=True)
                    status = "OK" if aligned else "TRK"
                    osd_img.draw_string_advanced(d_cx + 6, d_cy - 6, 14,
                                                 status, color=color)

                # FPS + 模式
                fps_str = "FPS:%d" % int(clock.fps())
                osd_img.draw_string_advanced(2, 2, 14, fps_str, color=CYN)
                osd_img.draw_string_advanced(2, 18, 12,
                                             "%s" % DETECTION_MODE.upper(),
                                             color=CYN)

                Display.show_image(osd_img, layer=Display.LAYER_OSD1)

            # 终端输出
            if PRINT_INFO and frame % 30 == 0:
                fps = int(clock.fps())
                if last_cx is not None:
                    aligned = "ALIGN" if aligned_count >= CONFIRM_FRAMES else ""
                    print("OK cx=%-4d cy=%-4d dx=%+4d dy=%+4d fps=%d %s" %
                          (last_cx, last_cy, last_dx, last_dy, fps, aligned))
                else:
                    print("NO DETECT fps=%d" % fps)
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
