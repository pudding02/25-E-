"""
    2025电赛E题 — K230 CanMV 适配版
    =================================
    基于原 MaixCam 版本，完整移植到 K230 CanMV 平台。

    原项目核心方法:
      - YOLO11 神经网络检测 A4 纸黑色外框
      - 两态状态机: IDLE(待机) / DETECTION(检测)
      - 边缘触发串口协议: 0xA1 进入检测，检测到黑框时发送 8 字节通知帧
      - 支持圆形检测、激光点定位等扩展功能（默认关闭）

    K230 适配要点:
      - maix API  → CanMV API (media.sensor / media.display / machine.UART)
      - nn.YOLO11 → KPU Pipeline (YOLOv8App)
      - 新增 CV 模式 (find_rects) 作为无模型时的备选方案

    @original  Neucrack@sipeed & lxo@sieed
    @adapt    K230 CanMV 移植
    @date     2025.8
"""

import time, gc, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import Pin

# =========================== YOLO 模块导入 ============================
# MicroPython 不允许函数内 import *，必须模块顶层导入
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

# ★ 检测模式: "cv" / "yolo"
#   cv:   经典CV find_rects 矩形检测（无需模型，推荐测试用）
#   yolo: KPU YOLO推理（需训练好的 .kmodel 模型文件）
DETECTION_MODE = "cv"


# 摄像头分辨率
CAM_WIDTH  = 800
CAM_HEIGHT = 480
SENSOR_ID  = 2                # K230 MIPI CSI 传感器ID

# 检测用低分辨率（YOLO/CV 都在此分辨率上运行）
DET_WIDTH  = 320
DET_HEIGHT = 192

# YOLO 模型路径（仅 yolo 模式）
# 注意: K230 使用 .kmodel 格式，原 MaixCam 的 .mud 模型无法直接使用
KMODEL_PATH = "/sdcard/model/best001.kmodel"
YOLO_LABELS = ["black_frame"]   # 标签名，需与训练时一致
YOLO_CONF   = 0.5               # 置信度阈值
YOLO_NMS    = 0.45              # NMS 阈值
YOLO_INPUT  = [320, 320]        # 模型输入尺寸

# CV 矩形检测参数（cv 模式）
FIND_RECTS_THRESHOLD = 15000     # 提高阈值，减少噪点候选矩形
AREA_MIN   = 200                 # 增大最小面积，过滤小噪点
ASPECT_MIN = 1.0
ASPECT_MAX = 3.0
ANGLE_TOL  = 25                  # 角度容差(度)

# ★ ROI 追踪窗口（检测分辨率下像素）
#   检测到目标后，下一帧只在目标周围搜索，大幅减少 find_rects 扫描面积
ROI_MARGIN = 80                  # 窗口半边长
ROI_LOST_MAX = 30                # ROI 内连续丢失多少帧后恢复全图搜索

# 调试开关
DEBUG = False
PRINT_TIME = False
debug_draw_err_line = False
debug_draw_err_msg  = False
debug_draw_circle   = False
debug_draw_rect     = True
debug_show_hires    = False
debug_draw_crosshair = True
print_fps_terminal  = True

# 裁切 & 圆检测参数（原项目保留，cv 模式下部分可用）
crop_padding = 12
rect_min_limit = 12
std_from_white_rect = True
circle_num_points = 50
std_res = [int(29.7 / 21 * 80), 80]

# =========================== 显示 & 摄像头 =============================

auto_awb = True
awb_gain = [0.134, 0.0625, 0.0625, 0.1139]
contrast = 80

# 上电自动进入检测模式（无需等待 MCU 发 0xA1）
AUTO_START_DETECTION = True

# =========================== 串口通信 ==================================

enable_serial_communication = True
UART_PORT = 2                  # K230 UART2
UART_BAUD = 115200

WORK_MODE_IDLE      = 0        # 待机模式
WORK_MODE_DETECTION = 1        # 检测模式
MODE1_TRIGGER_BYTE  = 0xA1     # 进入检测模式的单字节命令

# 检测到黑框时发送的 8 字节数据
BLACKLINE_DETECTED_DATA = [0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88]

# 命令帧格式（保留，用于复杂通信）
FRAME_HEADER1 = 0xAA
FRAME_HEADER2 = 0x55
FRAME_TAIL    = 0xFF
BLACK_RECT_COMMAND    = [0x11, 0x11, 0x11, 0x11]
LASER_CENTER_COMMAND  = [0x22, 0x22, 0x22, 0x22]
CENTER_ORIGIN_COMMAND = [0x33, 0x33, 0x33, 0x33]

# =====================================================================

# 全局变量
yolo_app = None
SCALE_X = float(CAM_WIDTH)  / float(DET_WIDTH)
SCALE_Y = float(CAM_HEIGHT) / float(DET_HEIGHT)

# 快速角度检查
_angle_limit_sq = math.cos(math.radians(90 - ANGLE_TOL)) ** 2


# =========================== CV 检测函数 ==============================

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
    """纯乘法角度检查，无 sqrt/acos"""
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


def detect_rect_cv(img):
    """经典CV矩形检测 + ROI追踪 — 首次全图搜索，追踪期仅在目标周围搜索"""
    global _roi_last_x, _roi_last_y, _roi_lost_count

    # 如果有上次位置，先在 ROI 内搜索
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
            # roi 参数不支持，回退全图
            rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)
            found_in_roi = False
    else:
        rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)
        found_in_roi = False

    if not rects:
        # ROI 内没找到，全图补搜
        if found_in_roi:
            rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)
            found_in_roi = False
        if not rects:
            _roi_lost_count += 1
            return None

    # 筛选最优矩形
    best = None
    best_score = 0
    for r in rects:
        c = r.corners()
        if len(c) != 4:
            continue
        p0, p1, p2, p3 = c[0], c[1], c[2], c[3]
        area = poly_area(p0, p1, p2, p3)
        if area < AREA_MIN:
            continue
        if not aspect_ok(p0, p1, p2, p3):
            continue
        if not angle_ok(p0, p1, p2, p3):
            continue
        cx, cy = centroid(c)
        if cx < 0 or cx >= DET_WIDTH or cy < 0 or cy >= DET_HEIGHT:
            continue
        # 面积越大越好
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


def detect_rect_yolo(img):
    """YOLO KPU 检测"""
    global yolo_app
    try:
        res = yolo_app.run(img)
        if res and len(res) > 0:
            x, y, w, h = map(lambda v: int(round(v, 0)), res[0][0])
            cx = x + w // 2
            cy = y + h // 2
            # 还原到全分辨率
            return (int(cx * SCALE_X), int(cy * SCALE_Y), None, (cx, cy))
    except Exception:
        pass
    return None


def detect(img):
    """统一检测入口，根据 DETECTION_MODE 选择方法"""
    if DETECTION_MODE == "cv":
        return detect_rect_cv(img)
    else:
        return detect_rect_yolo(img)


# =========================== 串口通信 =================================

def calculate_checksum(data):
    return sum(data) & 0xFF


def send_blackline_detected():
    """发送检测到黑框的 8 字节数据帧"""
    if not enable_serial_communication or uart is None:
        if DEBUG:
            print("[串口] 已禁用或未初始化")
        return
    try:
        if len(BLACKLINE_DETECTED_DATA) != 8:
            return
        data_bytes = bytes(BLACKLINE_DETECTED_DATA)
        uart.write(data_bytes)
        if DEBUG:
            print("[串口] 发送 8 字节检测帧: %s" % data_bytes)
    except Exception as e:
        if DEBUG:
            print("[串口] 发送失败: %s" % e)


def create_command_frame(command_data):
    """创建命令帧: 帧头1+帧头2+数据(4字节)+校验位+帧尾 = 8字节"""
    data_bytes = [FRAME_HEADER1, FRAME_HEADER2] + command_data
    checksum = calculate_checksum(data_bytes)
    command_frame = data_bytes + [checksum, FRAME_TAIL]
    return bytes(command_frame)


def check_serial_commands():
    """检查串口命令 — 扫描 0xA1 字节"""
    global uart
    if not enable_serial_communication or uart is None:
        return WORK_MODE_IDLE
    try:
        if uart.any() == 0:
            return WORK_MODE_IDLE
        data = uart.read(16)
        if not data:
            return WORK_MODE_IDLE
        for byte_val in data:
            if byte_val == MODE1_TRIGGER_BYTE:
                uart.write(b'ACK:DETECTION_MODE')
                print("[串口] 收到 0xA1，进入检测模式")
                return WORK_MODE_DETECTION
        return WORK_MODE_IDLE
    except Exception:
        return WORK_MODE_IDLE


# =========================== 画图工具 =================================

def draw_crosshair(img, cx, cy, size=20, color=(255, 255, 0), thickness=2):
    img.draw_line(cx - size, cy, cx + size, cy, color, thickness)
    img.draw_line(cx, cy - size, cx, cy + size, color, thickness)


# =========================== 初始化 ===================================

_t = time.ticks_ms()
def debug_time(msg):
    if PRINT_TIME:
        global _t
        now = time.ticks_ms()
        print("t: %4d %s" % (time.ticks_diff(now, _t), msg))
        _t = now


def init_yolo():
    """初始化 YOLO KPU"""
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

# ROI 追踪状态（检测分辨率坐标）
_roi_last_x = None
_roi_last_y = None
_roi_lost_count = 0

def main():
    global yolo_app, DETECTION_MODE, uart
    global _roi_last_x, _roi_last_y, _roi_lost_count

    print("=" * 60)
    print("K230 视觉检测系统 — 原版逻辑适配")
    print("=" * 60)
    print("检测模式:   %s" % DETECTION_MODE)
    print("分辨率:     %dx%d (检测 %dx%d)" %
          (CAM_WIDTH, CAM_HEIGHT, DET_WIDTH, DET_HEIGHT))
    print("串口:       UART%d @ %d" % (UART_PORT, UART_BAUD))
    print("串口通信:   %s" % ("启用" if enable_serial_communication else "禁用"))
    print("工作模式:   待机模式 (等待上位机指令)")
    print("触发命令:   0x%02X (单字节)" % MODE1_TRIGGER_BYTE)
    print("检测数据:   %s (长度:%d)" %
          ([hex(x) for x in BLACKLINE_DETECTED_DATA],
           len(BLACKLINE_DETECTED_DATA)))
    print("=" * 60)

    sensor_obj = None
    yolo_app = None
    uart = None

    try:
        # ---- 摄像头 ----
        sensor_obj = Sensor(id=SENSOR_ID)
        sensor_obj.reset()

        # chn0: 全分辨率 → 硬件 DMA 显示
        sensor_obj.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT,
                                 chn=CAM_CHN_ID_0)
        sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        bind_info = sensor_obj.bind_info(chn=CAM_CHN_ID_0)
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

        # chn1: 低分辨率 → 检测
        sensor_obj.set_framesize(width=DET_WIDTH, height=DET_HEIGHT,
                                 chn=CAM_CHN_ID_1)
        sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

        # chn2: 全分辨率 → OSD 叠加
        sensor_obj.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT,
                                 chn=CAM_CHN_ID_2)
        sensor_obj.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_2)

        Display.init(Display.ST7701, width=800, height=480,
                     to_ide=True, osd_num=1)
        print("DISP OK | ST7701 800x480")

        sensor_obj.run()
        print("CAM  OK | 三通道")

        # ---- YOLO 初始化 ----
        if DETECTION_MODE == "yolo":
            if not init_yolo():
                print("[WARN] YOLO 初始化失败，回退到 CV 模式")
                DETECTION_MODE = "cv"

        # ---- 串口 ----
        try:
            uart = UART(UART_PORT, UART_BAUD)
            print("UART OK | UART%d @ %d" % (UART_PORT, UART_BAUD))
        except Exception as e:
            print("UART NONE | %s" % e)
            uart = None

        # ---- 状态变量 ----
        center_pos = (CAM_WIDTH // 2, CAM_HEIGHT // 2)   # 画面中心（全分辨率）
        det_center = (DET_WIDTH // 2, DET_HEIGHT // 2)   # 画面中心（检测分辨率）

        current_work_mode = WORK_MODE_DETECTION if AUTO_START_DETECTION else WORK_MODE_IDLE
        last_black_rect_detected = False
        last_det_info = None         # (cx, cy, corners_det)

        clock = time.clock()
        frame = 0
        fps_counter = 0
        fps_last_time = time.ticks_ms()

        # 丢弃前 10 帧（摄像头稳定）
        for _ in range(10):
            sensor_obj.snapshot(chn=CAM_CHN_ID_1)

        print("LOOP...")

        while True:
            os.exitpoint()
            clock.tick()
            frame += 1

            # ==== 串口命令检查 ====
            new_mode = check_serial_commands()
            if new_mode == WORK_MODE_DETECTION and current_work_mode != new_mode:
                print("[系统] 当前工作模式: 检测模式")
                # 重置 ROI，全图搜索
                _roi_last_x = None
                _roi_last_y = None
                _roi_lost_count = 0
            current_work_mode = new_mode if new_mode == WORK_MODE_DETECTION else current_work_mode

            # ==== 检测图像 ====
            img_det = sensor_obj.snapshot(chn=CAM_CHN_ID_1)

            det_info = None   # (cx_full, cy_full, corners, (cx_det, cy_det))
            if current_work_mode == WORK_MODE_DETECTION:
                det_info = detect(img_det)

            black_rect_detected = det_info is not None

            # ---- 边缘触发: 首次检测到时发送串口信号 ----
            if black_rect_detected and not last_black_rect_detected:
                print("[检测] 检测到黑框，发送串口信号")
                send_blackline_detected()

            last_black_rect_detected = black_rect_detected
            if det_info is not None:
                last_det_info = det_info

            # ==== OSD 叠加 ====
            osd_img = sensor_obj.snapshot(chn=CAM_CHN_ID_2)

            # 屏幕中心十字线
            if debug_draw_crosshair:
                draw_crosshair(osd_img, center_pos[0], center_pos[1])

            if current_work_mode == WORK_MODE_DETECTION and last_det_info is not None:
                cx_full, cy_full, corners, (cx_det, cy_det) = last_det_info
                err_x = center_pos[0] - cx_full
                err_y = center_pos[1] - cy_full

                # 矩形框
                if debug_draw_rect and corners and len(corners) == 4:
                    scaled = [(int(p[0] * SCALE_X), int(p[1] * SCALE_Y))
                              for p in corners]
                    for i in range(4):
                        osd_img.draw_line(scaled[i][0], scaled[i][1],
                                          scaled[(i+1)%4][0], scaled[(i+1)%4][1],
                                          color=(255, 0, 0), thickness=2)

                # ★ 目标中心点 + 屏幕中心连线（始终显示）
                osd_img.draw_circle(cx_full, cy_full, 5,
                                    color=(0, 255, 0), thickness=1, fill=True)
                osd_img.draw_line(center_pos[0], center_pos[1],
                                  cx_full, cy_full,
                                  color=(0, 255, 255), thickness=2)

                # ★ 相对距离文字
                osd_img.draw_string_advanced(
                    cx_full + 8, cy_full - 8, 14,
                    "(%+d,%+d)" % (err_x, err_y),
                    color=(0, 255, 0))

            # 模式标识
            mode_str = "DET" if current_work_mode == WORK_MODE_DETECTION else "IDLE"
            osd_img.draw_string_advanced(2, 2, 14,
                                         "%s | %s" % (mode_str, DETECTION_MODE.upper()),
                                         color=(0, 255, 255))

            Display.show_image(osd_img, layer=Display.LAYER_OSD1)

            # ==== FPS 终端输出 ====
            if print_fps_terminal:
                fps_counter += 1
                now = time.ticks_ms()
                if time.ticks_diff(now, fps_last_time) >= 1000:
                    print("[FPS] %d" % int(clock.fps()))
                    fps_counter = 0
                    fps_last_time = now

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


# =========================== 测试函数 =================================

def test_serial_communication():
    """测试串口 — 发送 8 字节测试数据帧"""
    global uart
    if not enable_serial_communication or uart is None:
        print("[串口测试] 串口通信已禁用")
        return
    test_data = [0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x09, 0xFF]
    try:
        print("[串口测试] 发送: %s" % [hex(x) for x in test_data])
        uart.write(bytes(test_data))
        print("[串口测试] 发送完成")
    except Exception as e:
        print("[串口测试] 失败: %s" % e)


if __name__ == "__main__":
    main()
