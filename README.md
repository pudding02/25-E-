# 电赛E题 — K230 + STM32F407 矩形靶标追踪系统

基于 Simple_Tracking_Device 原项目的完整复刻，将 MaixCam 视觉端升级为 K230-CanMV，
STM32F407 云台端从开环控制升级为 PID 闭环追踪。

## 系统架构

```
┌─────────────────────┐     UART(115200)      ┌──────────────────────┐
│    K230-CanMV       │  "dx,dy,dist,status\n"  │    STM32F407         │
│   (视觉处理)         │ ──────────────────→  │   (云台PID控制)       │
│                     │ ←──────────────────   │                      │
│  MIPI CSI GC2093    │    控制命令(可选)      │  UART3 → X轴步进电机  │
│  LCD ST7701 800×480 │                       │  UART6 → Y轴步进电机  │
│  激光GPIO(可选)      │                      │   按键/OLED(可选)     │
└─────────────────────┘                       └──────────────────────┘
```

**与原项目的关键区别：**

| 维度 | 原项目 (MaixCam) | 复刻版 (K230) |
|------|:-----------:|:------:|
| 视觉芯片 | K210 (MaixCam) | K230 (庐山派) |
| 检测方式 | YOLO11 NN 模型 | 经典CV + YOLO KPU 双模式 |
| 控制方式 | 开环(检测到→转固定角度) | **PID闭环**(根据像素偏差连续调节) |
| 串口协议 | `0x88*8` 固定字节 | `"dx,dy,0,0\n"` CSV文本 |
| 分辨率 | 448×448 | 640×480 (检测可选降采样至320×240) |
| 帧率 | ~25fps | ~45fps (CV模式) |
| 配置方式 | 硬编码 | config.json 配置文件 |

## 新增功能 (v2.0)

| 功能 | 说明 |
|------|------|
| **配置文件化** | 所有参数从 `config.json` 读取，无需修改代码即可调参 |
| **检测降采样** | 可选低分辨率检测(320×240)，检测在1/4像素上运行，帧率提升显著 |
| **UART偏差输出** | 实时向STM32发送 `dx,dy` 像素偏差，支持 track/lost/aligned 状态 |
| **YOLO KPU模式** | 支持 detect（检测框）和 segment（填充蒙版）两种推理，与CV模式可切换 |
| **卡尔曼滤波(EMA)** | 一阶指数平滑，减少检测抖动，提高对准稳定性 |
| **UART串口输出** | 实时发送 `dx,dy,dist,status` 到云台STM32，支持 FPIOA 引脚映射 |

## 目录结构

```
复刻/
├── README.md                   # 本文档
├── k230/
│   ├── config.json             # 运行时配置文件 ★
│   └── main.py                 # K230 CanMV 主程序（CV + YOLO双模式）
├── stm32/
│   ├── README.md               # STM32端编译说明
│   └── User/
│       ├── main.c              # 主程序（PID追踪 + 状态机）
│       ├── pid.h / pid.c       # 双轴PID控制器
│       ├── DATOU.h / DATOU.c   # 步进电机云台控制
│       ├── frame.h / frame.c   # 数据帧处理
│       └── Key.h / Key.c       # 按键处理
├── skill/                      # 方法论文档
│   ├── K230-DRONE-AI-SKILL.md  # K230 AI 全流程 Skill
│   ├── config_template.yaml    # 配置参考模板
│   ├── 01_方法论文档_视觉识别与云台控制全解析.md
│   └── 02_训练配置指南.md
└── tools/
    └── calibrate_laser.py      # 激光/光轴偏移标定工具
```

---

## 一、K230 视觉端部署

### 1.1 硬件

- 庐山派 K230-CanMV 开发板
- GC2093 MIPI CSI 摄像头（板载，原生1920×1080@30fps，16:9）
- ST7701 LCD（板载，800×480）
- 可选：5V激光模块（GPIO控制）

### 1.2 部署步骤

```bash
# 1. 将 k230/main.py 和 k230/config.json 复制到 SD 卡根目录
cp k230/main.py /sdcard/main.py
cp k230/config.json /sdcard/config.json

# 2. （YOLO模式）复制 kmodel 到 SD 卡
mkdir -p /sdcard/model
cp model.kmodel /sdcard/model/

# 3. SD 卡插入 K230，CanMV IDE 中打开 main.py 运行
```

### 1.3 接线

```
K230          →    STM32F407
UART2_TX      →    PA10 (USART1_RX)   视觉数据
UART2_RX      ←    PA9  (USART1_TX)   控制命令(可选)
GND           ↔    GND

K230          →    激光模块(可选)
GPIO          →    激光驱动模块
```

---

## 二、配置文件说明 (config.json)

所有运行参数集中在 `k230/config.json`，修改后重新运行即生效。关键配置项：

### 2.1 检测模式

```json
"detection": {
  "mode": "cv",           // "cv" / "yolo" / "hybrid"
  "detect_every": 1       // 跳帧检测: 1=每帧, 2=隔帧(帧率翻倍)
}
```

### 2.2 降采样加速

```json
"detect_resolution": {
  "enabled": true,        // true=降采样检测
  "width": 320,           // 检测分辨率宽
  "height": 240           // 检测分辨率高
}
```

启用后检测在 320×240 上运行（仅1/4像素量），坐标自动映射回 640×480。帧率可提升 2-4 倍。

### 2.3 卡尔曼滤波

```json
"kalman": {
  "enabled": true,
  "dt": 0.033,                // 1/帧率
  "measurement_noise": 5.0,   // 越大越平滑但响应慢
  "process_noise": 0.1        // 越大越信任原始测量
}
```

### 2.4 UART 输出

```json
"uart": {
  "enabled": true,
  "port": 2,
  "baud": 115200,
  "tx_pin": 5,             // K230 GPIO引脚号, 庐山派UART2默认 GPIO5(TX) + GPIO6(RX)
  "rx_pin": 6,             // 接线不同时修改此处
  "format": "csv"          // csv=文本协议, binary=二进制帧
}
```

> K230 的 UART 需通过 FPIOA 映射 GPIO 引脚。如报 `tx not configured` 错误，检查 `tx_pin`/`rx_pin` 是否正确。

---

## 三、三种检测模式

### 3.1 CV模式（默认，推荐矩形靶标）

经典计算机视觉矩形检测，使用 OpenCV 白色掩膜 + 轮廓筛选 + 黑边/白心验证。

- 优点：帧率高（~45fps）、无需训练模型、参数可现场调优
- 缺点：背景杂乱时可能误检
- **适用场景：黑背景上的白色矩形靶标（白底黑边）**

### 3.2 YOLO KPU模式

使用 K230 KPU 进行 YOLOv8 神经网络推理，支持 detect（检测框）和 segment（检测框+填充蒙版）两种任务类型。

- 前提：需预先训练并转换 kmodel（参考 `skill/K230-DRONE-AI-SKILL.md`）
- 优点：可检测任意训练过的目标（无人机、水果、手势…），对复杂背景鲁棒
- 缺点：帧率较低（~15-25fps）、需提前训练模型

#### ⚠️ 关键配置规则（违反则不出框或误判满天飞）

| 规则 | 说明 |
|------|------|
| **input_size 必须等于训练 imgsz** | 模型名含 `320` → 填 `[320,320]`；含 `224` → 填 `[224,224]`。不匹配会导致坐标错乱/不出框 |
| **model_path 和 task 必须配对** | `_seg_` 模型 → `"task": "segment"`；`_det_` 模型 → `"task": "detect"`。交叉使用产生随机误检 |
| **labels 必须与训练时完全一致** | 顺序、数量、名称都不能错。果实模型 → `["apple","banana","orange"]` |
| **rgb888p_size 随 task 变化** | segment → `[320, 320]`；detect → `[640, 360]` |
| **GC2093 是 16:9 传感器** | 不能设 4:3 分辨率（如 640×480），否则传感器初始化失败 |

YOLO 配置示例：

```json
// 检测模式（仅检测框）
"yolo": {
  "task": "detect",
  "model_path": "/sdcard/examples/kmodel/fruit_det_yolov8n_320.kmodel",
  "labels": ["apple","banana","orange"],
  "input_size": [320, 320],
  "confidence": 0.5,
  "nms_threshold": 0.45,
  "max_boxes": 50,
  "select": "max_conf"
}

// 分割模式（检测框 + 填充蒙版）
"yolo": {
  "task": "segment",
  "model_path": "/sdcard/examples/kmodel/fruit_seg_yolov8n_320.kmodel",
  "labels": ["apple","banana","orange"],
  "input_size": [320, 320],
  "confidence": 0.5,
  "mask_threshold": 0.5,
  ...
}
```

`select` 多目标选取策略：`max_conf`=最高置信度 / `max_area`=最大面积 / `nearest`=最靠近画面中心。

### 3.3 Hybrid模式

以CV为主检测器，CV连续失败时回退到历史位置保持。适合CV为主、偶尔需要容错的场景。

---

## 四、通信协议

### 4.1 K230 → STM32（视觉偏差数据）

```
格式: "dx,dy,dist,status\n"

正常追踪:  "-25,18,150,0\n"         # dx=-25, dy=18, dist=150px, status=0(追踪中)
对准完成:  "-2,1,5,1\n"             # status=1(已对准)
目标丢失:  "404,404,0,0\n"          # status=404(目标丢失)
```

### 4.2 STM32 → K230（控制命令，可选）

```
0xA1     → 进入检测模式
0xA2     → 进入待机模式
0xA3     → 切换检测方法 (CV ↔ YOLO)
```

---

## 五、OpenCV 提速策略总结

`k230/main.py` 的CV模式采用了多层提速设计：

| 层级 | 策略 | 原理 |
|------|------|------|
| 输入级 | **降采样检测** | 320×240 vs 640×480，像素量仅1/4 |
| 输入级 | **跳帧检测** | detect_every=N，每隔N帧才完整检测 |
| 预处理级 | **inRange颜色掩膜** | 直接生成二值图，比灰度+Canny更快 |
| 筛选级 | **面积/宽高比边界** | boundingRect O(1)，在approxPolyDP前淘汰 |
| 筛选级 | **时空连续性** | 利用上一帧位置/面积排除远距离候选 |
| 筛选级 | **ROI局部验证** | has_black_border/has_white_center只检查边框和中心 |
| 轮廓级 | **RETR_EXTERNAL** | 只取最外层轮廓，跳过嵌套层级 |
| 轮廓级 | **松逼近精度** | approxPolyDP epsilon=0.04，4%周长容差 |
| 时序级 | **丢失帧保持** | 短暂丢失时复用上一帧，避免全图搜索 |
| 时序级 | **GC/打印限流** | GC每30帧、串口打印每60帧才执行一次 |

---

## 六、PID参数调优

STM32端默认PID参数（可在 `main.c` 中修改）：

```c
// 粗调（大偏差）: 快速逼近
#define KP_COARSE   0.12f
#define KI_COARSE   0.0005f
#define KD_COARSE   0.02f

// 精调（小偏差）: 稳定对准
#define KP_FINE     0.06f
#define KI_FINE     0.0003f
#define KD_FINE     0.05f

// 切换阈值（像素偏差）
#define FINE_THRESHOLD  15
```

| 现象 | 调整 |
|------|------|
| 云台振荡/过冲 | 降低 Kp，增加 Kd |
| 对准慢 | 提高 Kp |
| 有静态误差无法对准 | 增加 Ki |
| 对准后抖动 | 增加 Kd，提高 FINE_THRESHOLD |
| 跟踪不平滑 | 在K230端开启卡尔曼滤波 |

---

## 七、激光标定

```bash
python tools/calibrate_laser.py
```

将测量结果填入 `config.json`:
```json
"center": {
  "laser_offset_x": 5,    // 激光在摄像头右侧为正
  "laser_offset_y": -3    // 激光在摄像头下方为正
}
```

---

## 八、现场部署检查清单

- [ ] K230 与 STM32 共地
- [ ] UART 波特率一致 (115200)
- [ ] `config.json` 中 mode、input_size、labels、task 与模型匹配（YOLO模式）
- [ ] YOLO模式下 `input_size` 严格等于模型训练 `imgsz`
- [ ] YOLO模式下 `task`(detect/segment) 与模型类型（_det_/_seg_）一致
- [ ] 摄像头画面正常（GC2093，16:9）
- [ ] STM32 步进电机方向正确（发正dx→电机向减小偏差方向转）
- [ ] 靶标在画面中可见且矩形清晰
- [ ] 激光偏移量已标定（如使用激光）
- [ ] 准备备用SD卡（预烧录系统+代码）

---

## 九、故障排查

| 问题 | 排查 |
|------|------|
| 检测不到矩形 | 降低 `rectangle.min_area`（3500→1000），降低 `rectangle.white_low`。注意CV模式仅支持黑底白矩形 |
| 误检太多 | 提高 `rectangle.min_area`，收窄 `rectangle.min_aspect/max_aspect` |
| 帧率低 | 启用 `detect_resolution.enabled: true`，降低分辨率到 320×240 |
| 云台不转 | 检查 `uart.enabled: true`、STM32串口接收、电机使能引脚 |
| 云台乱转/抖动 | 在K230端开启 `kalman.enabled: true`，或降低STM32端Kp |
| 跟踪不平滑 | 增大 `kalman.measurement_noise`（5→10），或调大 `tracking.smooth_num` |
| 串口无数据 | 检查 TX/RX 接线、共地、`uart.port` 和 `uart.baud` |
| K230 画面卡 | 降低 `detect_resolution`，增大 `debug.gc_every` |
| YOLO模式报错 | 确认 kmodel 路径正确、KPU库已安装 |
| YOLO不出框 | ① `input_size` 是否等于模型训练 `imgsz` ② `labels` 顺序/名称是否与训练一致 ③ `task`(detect/segment)是否与模型类型匹配 ④ 摄像头是否拍到模型训练过的物体 |
| YOLO误判满天飞 | `input_size` 与模型不匹配，或 segment 模型配了 `task: detect`（反之亦然） |
| UART报 tx not configured | 在 `config.json` 添加 `"tx_pin": 5, "rx_pin": 6`，K230 需 FPIOA 映射 |
