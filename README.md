# 电赛E题 — K230 + STM32F407 矩形靶标追踪系统

基于 Simple_Tracking_Device 原项目的完整复刻，将 MaixCam 视觉端升级为 K230-CanMV，
STM32F407 云台端从开环控制升级为 PID 闭环追踪。

## 系统架构

```
┌─────────────────────┐     UART(115200)      ┌──────────────────────┐
│    K230-CanMV        │  "dx,dy,0,0\n"       │    STM32F407         │
│   (视觉处理)          │ ──────────────────→  │   (云台PID控制)       │
│                      │ ←────────────────── │                      │
│  MIPI CSI OV5640     │   控制命令(可选)      │   UART3 → X轴步进电机  │
│  LCD ST7701 800×480  │                      │   UART6 → Y轴步进电机  │
│  激光GPIO(可选)       │                      │   按键/OLED(可选)     │
└─────────────────────┘                      └──────────────────────┘
```

**与原项目的关键区别：**

| 维度 | 原项目 (MaixCam) | 复刻版 (K230) |
|------|:-----------:|:------:|
| 视觉芯片 | K210 (MaixCam) | K230 (庐山派) |
| 检测方式 | YOLO11 NN 模型 | 经典CV find_rects + 可选YOLO KPU |
| 控制方式 | 开环(检测到→转固定角度) | **PID闭环**(根据像素偏差连续调节) |
| 串口协议 | `0x88*8` 固定字节 | `"dx,dy,0,0\n"` CSV文本 |
| 分辨率 | 448×448 | 800×480 (检测用400×240) |
| 帧率 | ~25fps | ~45fps (CV模式) |

## 目录结构

```
复刻/
├── README.md                   # 本文档
├── config.yaml                 # 配置文件（K230 Linux模式用）
├── k230/
│   └── main.py                 # K230 CanMV 主程序（经典CV + YOLO双模式）
├── stm32/
│   ├── README.md               # STM32端编译说明
│   └── User/
│       ├── main.c              # 主程序（PID追踪 + 状态机）
│       ├── pid.h / pid.c       # 双轴PID控制器 (新增)
│       ├── DATOU.h / DATOU.c   # 步进电机云台控制
│       ├── frame.h / frame.c   # 数据帧处理
│       ├── frame_parser.h/c    # 帧解析器
│       ├── Key.h / Key.c       # 按键处理
│       ├── usart.h / usart.c   # 串口通信
│       ├── dma.h / dma.c       # DMA传输
│       └── gpio.h / gpio.c     # GPIO控制
└── tools/
    └── calibrate_laser.py      # 激光/光轴偏移标定工具
```

---

## 一、K230 视觉端部署

### 1.1 硬件

- 庐山派 K230-CanMV 开发板
- OV5640 MIPI CSI 摄像头（板载）
- ST7701 LCD（板载，800×480）
- 可选：5V激光模块（GPIO控制）

### 1.2 部署步骤

```bash
# 1. 将 k230/main.py 复制到 SD 卡根目录
cp k230/main.py /sdcard/main.py

# 2. （可选）如果使用 YOLO 模式，复制 kmodel
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

## 二、STM32F407 云台端

### 2.1 与原项目的兼容性

STM32 端保留原项目的**全部硬件驱动代码**，仅修改 `main.c` 和新增 `pid.c/h`：

- **DATOU.c/h** — 保留，步进电机控制协议不变
- **frame.c/h** — 保留，数据帧编解码
- **usart.c/h, dma.c/h, gpio.c/h** — 保留
- **main.c** — 重写：从按键触发的开环控制 → UART中断接收 dx,dy + PID闭环
- **pid.c/h** — 新增：双轴PID控制器

即：CubeMX 生成的 HAL 配置文件无需修改，仅替换 `Core/Src/main.c` 并添加 `pid.c/h` 即可编译。

### 2.2 编译

```bash
# 1. 用 STM32CubeIDE 打开原项目的 .ioc 文件
# 2. 将 stm32/User/ 下的所有 .c/.h 替换 Core/Src/ 和 Core/Inc/
# 3. 添加 pid.c 到编译列表
# 4. 编译 → 下载到 F407
```

---

## 三、两种检测模式

### 3.1 经典CV模式（默认，推荐）

无需模型，使用 K230 CanMV 的 `find_rects()` 硬件加速查找矩形：

- 优点：帧率高（~45fps）、不依赖模型、光照可调参
- 缺点：背景杂乱时可能误检

`k230/main.py` 中设置：
```python
DETECTION_MODE = "cv"       # 经典CV矩形检测
```

### 3.2 YOLO KPU模式（备选）

需要预先训练并转换 kmodel：

```python
DETECTION_MODE = "yolo"     # YOLO KPU推理
```

YOLO 训练→转换全流程参考 `D:\temp\电赛E题输出\02_训练配置指南.md` 和 `K230-DRONE-AI-SKILL.md`。

### 3.3 模式切换

现场可通过以下方式切换：
1. 修改 `main.py` 顶部 `DETECTION_MODE` 变量
2. （高级）通过 STM32 发送串口命令动态切换

---

## 四、通信协议

### 4.1 K230 → STM32（视觉偏差数据）

```
格式: "deltaX,deltaY,flag1,flag2\n"

正常追踪:  "-25,18,0,0\n"       # X偏差-25px, Y偏差+18px
目标丢失:  "404,404,0,0\n"      # 目标丢失通知
对准完成:  "-2,1,1,0\n"         # flag1=1表示已对准
```

### 4.2 STM32 → K230（控制命令，可选）

```
0xA1     → 进入检测模式
0xA2     → 进入待机模式
0xA3     → 切换检测方法 (CV ↔ YOLO)
```

---

## 五、PID参数调优

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
#define FINE_THRESHOLD  15      // 偏差<15px时切换到精调参数
```

调优方向：
| 现象 | 调整 |
|------|------|
| 云台振荡/过冲 | 降低 Kp，增加 Kd |
| 对准慢 | 提高 Kp |
| 有静态误差无法对准 | 增加 Ki |
| 对准后抖动 | 增加 Kd，提高 FINE_THRESHOLD |

---

## 六、激光标定

如果激光器与摄像头光轴不重合，运行标定工具：

```bash
python tools/calibrate_laser.py
```

在 `k230/main.py` 中配置偏移补偿：
```python
LASER_OFFSET_X = 0      # 激光在摄像头右侧为正
LASER_OFFSET_Y = 0      # 激光在摄像头下方为正
```

---

## 七、现场部署检查清单

- [ ] K230 与 STM32 共地
- [ ] UART 波特率一致 (115200)
- [ ] K230 摄像头画面正常
- [ ] STM32 步进电机方向正确（发正dx→电机向减小偏差方向转）
- [ ] 靶标在画面中可见且矩形清晰
- [ ] 激光偏移量已标定（如使用激光）
- [ ] 准备备用SD卡（预烧录系统+代码）

---

## 八、故障排查

| 问题 | 排查 |
|------|------|
| 检测不到矩形 | 降低 `FIND_RECTS_THRESHOLD`（10000→5000），增加 `AREA_MIN`（100→50） |
| 误检太多 | 提高 `FIND_RECTS_THRESHOLD`，提高 `AREA_MIN` |
| 云台不转 | 检查 STM32 串口接收、电机使能引脚 |
| 云台乱转 | 交换电机方向、降低 Kp |
| K230 画面卡 | 降低 `CAM_WIDTH/CAM_HEIGHT`，关闭 OSD 显示 |
| 串口无数据 | 检查接线 TX/RX、共地、波特率 |
