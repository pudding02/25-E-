# 电赛E题 — K230 双轴云台追踪系统 (CV/YOLO + 双步进)

基于 Simple_Tracking_Device 原项目思路的完整复刻：K230-CanMV **单板**完成视觉检测与云台控制，
双轴执行器均为 **Emm42_V5.0 闭环步进**（42 电机）——左右 yaw 与上下 pitch 都走串口「速度模式」（两轴同逻辑）。

> 历史沿革：原版跑在 MaixCam (MaixPy) + STM32F407 中继架构上（K230→STM32→云台）；
> 本仓库已演进为 **K230 直控**（不经 STM32），且 pitch 轴从 FashionStar 总线舵机改为第二台 Emm42 步进。

## 系统架构

```
┌─────────────────────────┐
│       K230-CanMV        │
│  视觉检测 (CV / YOLO)    │
│  + 云台控制 (双步进)      │
│                         │
│  MIPI CSI 摄像头         │
│  LCD ST7701 800×480     │
│  激光 GPIO(可选)         │
└──────┬──────────┬───────┘
       │ UART3    │ UART2
       ▼          ▼
  驱动器①      驱动器②
  Emm42_V5.0   Emm42_V5.0
  (速度模式)    (速度模式)
       │          │
       ▼          ▼
   42 步进      42 步进
   [左右 yaw]   [上下 pitch]
```

- 完整接线见根目录 **`接线图.md`**（引脚清单 / 驱动器预置表 / 电源共地 / 排障）
- 视觉→执行器唯一接口：`gimbal_send(dx, dy, dist, status)`（在 `k230/main.py` 内）

## 与原项目的关键区别

| 维度 | 原项目 (MaixCam) | 本复刻 (K230 双步进) |
|------|:-----------:|:------:|
| 视觉芯片 | K210 (MaixCam) | K230 (庐山派) |
| 检测方式 | YOLO NN 模型 | 经典CV + YOLO KPU 双模式 |
| 云台链路 | MaixCam→STM32→执行器 | **K230 直控**（无 STM32） |
| 左右轴 (yaw) | 舵机/开环 | Emm42 步进 · 速度模式（\|dx\|→RPM） |
| 上下轴 (pitch) | 舵机/开环 | Emm42 步进 · 速度模式（与 yaw 同逻辑） |
| 串口协议 | `0x88*8` 固定字节 | Emm42 帧（固定 0x6B 校验） |
| 分辨率 | 448×448 | 640×480 (检测可选降采样至 320×240) |
| 帧率 | ~25fps | ~45fps (CV模式) |
| 配置方式 | 硬编码 | `config.json` 配置文件 |

## 目录结构

```
复刻/
├── README.md                   # 本文档
├── CLAUDE.md                   # 给 AI 助手的项目说明
├── 接线图.md                    # 接线与驱动器预置（唯一权威）
├── k230/
│   ├── main.py                 # K230 CanMV 主程序（CV+YOLO 检测 + 双步进控制）
│   └── config.json             # 运行时配置（唯一调参面）★
├── skill/                      # 方法论文档（参考用，非运行时）
│   ├── 01_方法论文档_视觉识别与云台控制全解析.md
│   ├── FASHIONSTAR-SERVO-K230-SKILL.md    # 已弃用：pitch 舵机方案存档
│   ├── K230-DRONE-AI-SKILL.md
│   ├── K230-YOLO-TRAIN-DEPLOY-SKILL.md
│   └── LUSHAN-PI-K230-PINOUT-SKILL.md     # 40Pin 引脚速查
├── 云台/                       # 硬件资料（驱动器说明书 PDF / 上位机 / 历史接线图）
└── tools/
    ├── calibrate_laser.py      # 激光/光轴偏移标定
    └── test_emm_protocol.py    # Emm42 协议帧主机侧自检（对拍说明书例帧）
```

## 一、K230 视觉端部署

### 1.1 硬件

- 庐山派 K230-CanMV 开发板（标准版 1GB）
- MIPI CSI 摄像头（板载 GC2093）
- ST7701 LCD（板载，800×480）
- 2× Emm42_V5.0 闭环步进驱动器 + 2× 42 步进电机（yaw / pitch）
- 可选：激光模块（GPIO 控制）

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

### 1.3 上电流程

1. **上电前把云台俯仰摆到行程中部**（步进断电无保持力，pitch 可能因重力下垂；两轴都建议机械居中安装）
2. 上电后核对串口第一行：`actuator:` 两轴均应 `online`
3. 首次联调先单轴自检：config 里 `motor.test_mode` / `pitch_motor.test_mode` 置 `true`，不跑视觉只看动作

## 二、配置文件说明 (config.json)

所有运行参数集中在 `k230/config.json`，改完重启生效。以 `_` 开头的键是注释，程序自动跳过。

主要段落：

| 段 | 作用 |
|---|---|
| `detection` | 检测模式（cv / yolo）与跳帧 |
| `camera` / `display` / `center` | 摄像头分辨率、LCD、追踪中心点 |
| `rectangle` | CV 矩形检测阈值（现场光照变化主要调这里）|
| `yolo` | KPU 模型路径 / labels / input_size（见第四节规则）|
| `motor` | **yaw 轴**：Emm42 速度模式（RPM 区间 / 死区 / 方向 / 使能）|
| `pitch_motor` | **pitch 轴**：Emm42 速度模式（与 yaw 同构：min_rpm/max_rpm/dy_full_px/死区/方向）|
| `debug` | 打印与 GC 节奏 |

## 三、检测模式（CV / YOLO）

### 3.1 CV 模式（默认，推荐矩形靶标）

经典计算机视觉矩形检测，使用 OpenCV 白色掩膜 + 轮廓筛选 + 黑边/白心验证。

- 优点：帧率高（~45fps）、无需训练模型、参数可现场调优
- 缺点：背景杂乱时可能误检
- **适用场景：黑背景上的白色矩形靶标（白底黑边）**

### 3.2 YOLO KPU 模式

使用 K230 KPU 进行 YOLOv8 神经网络推理，支持 detect 与 segment 两种任务。

- 前提：需预先训练并转换 kmodel（参考 `skill/K230-YOLO-TRAIN-DEPLOY-SKILL.md` 与 `skill/K230-DRONE-AI-SKILL.md`）
- 优点：可检测任意训练过的目标（无人机、水果、手势…），对复杂背景鲁棒
- 缺点：帧率较低（~15-25fps）、需提前训练模型

#### ⚠️ 关键配置规则（违反则不出框或误判满天飞）

| 规则 | 说明 |
|------|------|
| **input_size 必须等于训练 imgsz** | 模型名含 `320` → 填 `[320,320]`；不匹配会导致坐标错乱/不出框 |
| **model_path 和 task 必须配对** | `_seg_` 模型 → `"task": "segment"`；`_det_` 模型 → `"task": "detect"` |
| **labels 必须与训练时完全一致** | 顺序、数量、名称都不能错 |
| **rgb888p_size 随 task 变化** | segment → `[320, 320]`；detect → `[640, 360]` |
| **GC2093 是 16:9 传感器** | 不建议设 4:3 分辨率（如 640×480），否则传感器初始化可能失败 |

## 四、云台控制（双步进）

两轴均为 Emm42_V5.0 闭环步进，协议来自 `云台/Emm_V5.0步进闭环驱动说明书Rev1.3.pdf`。
驱动器侧预置（校验 0x6B / 波特率 / ID / UART_FUN / 应答）见 `接线图.md`。

### 4.1 左右轴 (yaw) —— 速度模式

`|dx| ≤ deadband_px` → 停转；之后从 `min_rpm` 线性升到 `max_rpm`（`|dx| ≥ dx_full_px` 时满速）。

**丢靶预判**：目标丢失时若最后偏差 `|dx| ≥ coast_edge_px`（默认 160，即最后在画面左右两侧），
不立即停，而是朝最后方向按最后偏差映射的转速续转 `coast_ms`（默认 800ms）去找（无目标追踪）；
期间重新捕获立即恢复常规追踪，超时则停。开关/参数在 `[motor]` 段 `coast_enabled` / `coast_edge_px` / `coast_ms`；
`coast_enabled=false` 回到「丢靶即停」。

### 4.2 上下轴 (pitch) —— 速度模式（与 yaw 相同逻辑）

`|dy| ≤ deadband_px` → 停转；之后从 `min_rpm` 线性升到 `max_rpm`（`|dy| ≥ dy_full_px` 时满速）；丢靶停转。

> 历史：曾试行位置模式（角度记账 + 绝对位置目标，需上电摆中位校准 0 点），实测驱动器②不工作，已改为速度模式。

### 4.3 调参速查

| 现象 | 调整 |
|------|------|
| 云台振荡/过冲 | 降对应轴 `max_rpm` / 调大 `deadband_px` |
| 对准慢 | 调大 `max_rpm` / 调小 `dx_full_px`(yaw)、`dy_full_px`(pitch)；起步肉可调大 `acc` |
| 低速"哼着不转" | 对应轴 `min_rpm` 调大（步进低速扭矩小） |
| 中心附近抖动 | 调大对应轴 `deadband_px` |
| 方向反了 | `motor.reverse` / `pitch_motor.reverse` 置 true |
| 目标甩出画面后跟丢 | `coast_ms` 调大（续追更久）/ `coast_edge_px` 调小（更易触发） |
| 丢靶后乱转 | `coast_enabled` 置 false（回「丢靶即停」）或调小 `coast_ms` |
| 跟踪不平滑 | 增大检测端滤波/跳帧参数 |

## 五、激光标定

```bash
python tools/calibrate_laser.py
```

将测量结果填入 `config.json`：
```json
"center": {
  "laser_offset_x": 5,    // 激光在摄像头右侧为正
  "laser_offset_y": -3    // 激光在摄像头下方为正
}
```

## 六、主机侧自检（不接硬件）

```bash
python tools/test_emm_protocol.py
```

从 `k230/main.py` 提取 EmmStepper 驱动，对拍说明书例帧（使能/速度/位置/清零等），
防止字节序/字段序写错。改动协议相关代码后先跑这个，再上机。

## 七、现场部署检查清单

- [ ] 两台驱动器均按 `接线图.md` 预置（校验 0x6B / 115200 / UART_FUN / 应答 Receive）
- [ ] K230 与两台驱动器电源**共地**
- [ ] 上电前：云台 pitch 已摆到行程中部（步进断电会下垂）
- [ ] 上电后 `actuator:` 行两轴均 `online`
- [ ] `config.json` 中 mode、input_size、labels、task 与模型匹配（YOLO 模式）
- [ ] 摄像头画面正常
- [ ] 先单轴 `test_mode` 自检（yaw 持续旋转 / pitch 往返±角度），再跑视觉追踪
- [ ] 方向核对：发正 dx → yaw 向减小偏差方向转；正 dy → pitch 同理
- [ ] 激光偏移量已标定（如使用激光）
- [ ] 准备备用 SD 卡（预烧录系统+代码）

## 八、故障排查

| 问题 | 排查 |
|------|------|
| 检测不到矩形 | 降低 `rectangle.min_area`，放宽 `white_low`（CV 模式仅支持黑底白矩形）|
| 误检太多 | 提高 `rectangle.min_area`，收窄 `min_aspect/max_aspect` |
| 帧率低 | 启用 `detect_resolution.enabled: true`（如配置），降低分辨率到 320×240 |
| 某轴电机不动 | 先看 `actuator:` 行是否 `online`；`(no resp)` → 按 `接线图.md` 第六节排障（共地/交叉/预置/回环）|
| 云台乱转/抖动 | 调死区 / 降低 K 参数 / 检查方向位（`reverse`）|
| pitch 电机不动/动作不对 | 与 yaw 同排查路径（接线图第六节）；确认 `[pitch_motor]` 参数与 `[motor]` 同构 |
| 电机被"卡住"后拒绝指令 | 堵转保护触发 → 检查机械卡阻，用上位机/命令解除堵转保护（0x0E 52）|
| K230 画面卡 | 增大 `debug.gc_every`，降低检测分辨率 |
| YOLO模式报错 | 确认 kmodel 路径正确、KPU 库已安装 |
| YOLO不出框 | ① `input_size` ② `labels` 顺序 ③ `task` 配对 ④ 是否拍到训练过的物体 |
| UART报 tx not configured | 检查 config 中 `tx_pin`/`rx_pin` 与 FPIOA 映射 |

> 串口类问题的完整排障顺序（OLED 状态 → 共地 → 回环测试 → 引脚实测）见 `接线图.md` 第六节。
