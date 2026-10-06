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
| 分辨率 | 448×448 | 640×360 (16:9) |
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
| `camera` / `display` / `center` | CSI 端口（`csi_id`）、输出分辨率（16:9 全幅视野）与显示偏移、追踪中心点（含激光偏移 `laser_offset_x/y`）|
| `center_calib` | **触屏中心校准**：屏幕上触摸按钮移动绿色校准点（=追踪收敛点，即 `center.laser_offset`）；`●` 隐藏 / `S` 保存写回 |
| `rectangle` | CV 矩形检测阈值（现场光照变化主要调这里）|
| `yolo` | KPU 模型路径 / labels / input_size（见第四节规则）|
| `motor` | **yaw 轴**：Emm42 速度模式（RPM 区间 / 死区 / 方向 / 使能）|
| `pitch_motor` | **pitch 轴**：Emm42 速度模式（与 yaw 同构：min_rpm/max_rpm/dy_full_px/死区/方向）|
| `red_target` | **红环检测**（与矩形检测同帧联合进行，画圈模式用）：红色掩膜/环直径/空心验证阈值、ROI 外扩、半径修正 |
| `search` | **上电搜索**：未捕获目标时 yaw 单方向匀速旋转找靶（enabled/rpm/direction/max_ms）|
| `circle` | **画圈运动**：沿红环匀速转圈的角速度/方向/误差门控（speed_deg_s/direction/pause_px）|
| `mode_pin` | **画圈模式引脚**（GPIO49/Pin3）：被供电=画圈、其余=仅追踪（enabled/gpio/active_level/debounce_n）|
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
| **GC2093 是 16:9 传感器** | 本条针对 YOLO 输入参数（`rgb888p_size` 等）：不建议设 4:3，否则初始化可能失败。CV 模式采集配置另见「故障排查」（1280×720 提示 + `crop=True` 裁切）|

## 四、云台控制（双步进）

两轴均为 Emm42_V5.0 闭环步进，协议来自 `云台/Emm_V5.0步进闭环驱动说明书Rev1.3.pdf`。
驱动器侧预置（校验 0x6B / 波特率 / ID / UART_FUN / 应答）见 `接线图.md`。

### 4.1 左右轴 (yaw) —— 速度模式

`|dx| ≤ deadband_px` → 停转；之后从 `min_rpm` 线性升到 `max_rpm`（`|dx| ≥ dx_full_px` 时满速）。

**丢靶预判**（默认关闭；先单独验证、再启用）：目标丢失时若最后偏差 `|dx| ≥ coast_edge_px`（默认 160，即最后在画面左右两侧），
不立即停，而是朝最后方向按最后偏差映射的转速续转 `coast_ms`（默认 800ms）去找（无目标追踪）；
期间重新捕获立即恢复常规追踪，超时则停。

单独验证：`[motor].coast_test` 置 `true` 上机跑——不跑视觉，自动模拟左右两侧丢靶 + 中途重捕获，观察云台续追方向/时长/停止
（串口打印 `COAST start` / `COAST stop`；与 `test_mode` 不要同时开，同开时 `coast_test` 先执行）；
验证满意后把 `coast_enabled` 改 `true` 正式启用。
`coast_enabled=false`（默认）即「丢靶即停」，不影响日常追踪。

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
| 目标甩出画面后跟丢 | 先 `coast_test` 自检预判，确认行为后 `coast_enabled` 置 true；力度不够再调大 `coast_ms` / 调小 `coast_edge_px` |
| 丢靶后乱转/干扰追踪 | `coast_enabled` 保持/置 false（回「丢靶即停」）；或调小 `coast_ms` |
| 跟踪不平滑 | 增大检测端滤波/跳帧参数 |

### 4.4 目标识别：矩形追踪 / 红环画圈（V7）

矩形检测与红色（红环）检测**每检测帧联合进行**；早期版本的红点（靶心）瞄准已删除（与矩形识别冲突），追踪以**矩形中心**为准：

| 模式 | 目标 | 串口目标源 |
|---|---|---|
| 追踪（默认） | **矩形中心** | `OK [rect]` |
| 画圈（GPIO49 被供电，见 4.5） | **红环**（环上 θ 点） | `OK [ring]` |

无有效目标 → 丢靶逻辑（coast/停）；上电未捕获期 → 自动搜索（见 4.6）。

### 4.5 画圈模式（GPIO49 被供电）

- 画圈模式下红环可见 → 光斑沿红环**连续匀速画圆**（`circle.speed_deg_s` 角速度、`direction` 方向；光斑离目标点超过 `pause_px` 时暂停 θ 推进等伺服追上）。
- 红环暂不可见 → 保持矩形追踪瞄准、**θ 冻结**（不乱转不丢靶），环恢复即续圈。
- 引脚接法：**三脚跳线**（中间=GPIO49/Pin3，两侧=3.3V/Pin1 与 GND）：拨 3.3V=画圈、拨 GND=仅追踪；**悬空不可信**（I2C0 引脚板载上拉情况以实测为准）。上电串口打印 `mode pin: GPIO49 level=..` 供核对。

### 4.6 上电搜索

- 开机后**未捕获过任何目标**时，yaw 按 `search.rpm` 单方向匀速旋转找靶；捕获目标（矩形或红环）即停并转入追踪。
- 一旦捕获过目标（发出过 track），搜索**永久退出**——后续丢靶行为与原来一致（coast/停），不会被搜索干扰。
- `search.max_ms=0` 表示一直找到为止；>0 为搜索上限（超时停转，不再重启）。

## 五、激光标定

激光与摄像头光轴不同轴时，标定 `center.laser_offset_x/y`（激光在画面中心右/下为正）。
含义：把"追踪收敛点"偏移到激光实际落点处——追踪时目标停在这个点，激光即可命中。

### 5.1 触屏现场标定（推荐，免电脑）

运行 `main.py` 后画面四缘出现触摸按钮：

| 按钮 | 位置 | 作用 |
|---|---|---|
| `▲ ▼` | 左缘 | 绿点（校准点）上 / 下移动 |
| `◀ ▶` | 右缘 | 绿点左 / 右移动 |
| `●` | 顶中 | 隐藏绿点+按钮（画面干净）；再触摸屏幕任意处唤出 |
| `S` | 顶中 | 当前偏移写回 `config.json`（自动备份 `.bak`、写后校验）|

单点一步 `step_px`（默认 4px），按住不放连发（间隔 `repeat_ms`）；绿点即时生效于追踪/画圈。
操作：把靶标与激光同处画面，用按钮把绿点拖到"激光应命中的位置"，观察云台收敛，满意后按 `S`——
串口打印 `calib: save OK saved (x,y)`，重启后仍生效。

排障：按键无反应 → 查 6P 触摸排线、看串口 `calib: touch ...` 行（触摸异常自动禁用，不影响追踪主程序）；
按 `▲` 绿点却往反方向走 → 把 `center_calib.touch_rotate` 从 -1 改 0/1/2/3 逐个试（坐标旋转）。

### 5.2 主机侧辅助（可选）

```bash
python tools/calibrate_laser.py
```

旧方式：把靶标对准画面中心，测激光点相对中心的像素偏移，手填 `config.json` 的
`center.laser_offset_x/y`（激光在右/下为正）。

## 六、主机侧自检（不接硬件）

```bash
python tools/test_emm_protocol.py    # 协议帧对拍（12 用例）
python tools/test_coast_logic.py     # 丢靶预判状态机（10 用例）
python tools/test_circle_logic.py    # V7 纯函数：红环分流/画圈几何/搜索/引脚去抖（29 用例）
python tools/test_camera_init.py     # 采集初始化守门：CSI id / 720 提示 / crop 兜底 / config 几何链（9 用例）
python tools/test_center_calib.py    # V7.7 触屏校准：按钮命中/触摸状态机/config 文本替换（32 用例）
```

五套均为 stdlib 独立脚本，从 `k230/main.py` AST 提取目标函数/常量后断言（不需要硬件、不需要 pytest）。
改动协议层先跑第一套；改动视觉/控制/配置面先跑其余几套；再上机。
（`test_circle_logic.py` 的阈值常量直接取 main.py 定义，若 main.py 漏定义会直接报错。）

## 七、现场部署检查清单

- [ ] 两台驱动器均按 `接线图.md` 预置（校验 0x6B / 115200 / UART_FUN / 应答 Receive）
- [ ] K230 与两台驱动器电源**共地**
- [ ] 上电前：云台 pitch 已摆到行程中部（步进断电会下垂）
- [ ] 上电后 `actuator:` 行两轴均 `online`
- [ ] `config.json` 中 mode、input_size、labels、task 与模型匹配（YOLO 模式）
- [ ] 摄像头画面正常
- [ ] 先单轴 `test_mode` 自检（yaw 持续旋转 / pitch 往返±角度），再跑视觉追踪
- [ ] 方向核对：发正 dx → yaw 向减小偏差方向转；正 dy → pitch 同理
- [ ] 激光偏移已标定：触屏 `▲▼◀▶` 调绿点、满意后按 `S` 保存（或手填 `center.laser_offset_x/y`）
- [ ] 画圈跳线接好：GPIO49(Pin3) 三脚跳线，拨 3.3V / GND 两态可用；上电核对 `mode pin:` 电平打印
- [ ] 红环阈值现场核对：画圈跳线拨 3.3V 后应打印 `OK [ring]` 并绕环转圈；追踪模式应打印 `OK [rect]`
- [ ] 不摆靶开机 → yaw 匀速搜索；放靶 1~2 秒内停转进入追踪
- [ ] 准备备用 SD 卡（预烧录系统+代码）

## 八、故障排查

| 问题 | 排查 |
|------|------|
| 检测不到矩形 | 降低 `rectangle.min_area`，放宽 `white_low`（CV 模式仅支持黑底白矩形）|
| 画面视野偏移（偏左/偏右）| 核查 `camera.csi_id`（须为 2）与采集提示 1280×720；**勿用 1280×960**（实测该模式视野偏左）|
| 画面横向被压缩（比例不对）| 输出须与 720 提示同为 16:9（640×360）；`set_framesize` 的 `crop=True` 为兜底（输出若被改成 4:3 会自动居中裁切而非拉伸）|
| 画面放大太多（视野窄）| 输出须为 640×360（与 720 提示同 16:9）＝全幅最大视野；4:3 输出(640×480)会触发 crop 只取中央 75% |
| 触屏校准按键无反应 | 查 6P 触摸排线；看串口 `calib: touch ...` 行（触摸异常会自动禁用，不影响追踪）；确认 `center_calib.enabled=true` |
| 触屏绿点移动方向不对 | `center_calib.touch_rotate` 从 -1 改 0/1/2/3 逐个试（触摸坐标旋转）|
| 误检太多 | 提高 `rectangle.min_area`，收窄 `min_aspect/max_aspect` |
| 帧率低 | 调大 `debug.gc_every`、关 `display.show_osd`，或将 `detection.detect_every` 设 2（隔帧检测）|
| 某轴电机不动 | 先看 `actuator:` 行是否 `online`；`(no resp)` → 按 `接线图.md` 第六节排障（共地/交叉/预置/回环）|
| 云台乱转/抖动 | 调死区 / 降低 K 参数 / 检查方向位（`reverse`）|
| pitch 电机不动/动作不对 | 与 yaw 同排查路径（接线图第六节）；确认 `[pitch_motor]` 参数与 `[motor]` 同构 |
| 电机被"卡住"后拒绝指令 | 堵转保护触发 → 检查机械卡阻，用上位机/命令解除堵转保护（0x0E 52）|
| K230 画面卡 | 增大 `debug.gc_every`，降低检测分辨率 |
| YOLO模式报错 | 确认 kmodel 路径正确、KPU 库已安装 |
| YOLO不出框 | ① `input_size` ② `labels` 顺序 ③ `task` 配对 ④ 是否拍到训练过的物体 |
| 画圈不转圈（环不见了） | 属预期退化：θ 冻结保持矩形追踪瞄准；查 `ring_min_diam_px/ring_max_diam_px` 与环的实际像素尺寸、红色阈值 |
| 画圈方向反了 | `circle.direction` 取反（1/-1）|
| 开机不搜索 | 确认 `search.enabled=true` 且开机后从未捕获过目标；已捕获过则永不搜索（设计如此）|
| 引脚切换不生效 | 核对 `mode_pin.gpio/active_level`；看启动 `mode pin:` 打印与实际跳线电平；悬空电平不可信，务必接 3.3V 或 GND |
| UART报 tx not configured | 检查 config 中 `tx_pin`/`rx_pin` 与 FPIOA 映射 |

> 串口类问题的完整排障顺序（OLED 状态 → 共地 → 回环测试 → 引脚实测）见 `接线图.md` 第六节。
