# 交接文档 — K230 双轴云台（双步进）项目

> 写给完全没有上下文的新会话（人或 agent）。本文自包含，所有关键事实已落字。
> 项目根：`C:\Users\12553\Desktop\视觉\复刻`　　git 分支：`main`，HEAD = `f912af7`（V7.7g，2026-10-06）
> 本文件由上一会话（2026 年，启明星域）在收尾时写入；2026-10-06 会话更新了 HEAD 与任务状态
> （V7.6 → `1122ea8`；V7.7 系列 → `cdee979`/`231ed79`/`7ea543a`/`1b7a2f8`/`a58929c`/`4a90458`；V7.7f 显示放大 → `8054f65`；V7.7g 修复无法运行 → `f912af7`）。
> 中间段落保留为过程档案，最新状态以文末各段「2026-10-06」更新为准。

---

## 任务目标

### 主线目标（已完成并提交）
用户原话：「**我将要改为使用2电机的方式，电机与当前使用的相同，均是42电机**」——
把云台俯仰（pitch）轴从 FashionStar 总线舵机换成与左右（yaw）轴**同款的 Emm42_V5.0 闭环步进（42 电机）**，形成双步进云台。
**非目标**：不改视觉检测链（CV/YOLO）、不改 yaw 轴既有控制逻辑。

### 「画面偏左」诊断（已解决 → 见文末「2026-10-06 更新」）
用户原话：「**现在屏幕的摄像头影像偏左，阅读代码查看是什么原因导致角度偏左**」。
实机对照官方例程后确认根因在**采集端初始化**（非显示层、非采集后处理）；已修复并提交 `1122ea8`（V7.6）。
本篇后续的历史诊断记录（候选 X/Y 表、标尺探针卡点、「下一步」清单等）保留作过程档案，**不必继续执行**。

### 项目速览（读者定位用）
- 架构：**K230-CanMV 单板直控**（无 STM32；`stm32/` 目录不存在，README 里的旧描述已修正）。
  视觉检测 → `gimbal_send(dx, dy, dist, status)`（`k230\main.py` 第四节）→ 两条轴执行器。
- 两轴**控制器数完全相同**（串口速度模式：`|偏差|` 线性映射 RPM、死区内停转、丢靶停转）：
  - yaw（左右）= 驱动器①，UART3，GPIO32(Pin37)/GPIO33(Pin40)，参数在 `config.json` `[motor]`
  - pitch（上下）= 驱动器②，UART2，GPIO5(Pin11)/GPIO6(Pin13)，参数在 `config.json` `[pitch_motor]`（与 `[motor]` 同名同构）
- 关键文件：
  - `k230\main.py` — 单文件运行时（配置加载 → EmmStepper 驱动 → 执行器初始化 → CV/YOLO 检测 → 主循环 → 入口；共约 9 节）
  - `k230\config.json` — **唯一调参面**（键以 `_` 开头=注释，程序自动跳过）
  - `接线图.md` — 接线与驱动器预置权威（两轴 UART、驱动器菜单预置表、电源共地、排障顺序）
  - `tools\test_emm_protocol.py` — 主机侧协议帧测试（AST 提取 EmmStepper 并对拍说明书例帧，12 用例）
  - `README.md` / `CLAUDE.md` / `skill\`（方法论文档，其中 `FASHIONSTAR-SERVO-K230-SKILL.md` 已标「弃用存档」）
- 运行方式：把 `k230\main.py` + `k230\config.json` 复制到 K230 的 SD 卡根目录，CanMV IDE 运行 `main.py`；单轴自检=把 `config.json` 里 `motor.test_mode` 或 `pitch_motor.test_mode` 置 `true`（不跑视觉，打印在线探测+持续旋转）。串口上电自检行形如：`actuator: yaw=emm(...) online 24000mV | pitch=emm(...) online 24000mV`。

---

## 已完成

### A. 双步进改造（已提交，3 个提交，均未 push）
| 提交 | 内容 |
|---|---|
| `157d8b0` | V3：pitch 轴改 Emm42 步进·位置模式（初版）；`k230\config.json` 的 `[servo]` 段替换为 `[pitch_motor]`；新增 `tools\test_emm_protocol.py`（第一次提交用 `deliver_task`，files=3） |
| `e691e40` | V3 文档：`接线图.md`、`README.md`、`CLAUDE.md`、`skill\` 4 个文档同步为双步进；修正 skill 中 8 处指向不存在目录 `.trae\` 的死链 |
| `231323f` | **V4：pitch 轴改回「速度模式」并完全镜像 yaw**（两轴同构）——动因见下；同步上述全部文档 |

- **V4 的动因**：用户实机反馈「驱动器1可以正常运行，但驱动器2不行，仿真1的逻辑对2进行修改」——即 pitch（位置模式）实机不工作，改为与已验证的 yaw 完全相同的速度模式逻辑（|dy|→RPM、丢靶停转、test_mode 持续旋转）。位置模式的协议方法（`move`/`clear_position`/`home`）保留在 `k230\main.py` 中作为**备用**，未接入控制流，`tools\test_emm_protocol.py` 仍覆盖它们。
- 验证（本轮工具输出）：
  - `python tools\test_emm_protocol.py` → `Ran 12 tests ... OK`
  - `python -m py_compile k230\main.py` → 通过
  - config 键 ↔ 代码读取对账（pm/mv 键无缺失、无未用）；`P_*`/`M_*` 变量无「用了但未赋值」
- **未验证**：V4 之后的**实机行为**（用户尚未回报第 2 台驱动器在速度模式下的测试结果）。

### B. 「画面偏左」诊断（进行中；诊断代码在工作区、未提交）
- 已加**临时诊断块**于 `k230\main.py:716-728`（紧跟 `Display.show_image` 之前，`py_compile` 已过）：
  - 每帧在画面上画 4 条**标尺线**：帧左边界(竖)/帧中心(竖)/帧右边界(竖)/水平中线；
  - 每 60 帧打印一行 `DIAG frame=... | disp x/y=... | lcd=... | sensor_cfg=... | center=...`
- **已获得的实机数据**（用户运行回传原文）：
  `DIAG frame=640x480 | disp x/y=80/0 lcd=800x480 | sensor_cfg=640x480 | center=320/240`
- 关键代码/配置锚点：
  - `k230\main.py:117-118` — `DISPLAY_X/DISPLAY_Y` ← `config.json` `display.x_offset/y_offset`
  - `k230\main.py:659` — `Display.init(Display.ST7701, width=800, height=480, ...)`
  - `k230\main.py:676` — `frame = sensor.snapshot(chn=CAM_CHN_ID_0)`
  - `k230\main.py:716` — `Display.show_image(frame, x=DISPLAY_X, y=DISPLAY_Y)`（**CV 模式唯一的画面摆放点**）
  - `k230\main.py:613` — `dx, dy = cx - CENTER_X, cy - CENTER_Y`（追踪偏差）
  - `k230\main.py:654-657` — `Sensor(width=1280, height=960)` → `set_framesize(640, 480)` → `set_pixformat(RGB888)`
  - `k230\config.json:20-21`（sensor 640×480）、`:32-34`（`x_offset:80` / `y_offset:0`）、`:41`（`center.x:320`，注释在 `:19`：「改这里必须同步改 [center] 的 x/y, 否则追踪中心会偏」）
- **已排除（有硬证据）**：
  1. 「实际帧宽≠640」→ 实测 `frame=640x480`，与配置一致；`x_offset=80` 按 `(800−640)/2` 算术正确。
  2. 「x_offset/显示链被最近改动碰过」→ `git blame`：`config.json:32` 自 `7fa97153`(2026-09-06) 未变；`main.py:117/654-656/716` 自 `36b9e38d`(2026-07-24) 未变；V3/V4 只改执行器与文档。
  3. 「代码里有裁剪/镜像/旋转设置」→ 全项目 grep 无 `set_windowing` / `hmirror` / `vflip` / `rotation` 调用。
- **未验证/未知**：
  - CanMV `Display.show_image(img, x, y)` 的 (x,y) 是否确为「图像左上角」——**未在外部文档核实**（web 检索失败，网络不通）；仅有项目内证据：`config.json:34` 原作者注释与数值自洽。
  - YOLO 模式的显示由 PipeLine 自理（`k230\main.py:767` `pl = PipeLine(...)`、`:833` `pl.show_image()`），与 `x_offset` 无关（当前 config `detection.mode="cv"`）。

---

## 当前卡点

**卡在：标尺探针的实机落点尚未观测。** 缩略为二选一，无法继续：

| 候选 | 机制 | 观感 | 判据（标尺落点） |
|---|---|---|---|
| **X（优先怀疑）** | `Display.show_image` 的 `x=80` 未生效（或 x,y 语义非「左上角」）→ 画面贴左显示 | 左黑边 0、右黑边 ≈160px；帧中心（=追踪中心 320）落在 LCD x=320 而非屏幕中心 400 → 所有「应该居中」的东西整体左移 ~80px | 帧**左边界线贴 LCD 最左**、右边界线离右缘 ≈160px |
| **Y** | 显示位置正常（左右各 80px），偏的是**视野/内容**（光轴/镜头/模组或 sensor 输出窗口） | 画面铺满居中但内容偏向一侧 | **左右边界线各离 LCD 边缘 ≈80px、中心线落在屏幕正中** |

- **障碍**：本机（开发机）**无 K230 硬件、无 CanMV 运行环境**，所有设备行为验证必须由人在实机执行。上一会话已两次把待验证项标为 blocked（记录在会话 todo 的 acceptance 里）。
- 排除方向见「已完成 B」的已排除清单。

---

## 下一步（按优先级，均可立即执行）

1. **（用户上机）运行当前标尺版 `k230\main.py`**，观察/拍摄 4 条线相对 LCD 边缘的落点，回传照片或文字（「三条竖线各离屏幕左右边缘多远」）。→ 判定候选 X / Y。
2. **若命中 X**：先做单变量扰动——把 `k230\config.json:32` 的 `x_offset` 从 80 改为 160，再看画面是否整体右移 80px：
   - 移动 → x 参数生效，原值/语义认知需修正（按实测重新标定 `x_offset`）；
   - 不动 → `show_image` 忽略 x（或需 `layer`/其它参数），需查 CanMV Display API 的正确用法。
3. **若命中 Y**：转硬件/光学排查（摄像头模组安装位置与光轴、镜头是否偏、`Sensor(width=1280,height=960)` 输出窗口是否为居中裁剪）。
4. **定罪后实现修复**，并同步文档：`README.md`（云台/显示相关段）、`接线图.md`（若涉及接线无关则不动）。
5. **删除临时诊断代码** `k230\main.py:716-728`（它是探针，不留进正式代码；同一次改动里一并提交）。
6. **验证**：`python tools\test_emm_protocol.py`（应 12/12 OK）+ `python -m py_compile k230\main.py`。
7. **提交**：用 `deliver_task(commit=true, message=..., files=[...], adopt=[...])`——`k230/main.py`、`k230/config.json`、`skill/01_方法论文档_视觉识别与云台控制全解析.md`、`tools/test_emm_protocol.py` 在归属台账里是 historical owned/external，**不 adopt 会被拒**（V3、V4 提交都用了 adopt）。`README.md`/`CLAUDE.md`/`接线图.md` 已是 owned。
8. （可选）`git push`：本地领先 `origin/main` **3 个提交**（`157d8b0`、`e691e40`、`231323f`），上一会话未推送。

---

## 坑（不要再踩）

- **在拿到实机标尺观测前，不要改 `x_offset`/`center`/`sensor_width` 任何一个**：三者是手工关联（`config.json:19` 注释明说同步要求），乱改会让「追踪中心」与「画面中心」错位，云台会稳定停偏。
- **`k230\main.py:716-728` 是未提交的临时诊断代码**：定罪后必须删除；忘记删 → 标尺线会永久画进正式画面。
- **提交这几个文件必须带 `adopt`**：`k230/main.py`、`k230/config.json`、`skill/01...md`、`tools/test_emm_protocol.py` 归属为 historical/external；直接 commit 会报 `File(s) not in owned files. Cannot commit`。
- **仓库根 `.gitignore` 是本地私有文件**（第 5 行有 `.gitignore` 自忽略规则，`git check-ignore -v .gitignore` 可复现）：**不要提交它**，会报 `paths are ignored`；它已被追加 `__pycache__/`、`*.pyc`（本地生效，pycache 不再出现在 git status）。
- **改 pitch 必须对照 yaw 同步改**：两轴代码同构（`k230\main.py` 第四节两条分支逐行对称；`config.json` 的 `[pitch_motor]` 与 `[motor]` 同名参数），只改一边会造成两轴行为不一致。
- **`Sensor(width=1280, height=960)` 是硬编码**（`k230\main.py:654`），**不随** `config.camera.sensor_width` 变化；`sensor_width` 只喂 `set_framesize`（`:656`）与 center 推导（`:132-133`）——改相机分辨率时别漏这条硬编码。
- **协议测试不用 pytest**：项目无测试框架标记（`run_tests` 工具会返回「无法自动推断」）；固定跑法 `python tools\test_emm_protocol.py`（stdlib unittest 风格）。
- **不要删 `EmmStepper.move/clear_position/home`**（`k230\main.py` 第三节，约 L278-315）：虽已不进控制流，但 `tools\test_emm_protocol.py` 有 4 个用例覆盖它们，删了测试会红。
- **不要用 bash 读写项目文件**（项目约定）：文件改动走 edit_file/write_file/hash_edit；实测数据类检查用 `python -c`/`grep` 只读方式。
- **别信「记忆里的行号」**：本项目行号随 AI 编辑频繁漂移；引用前先用 `grep -n` 或 `git blame -L a,b -- file` 复核（`git blame` 是本项目追溯归属的硬手段，作者 `pudding02`，2026-07~09 的提交是原始开发，之后是 AI 改动）。

---

## 2026-10-06 更新（启明星域会话）

### 已完成：采集画面偏移修复（V7.6 · 提交 `1122ea8`）
- **现象**：CV 模式画面视野偏移（偏左）。用户实机对照 `官方代码\rgb888_find_circles_1.py`：
  官方例程画面正常、本程序画面偏 → 判定差异在采集端（非显示层、非采集后处理）。
- **根因**：`Sensor(...)` 初始化在 v5（`36b9e38`）重构时丢失 CSI 端口 id（v4 原始代码为
  `SENSOR_ID = 2`），且模式提示被改为 4:3 的 `1280x960`。官方例程为 `Sensor(id=2, width=1280,
  height=720, fps=90)`——庐山派板载 GC2093 在 CSI2，应使用 16:9 模式提示。
- **修复内容**（`1122ea8`，5 文件）：`k230/main.py` 采集行 → `Sensor(id=CSI_ID, width=1280,
  height=720, fps=FPS)`；`k230/config.json [camera]` 新增 `csi_id: 2`；新增守门测试
  `tools/test_camera_init.py`（5 用例：显式 id / 16:9 提示 / CSI_ID 定义 / config 键；RED→GREEN
  已复现）；README 配置表与排障表、CLAUDE 架构注与 Testing 段同步。
- **验证**：host 侧 `test_camera_init` 5/5、`test_circle_logic` 29/29、`test_coast_logic` 10/10、
  `test_emm_protocol` 12/12、`py_compile` 全绿（当轮工具输出）。**实机验证待用户**：
  画面是否不再偏、追踪/画圈是否正常。
- **注意**：`k230/sensor_probe.py`（「画面偏左」旧诊断脚本，未提交）已过时——若以后再用，
  需先同步为 `Sensor(id=2, width=1280, height=720)` 才有对照意义。

### 仍挂账（非本任务）
- **V7.5（`b60f6a4`）提交后审查遗留**：`red_target` 段三个既有配置键 `dot_min_area` /
  `dot_max_area` / `dot_standalone` 被删（违反该次工单「不删现有配置键」约束）；补丁工已在隔离
  worktree 备好还原补丁（`hands_session:368062c3`），**尚未应用**——需在后续提交处理。
- 旧「下一步」第 5 条（删临时标尺探针）已由 `82fdc43`（V6）完成，无需再处理。
- 本地领先 `origin/main` **13 个提交**（V3 起，截至第一轮更新时），全部未推送。

### 2026-10-06 第二轮更新：触屏中心校准（V7.7 · 提交 `cdee979`）
- **用户需求**：激光与摄像头不同轴 → 加"屏幕中心校准"：触屏上下左右按键调中心、绿点可隐藏。
- **实现**：`k230/main.py` 新增「六点五」节（`_calib_rects` / `calib_hit` / `calib_step` /
  `_replace_json_int` / `calib_save_text` / `calib_save_file` / `_calib_arrow` / `calib_draw`）；
  `run_cv` 集成触摸扫描 → 调 LOX/LOY（即时作用于追踪/画圈）→ UI 绘制；`_load_json` 记录
  `CFG_PATH`；新配置段 `[center_calib]`（6 键）。交互：▲▼(左缘) / ◀▶(右缘) / ●隐藏(触摸任意处
  唤出) / S 保存（文本级写回 config.json + .bak 备份 + 读回校验）。触摸异常自动禁用（打印
  `calib: touch ...`），不影响主程序。
- **验证**：`tools/test_center_calib.py` **32 用例**（先红后绿）；既有 4 套（emm 12 / coast 10 /
  circle 29 / camera_init 5）全绿；py_compile OK。**实机验证待用户**：触摸方向（不符则调
  `[center_calib].touch_rotate` 0-3）、连发、●隐藏/唤出、S 保存后重启保持。
- HEAD `1122ea8` → `cdee979`；本地领先 origin **14 个提交**，均未推送。

### 2026-10-06 第三轮更新：修复画面横向压缩（V7.7b · 提交 `7ea543a`）
- **现象**：用户实机「屏幕比例不对, 横向被压缩」→ 要求改回原分辨率。
- **根因**：V7.6 把采集提示改为 1280x720(16:9)，与 640x480(4:3) 输出宽高比不符，
  K230 非等比缩放（X 0.5 / Y 0.667）→ 横向压缩。
- **修复**：采集提示恢复 **1280x960(4:3)**（保留 id=2）；守门测试 `test_camera_init` 断言与
  注释同步（1280x960 + 防横压语义）；README/CLAUDE 中全部「勿用 4:3 / 须 16:9」旧表述反转为
  「须与输出宽高比一致」。（绿点调小为前一提交 V7.7a: radius 6→2。）
- **验证**：`test_camera_init` 先红(720!=960)后绿；全量 5 套测试绿（12/10/29/5/32）；py_compile OK。
  **实机验证待用户**：①画面比例恢复 ②「偏左」未复发（若复发 → 说明与分辨率模式相关, 需再分离变量）。
- HEAD `cdee979` → `231ed79` → `7ea543a`；本地领先 origin **16 个提交**，均未推送。

### 2026-10-06 第四轮更新：方向+比例双满足（V7.7c · 提交 `1b7a2f8`）
- **实机反馈链（变量分离完成）**：V7.7b 改回 1280x960 后用户实机反馈「比例正常了, 但摄像头又歪了
  （偏左复发）」；结合 V7.6（720 不偏）、V7.7b（960 偏左）→ 实锤：**「偏左」由 1280x960 模式本身
  引起**（缺 id 是另一个独立缺陷）、**「横向压缩」由 720 无裁切引起**。
- **定案**：`1280x720(16:9, 方向正)` + `set_framesize(crop=True)`（居中裁 4:3 再缩, 比例正）
  —— 方向与比例同时满足；`id=2` 保留。三要素（id / 720 提示 / crop）均勿回退。
- **守门**：`test_camera_init` 新增 `crop=True` 断言（共 6 用例）、断言 720 提示；头注释记录
  V7.6/V7.7b/V7.7c 三次迭代。README/CLAUDE 同步为定案表述。
- **验证**：先红（2 失败: crop 缺失 + 960!=720）后绿；全量 89 用例（6/12/10/29/32）绿；py_compile OK。
  **实机验证待用户**：比例与方向二者同时成立（本次核心验收点）。
- HEAD `7ea543a` → `1b7a2f8`；本地领先 origin **17 个提交**，均未推送。

### 2026-10-06 第五轮更新：视野放宽（V7.7d · 提交 `a58929c`）
- **实机反馈**：V7.7c（720+crop, 输出 640x480）用户确认「可以了」但「放大太多, 要照出更多东西」。
- **方案**：crop 版 4:3 输出只用了 720 全幅的中央 75%; 改 **输出 640x360（与 720 提示同 16:9）
  = 全幅最大视野**（比 4:3 版宽 33%）, 等比缩放无变形、方向保持正; `display.y_offset=60` 垂直
  居中; `crop=True` 保留作防呆; config 三处联动同步（sensor_height 360 / center.y 180 / y_offset 60）。
- **守门**：test_camera_init 新增 `TestCameraConfigChain`（输出 16:9 全幅 / center=尺寸半 / 垂直
  居中; 共 9 用例）——先红（460800!=614400）后绿。
- **注意**：`laser_offset` 当前为 0（未校准）无换算问题; 若以后在 480 帧上校准过偏移, 切到 360
  帧需按 0.75 换算或重新触屏校准。
- **验证**：全量 92 用例（9/12/10/29/32）绿 + py_compile OK。**实机验证待用户**：
  视野变宽 + 方向正 + 比例正三者同时成立。
- HEAD `1b7a2f8` → `a58929c`；本地领先 origin **18 个提交**，均未推送。

### 2026-10-06 第六轮更新：V7.7d 实机未生效排查 + 收尾（V7.7e · 提交 `4a90458`）
- **实机反馈**：用户报「比例没变, 并没有缩小」—— V7.7d 的视野变化未出现。
- **初判**：640x360 输出定义在 config.json；若 SD 卡只换了 main.py 而 config 仍旧(480)，
  行为与 V7.7c 完全一致（画面无上下黑边、视野不变）。**需用户确认 SD 卡 config.json 的
  `sensor_height` 是否 360**（或看画面上下有无各 60px 黑边）。
- **本轮收尾改动**（`4a90458`）：① 启动打印 `geom: cam=WxH lcd=WxH off=(x,y) center=(x,y)`
  —— 串口一眼核对配置是否生效; ② DISPLAY_X/Y 缺键默认改为动态居中（(lcd-output)/2, 防老配置
  升级贴顶/偏心）; ③ 审查修正: README 无效键 `detect_resolution` 行改写、crop 行口径与新链
  一致、测试补 x_offset 居中守门（10 用例）。
- **验证**：全量 93 用例（10/12/10/29/32）绿 + py_compile OK + 缺键模拟 (80,60) OK。
- HEAD `a58929c` → `4a90458`；本地领先 origin **19 个提交**，均未推送。

### 2026-10-06 第七轮更新：显示放大（V7.7f · 提交 `8054f65`）
- **用户方向澄清**：「不是把屏幕缩小，而是让摄像头能照出更多东西」——V7.7d 的 1:1 显示让画面观感变小
  （640x360 在 480 屏上四周黑边），用户不接受；诉求 = 视野广 **且** 屏上画面不缩小。
- **方案**：采集保持 640x360（16:9 全幅视野）；**显示端** scale_view：每帧 `cv2.resize` 等比放大到
  800x450（满宽, 上下各 15px）——视野不变、占屏最大。新增纯函数 `view_geom`（scale 开=先满宽,
  超高则满高; 关=1:1 居中）驱动 VIEW_W/H/X/Y; `calib_draw` 换算改为"LCD→帧"经视图系数; CALIB_BTNS
  基于视图几何生成; 启动 geom 打印含 view= 信息。
- **配置**：`display.scale_view: true`（新键）; `x/y_offset` 语义改为"仅 1:1 模式生效"。
- **验证**：test_camera_init 15 用例（view_geom 4 + scale_view 1 新增, 先红后绿）；全量 98 用例
  （15/12/10/29/32）绿 + py_compile OK。**实机验证待用户**：①画面满宽放大（无小黑边变小感）
  ②视野仍广 ③触屏按钮命中位置准 ④帧率无明显下降（resize 每帧开销未实测）。
- **未验证风险**（本机无硬件）：`cv2.resize(frame, …)` 对帧对象的类型兼容性、每帧 resize 的性能、
  放大后触摸命中换算的实机精度——异常时备选：换 `frame.resize(w,h)` / 关 `scale_view` 回 1:1。
- HEAD `4a90458` → `8054f65`；本地领先 origin **20 个提交**，均未推送。

### 2026-10-06 第八轮更新：修复实机「无法运行」（V7.7g · 提交 `f912af7`）
- **事故**：V7.7f（cv2.resize 显示放大）上机后用户报「无法运行」。
- **诊断**：模块级新代码（view_geom / CALIB_BTNS 纯算术）主机复核无异常；崩点锁定 `cv2.resize`
  ——三嫌疑：固件无此 API / 不接受 Image 对象 / 返回类型不被 show_image 接受（每帧异常 →
  主循环退出）。**注：未取得实机错误文本**（用户未提供；下次若再异常须先要串口报错原文）。
- **修复**：改用 image 原生路径——`image.Image(w,h,image.RGB888)` 循环外预建画布 +
  `dst.draw_image(src, 0, 0, sx, sy)` 每帧等比缩放绘制（官方 image API 手册明确列出, OpenMV
  血统）; 画布复用零每帧分配; **双保险降级**：预检（画布创建）+ 每帧（绘制）任一处失败 →
  自动回退 1:1 显示 + 串口打印原因（程序不再因显示放大不可运行）。cv2.resize 全移除。
- **验证**：py_compile OK; 全量 98 用例（15/12/10/29/32）绿; 按钮布局(新视图参数 (0,15,800,450))
  主机复核无越界。**实机验证待用户**：能否运行 + 放大效果；若仍异常 → 索取串口错误文本。
- HEAD `8054f65` → `f912af7`；本地领先 origin **21 个提交**，均未推送。

### 2026-10-06 第九轮更新：显示方案定案 —— 回归 1:1 帧直显（V7.7h · 提交 `963b747`）
- **背景**：V7.7g 后用户实测仍花屏/卡死。随后用独立探针（`k230/display_probe.py`，v2~v8 共 7 版）
  在板上逐步排查，得到完整的"死亡清单"：
  - `cv2.resize` 每帧调用 → 程序无法运行；`image.draw_image` 路径 → 卡死；
  - `image.Image`(画布) 经 `show_image` 显示 → 程序终止（v2-C 实测）；
  - 输出 800x450 / 768x432（非源 1/2 缩放）→ 无论 `show_image`/`bind_layer` 均花屏；
  - `Display.bind_layer` 直通（OSD0）→ 花屏；`Display.config_layer`（无论 init 前后）→
    `sensor.run()` 卡死（v6-P/v7-D2 两个单变量对照）；
  - `deprecated function` = `MediaManager.init()` 的无害提示（非故障信号）。
  - **唯一验证可靠的显示 = 640x360 输出 + 1:1 帧直显 `show_image(frame, DISPLAY_X/Y)`**。
- **定案改动**（`963b747`，5 文件 +45/-104）：main.py 删 `view_geom`/`scale_view`/画布全链，
  显示回退单行帧直显；校准 UI 换算简化为平移（1:1 坐标系，全由 DISPLAY_X/Y 驱动）；
  `geom:` 启动打印改为 `cam=.. lcd=.. disp@(x,y) center=(x,y)`；config 删 `scale_view` 键；
  `test_camera_init` 删 view_geom/scale_view 用例、**新增 `TestNoDeadDisplayPaths`** 静态守门
  （禁 `cv2.resize`/`draw_image`/`image.Image`/`bind_layer`/`config_layer` 回归 —— 五者实机必炸）；
  README/CLAUDE 同步。
- **验证**：5 套测试 bash 逐字全绿（emm 12 / coast 10 / circle 29 / camera_init / center_calib 32）
  + py_compile；run_tests 受阻记录已附。**实机验证待用户**：拷贝 `k230/main.py` + `k230/config.json`
  到 SD 卡运行 —— 预期：`run ok` 后画面 640x360 居中显示（四周黑边），追踪/校准正常。
- **给下个会话**：显示"放大/填屏"是这块板+固件的硬边界，全部路线已试尽并证伪；若用户再提
  "画面小"，解释黑边为硬件限制，可选调整方向 = 输出尺寸/视野取舍（640x480 可视中央 75%、
  但显示更大；改 config 三处联动：sensor_height 480 + center.y 240 + y_offset 0），
  **不要**再引入软件缩放/直通绑定。探针 `display_probe.py`（untracked）保留作历史工具。
- HEAD `f912af7` → `963b747`；本地领先 origin **22 个提交**，均未推送。

### 2026-10-06 第十轮更新：触屏模式切换（V7.8 · 提交 `79aa045`）+ 黑边 UI 探针
- **V7.7h 实机确认**：用户实测「可以了」——显示方案定案通过。
- **用户新需求**：① 触屏按钮移到屏幕黑边区（不再压在画面上）；② 追踪/画圈切换改触屏操作。
- **已实现（V7.8, `79aa045`）**：新增第 7 个按钮 `T/C`（顶中）——点击切换追踪/画圈，
  串口打印 `MODE->circle/track (touch)`；切换时重置画圈 θ（与引脚逻辑一致）；`mode_pin.enabled`
  默认改 **false**（触屏为默认切换方式；引脚开启时 `T/C` 故意失效、以引脚为准——防双源打架）。
  `calib_step` 状态机零改动（新按钮名自动"单发"）；`_calib_rects` 7 键（不重叠）；`calib_draw`
  加 `mode_on` 参数（T=绿 / C=橙）。
- **黑边 UI 的技术关键（重要）**：按钮画在"帧"上 → **物理上不可能出现在黑边**（帧只覆盖画面
  区）。需新显示能力：「第二采集通道(chn1)当 UI 画布 + 双帧叠加」——chn0=640x360 主画面,
  chn1=640x480crop 涂黑+画按钮, 双帧 show 叠加, 预期后显示者在上 → L 形黑边 UI。全部踩已验证
  原语（帧显示 / 1/2·2/3 缩放输出 / cv2 绘制），但**双通道+双帧叠加从未实机用过**。
  探针 `k230/ui_probe.py`（15 秒, 两阶段测层序）**待用户跑**；通过后把按钮布局整体搬黑边
  （顶条 640x60 + 左条 80x420）；备选：黑边纯触摸热区（不绘制）。
- **验证**：全套 95 用例绿（12/10/29/11/33）+ py_compile。**实机验证待用户**：① T/C 切换 ② ui_probe。
- HEAD `963b747` → `79aa045`；本地领先 origin **23 个提交**，均未推送。

### 2026-10-06 第十一轮更新：PID 控制 + 触屏调参（V7.9）

- **用户需求**：「现在将 pid 调节参数功能也加入触屏版」。澄清后确认两件都要：
  ① 给云台控制引入真正的 PID（Kp/Ki/Kd）；② 现有云台参数（速度上限/死区/加速度/满速阈值）
  也做成触屏可调——现场点 +/- 实时生效、可保存回写 config.json。
- **实现**（`k230/main.py` + `k230/config.json` + `tools/test_pid_tune.py` + README/CLAUDE）：
  - **PID（main.py 三点五节，可选，默认关）**：速度式 `u = Kp·e + Ki·∫e·dt + Kd·de/dt`，
    输出限幅 [min_rpm 抬底, max_rpm]，抗饱和（饱和且误差同向时冻结积分），首帧 dt 33ms 兜底、
    长帧截 0.5s；死区/丢靶/上电搜索帧调 `_pid_clear()` 清状态（不跨状态积累积分）。
    `pid_enabled=false`（默认）时**完全走原线性映射**（行为逐字节不变）。
    新键 `[motor].pid_enabled/pid_kp/pid_ki/pid_kd`（默认 0.28/0/0 ≈原映射斜率）与 `[pitch_motor]` 同名（kp=0.14）。
  - **触屏调参页（main.py 六点五节扩展）**：校准页新增 `P` 按钮（现 8 键）→ 调参页 6 键
    （`+`/`-` 左缘上下、`项`/`Y-P` 右缘上下、`S`/`X` 顶中）；9 项循环
    （PID 开关→Kp→Ki→Kd→Max→Min→Dead→Acc→Full）× 2 轴；
    改动写**模块级"活参数"**（`M_*`/`P_*` 已从执行器闭包提升到第二节），**下一帧即生效**；
    `S` 把 18 项文本级写回 config.json（`.bak` + 写后读回逐项校验，全命中才写盘）；
    `_calib_skip` 防切页后同一次按压串到新页按钮。
  - **保存健壮性**：`param_save_text` 带段跟踪（motor 与 pitch_motor 的同名键互不串）；
    `acc` 调值同步进已构造电机实例（`_yaw.acc`/`_pitch.acc`）。
- **验证**：六套主机测试 **143 用例全绿**（12/10/29/11/33/48；其中新增 `tools/test_pid_tune.py`
  48 用例，先红后绿）+ `py_compile` OK + config.json JSON 合法。
  **实机验证待用户**：① 触屏调参页（P 进入 / 项·轴切换 / +/- 连发 / S 保存 / X 返回）
  ② 开 PID 后追踪行为对比原映射 ③ 新按钮命中位置（调参页 6 键 + 校准页 P 键）。
- **注意（给下个会话）**：
  - PID 默认关。用户要开就在触屏调参页把 `PID` 项调到 ON（或 config `pid_enabled=true`）。
    默认 Kp≈原映射斜率（yaw 0.28 / pitch 0.14），开了差异小属正常——用 Ki/Kd 或加大 Kp 拉开对比。
  - `M_*`/`P_*` 参数现在是模块级"活参数"（第二节），gimbal_send 每帧读全局。
    **不要再把它们改回执行器闭包内局部**——那样触屏就调不动了（本设计的核心链路）。
  - `param_save_text` 要求 config 里 18 键全存在（缺键=整体失败不写盘）。SD 卡用旧 config 时
    保存会 FAIL 并打印原因，属预期（fail loudly，用仓库内最新 config.json 即可）。
  - 顺带修复 V7.8 审查遗留两条：`PIN_ENABLED` 缺省 True→False（缺键回退与文档/config 一致）；
    config `[circle]._说明` 去除过时 GPIO49 单一归属描述。
- HEAD `79aa045` → 本次提交；本地领先 origin **24 个提交**，均未推送。

### 2026-10-06 第十二轮更新：触屏点击防抖（V7.9a）

- **用户反馈（实机）**：「将触屏延迟拉长到 0.3s，现在容易一下过两个选项」——触摸抖动被
  当成两次点击（调参页 `项` 一下跳两格）。
- **修复**：`calib_step` / `param_step` 增加**同钮防抖窗**：
  - 新签名 `(vis, btn, hold_t0, last_btn, now, pressed, was_pressed, hit, repeat_ms, guard_ms)`，
    返回 5 元组（新增 `last_btn` = 最近一次发出动作的按钮）。
  - 规则：`hit == last_btn` 且距上次动作 `< guard_ms` 的再次按沿视为抖动**吞掉**；
    换按钮不受限；`guard_ms=0` 禁用；被吞后按住不放仍走连发通道（不损失长按体验）。
  - 配置：`[center_calib].debounce_ms` 默认 **300**（即用户要求的 0.3s）；
    run_cv 集成 `_calib_last` 状态（切页时重置，跨页不沿用防抖基准）。
- **测试**：`test_center_calib` +4 / `test_pid_tune` +4 防抖用例（先红——17/9 errors——后绿）；
  全量 **151 用例**全绿（12/10/29/11/37/52）+ py_compile + JSON 合法。
- **实机验证待用户**：① 点 `项` 一下应只跳一项（不再跳两格）② <300ms 内快速换到别的按钮
  应立即响应 ③ 按住 `+`/`-` 连发手感不变。
- 调参页全参数解释已随本次交付写给用户（Kp/Ki/Kd/Max/Min/Dead/Acc/Full/PID 开关 + 现场建议）。
