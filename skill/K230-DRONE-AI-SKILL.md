---
name: "k230-drone-ai"
description: "K230无人机AI视觉部署全流程：YOLOv8训练→ONNX→KModel→K230部署→FPV仿真→MAVLink自主追踪。适用于庐山派K230-CanMV开发板的AI视觉项目。"
---

# K230 无人机AI视觉 — 全流程Skill

## 简介

该Skill覆盖从数据集准备、YOLOv8训练、ONNX/KModel转换、K230实机部署、FPV仿真数据采集到MAVLink自主追踪闭环的完整流程。适用于庐山派K230-CanMV开发板的任何目标检测场景。

---

## 一、硬件平台

### 1.1 庐山派K230开发板

| 规格 | 参数 |
|------|------|
| AI芯片 | K230 SoC（双核RISC-V 64位 + KPU） |
| KPU算力 | INT8量化推理，~1 TOPS |
| 内存 | 512MB DDR3 |
| 存储 | TF卡扩展 |
| 接口 | USB-C、MIPI CSI、HDMI、UART |
| NPU对齐要求 | **16字节对齐**（输入宽高必须整除16） |

### 1.2 K230 庐山派 40-Pin 引脚定义

> 数据来源：庐山派K230开发板排针引脚表（2026-09-04 校验）

| 物理 Pin | 板上功能 | GPIO 号 | 物理 Pin | 板上功能 | GPIO 号 |
|----------|----------|---------|----------|----------|---------|
| 1 | 5V0（5V 电源输出） | — | 2 | 5V0（5V 电源输出） | — |
| 3 | I2C0_SDA | 49 | 4 | GND | GND |
| 5 | I2C0_SCL | 48 | 6 | GND | GND |
| 7 | GPIO50 | 50 | 8 | UART1_TX | 3 |
| 9 | GND | GND | 10 | UART1_RX | 4 |
| 11 | **UART2_TX** | **5** | 12 | I2C4_SDA | 47 |
| 13 | **UART2_RX** | **6** | 14 | GND | GND |
| 15 | PWM5 | 26 | 16 | QSPI0_D2 | 18 |
| 17 | 3V3（3.3V 电源输出） | — | 18 | QSPI0_D3 | 19 |
| 19 | QSPI0_DO | 16 | 20 | GND | GND |
| 21 | QSPI0_D1 | 17 | 22 | PDM_IN0 | 27 |
| 23 | QSPI0_CLK | 15 | 24 | QSPI0_CS0 | 14 |
| 25 | GND | GND | 26 | I2C1_SCL | 40 |
| 27 | I2C1_SDA | 41 | 28 | GND | GND |
| 29 | I2C3_SCL | 36 | 30 | GND | GND |
| 31 | I2C3_SDA | 37 | 32 | PWM2 | 46 |
| 33 | PWM4 | 52 | 34 | GND | GND |
| 35 | PWM0 | 42 | 36 | ADC0-1.8V | 54 |
| 37 | UART3_TX | 32 | 38 | ADC1-1.8V | 55 |
| 39 | GND | GND | 40 | UART3_RX | 33 |

**UART 引脚速查：**

| UART | TX (物理Pin / GPIO) | RX (物理Pin / GPIO) | 默认用途 |
|------|---------------------|---------------------|----------|
| UART1 | Pin 8 / GPIO 3 | Pin 10 / GPIO 4 | 调试/备用 |
| **UART2** | **Pin 11 / GPIO 5** | **Pin 13 / GPIO 6** | **视觉数据输出（本项目）** |
| UART3 | Pin 37 / GPIO 32 | Pin 40 / GPIO 33 | 扩展/备用 |

> **本项目使用 UART2**：K230 通过 GPIO5(TX)/GPIO6(RX) 发送矩形中心偏差数据 `"dx,dy,dist,status\n"` 至 STM32 或 USB-TTL 模块。

### 1.3 软件栈

| 组件 | 版本/说明 |
|------|-----------|
| SDK | K230 SDK（RTOS / Linux双系统） |
| AI编译器 | NNCase 2.8.3 |
| 模型格式 | kmodel（KPU专用） |
| 开发环境 | CanMV IDE |
| Python | **必须3.10**（高版本不兼容NNCase） |

---

## 二、完整工作流程

```
数据集准备 → YOLOv8训练 → ONNX导出 → KModel转换 → K230部署 → FPV仿真 → MAVLink追踪
```

---

## 三、阶段1：数据集准备

### 3.1 数据集目录结构

```
dataset/
├── images/
│   ├── train/     # 训练图片
│   ├── val/       # 验证图片
│   └── test/      # 测试图片（可选）
├── labels/
│   ├── train/     # YOLO格式标签
│   ├── val/
│   └── test/
└── data.yaml      # 数据集配置文件
```

### 3.2 YOLO标签格式

每张图片对应一个同名 `.txt` 文件，每行一个目标：

```
<class_id> <x_center> <y_center> <width> <height>
```

所有坐标均为**归一化相对值（0-1）**，相对于图片宽高。

### 3.3 data.yaml 配置

```yaml
path: ./dataset        # 数据集根目录
train: images/train    # 训练图片路径（相对path）
val: images/val        # 验证图片路径
test: images/test      # 测试图片路径（可选）

nc: 3                  # 类别数量

names:
  - drone              # 类别名称 — 顺序决定 class_id！
  - car
  - person
```

> **关键**：`names` 列表的顺序决定了 `class_id`（class_id 从0开始），训练和部署时必须保持完全一致。

### 3.4 量化数据集

创建独立目录，放入 **100-200张代表性图片**，用于ONNX→KModel量化校准：

```
quant_dataset/
├── scene_sunny_001.jpg
├── scene_cloudy_002.jpg
├── scene_indoor_003.jpg
└── ...
```

图片应覆盖实际场景的各种变化（角度、光照、距离、背景）。

---

## 四、阶段2：模型训练

### 4.1 输入尺寸选择

| 尺寸 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| **320×320** ★推荐 | 速度与精度最佳平衡；满足KPU 16对齐 + YOLO 32 stride | 极小目标检测稍弱 | 通用场景 |
| 224×224 | 速度最快，内存最低 | 精度下降明显 | 仅大目标/资源极度受限 |
| 416×416 | 精度较好 | 速度下降 | 中等目标 |
| 640×640 | 小目标检测好 | 速度最慢，可能内存溢出 | 远距离极小目标 |

> **硬性约束**：输入尺寸必须整除16（KPU要求）且建议整除32（YOLOv8 stride=32）。**320 = 32×10，同时满足两个约束，为K230最佳选择。**

### 4.2 训练脚本

```python
from ultralytics import YOLO

# 加载预训练模型（推荐yolov8n，最适合K230嵌入式）
model = YOLO('yolov8n.pt')

results = model.train(
    data='data.yaml',
    epochs=200,
    imgsz=320,              # ★ 与K230推理尺寸保持一致
    batch=16,
    device=0,               # GPU设备号，CPU用"cpu"
    workers=4,
    project='./runs/detect',
    name='drone_detect',
    pretrained=True,
    optimizer='SGD',
    lr0=0.01,
    cos_lr=True,
    patience=50,            # 早停耐心值
    cache=True,             # 缓存图片加速训练
    close_mosaic=10,        # 最后10轮关闭mosaic增强
    plots=True,
)
```

### 4.3 导出ONNX

```python
# 导出ONNX — opset 必须为11
model.export(
    format='onnx',
    opset=11,               # K230支持opset 11
    simplify=True,          # 简化模型图
    imgsz=320,              # 与训练尺寸一致
)
```

### 4.4 训练输出

训练完成后在 `runs/detect/drone_detect/weights/` 下找到：
- `best.pt` — 验证集最佳的权重
- `last.pt` — 最后一轮权重
- `best.onnx` — 导出的ONNX模型

### 4.5 检查训练效果

查看 `runs/detect/drone_detect/results.csv`：

| 指标 | 含义 | 目标 |
|------|------|------|
| `metrics/precision(B)` | 准确率 | > 0.8 |
| `metrics/recall(B)` | 召回率 | > 0.8 |
| `metrics/mAP50(B)` | mAP@0.5 | > 0.9 |

如效果不佳：增加训练数据量、调整学习率、增加epochs、检查标注质量。

---

## 五、阶段3：ONNX转KModel

### 5.1 环境准备

```bash
# 必须使用Python 3.10
py -3.10 -m venv k230_env
py -3.10 -m pip install numpy pillow

# 安装NNCase 2.8.3
py -3.10 -m pip install nncase-2.8.3-cp310-cp310-win_amd64.whl
py -3.10 -m pip install nncase_kpu-2.8.3-py2.py3-none-win_amd64.whl
```

### 5.2 环境变量

```python
import os
# Windows — 替换<用户名>为实际用户名
os.environ['NNCASE_PLUGIN_PATH'] = r"C:\Users\<用户名>\AppData\Local\Python\PythonCore-3.10-64\Lib\site-packages\nncase"

# 查实际路径：
import nncase
print(nncase.__file__)
```

### 5.3 完整转换脚本

```python
import os
import sys
import math
import nncase
import numpy as np
from PIL import Image

# ==================== 配置区 ====================
NNCASE_PLUGIN_PATH = r"C:\Users\<用户名>\AppData\Local\Python\PythonCore-3.10-64\Lib\site-packages\nncase"

ONNX_PATH = "best.onnx"                # 输入ONNX
OUTPUT_PATH = "model.kmodel"           # 输出kmodel
QUANT_DATASET = "quant_dataset/"       # 量化图片目录

INPUT_WIDTH = 320
INPUT_HEIGHT = 320
QUANT_SAMPLES = 100                    # 量化样本数（50-200）
# ===============================================

os.environ['NNCASE_PLUGIN_PATH'] = NNCASE_PLUGIN_PATH


def generate_quant_data(shape, batch, calib_dir):
    """生成uint8量化校准数据（0-255原始像素，不归一化）"""
    img_paths = [os.path.join(calib_dir, p) for p in os.listdir(calib_dir)
                 if p.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]

    if len(img_paths) == 0:
        raise ValueError(f"量化数据目录为空: {calib_dir}")

    print(f"找到 {len(img_paths)} 张量化图片，使用前 {min(batch, len(img_paths))} 张")

    data = []
    for i in range(min(batch, len(img_paths))):
        img = Image.open(img_paths[i]).convert('RGB')
        img = img.resize((shape[3], shape[2]), Image.BILINEAR)
        img = np.asarray(img, dtype=np.uint8)     # uint8，不归一化
        img = np.transpose(img, (2, 0, 1))        # HWC → CHW
        data.append([img[np.newaxis, ...]])

    return np.array(data)


def main():
    # 检查输入
    if not os.path.exists(ONNX_PATH):
        print(f"错误: 找不到ONNX文件: {ONNX_PATH}")
        return 1
    if not os.path.exists(QUANT_DATASET):
        print(f"错误: 找不到量化数据目录: {QUANT_DATASET}")
        return 1

    # 确保输出目录存在
    output_dir = os.path.dirname(OUTPUT_PATH)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 计算输入尺寸 — 向上取整到32的倍数
    input_width = int(math.ceil(INPUT_WIDTH / 32.0)) * 32
    input_height = int(math.ceil(INPUT_HEIGHT / 32.0)) * 32
    input_shape = [1, 3, input_height, input_width]

    print(f"ONNX: {ONNX_PATH}")
    print(f"输出: {OUTPUT_PATH}")
    print(f"输入尺寸: {input_shape}")

    # 加载ONNX
    with open(ONNX_PATH, 'rb') as f:
        onnx_content = f.read()

    # ★ 编译配置 — 关键参数（与官方样本一致）
    compile_options = nncase.CompileOptions()
    compile_options.target = "k230"
    compile_options.preprocess = True         # ★ 必须True，否则框乱飘
    compile_options.input_type = "uint8"      # ★ 必须uint8
    compile_options.input_shape = input_shape
    compile_options.input_range = [0, 1]
    compile_options.input_layout = "NCHW"
    compile_options.swapRB = False
    compile_options.mean = [0, 0, 0]
    compile_options.std = [1, 1, 1]

    compiler = nncase.Compiler(compile_options)
    compiler.import_onnx(onnx_content, nncase.ImportOptions())

    # ★ 量化配置
    cali_data = generate_quant_data(input_shape, QUANT_SAMPLES, QUANT_DATASET)
    print(f"加载了 {len(cali_data)} 张校准图片")

    ptq_options = nncase.PTQTensorOptions()
    ptq_options.quant_type = "uint8"
    ptq_options.w_quant_type = "uint8"
    ptq_options.calibrate_method = "NoClip"   # ★ 推荐NoClip，避免框乱飘
    ptq_options.samples_count = len(cali_data)
    ptq_options.set_tensor_data(cali_data)
    compiler.use_ptq(ptq_options)

    # 编译
    print("编译中（可能需要几分钟）...")
    compiler.compile()

    # 生成kmodel
    kmodel_bytes = compiler.gencode_tobytes()
    with open(OUTPUT_PATH, 'wb') as f:
        f.write(kmodel_bytes)

    size_kb = len(kmodel_bytes) / 1024
    print(f"转换成功！输出: {OUTPUT_PATH} ({size_kb:.1f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

### 5.4 关键配置速查表

| 配置项 | 正确值 | 错误值（会导致问题） |
|--------|--------|---------------------|
| `preprocess` | **True** | False → 框乱飘 |
| `input_type` | **"uint8"** | "float32" → 框乱飘 |
| `calibrate_method` | **"NoClip"** ★ | 官方默认 "Kld" 也可用，但NoClip经社区验证更稳定 |
| 校准数据格式 | **uint8 (0-255)** | 归一化到0-1 → 框乱飘 |
| Python版本 | **3.10** | 3.11+ → NNCase不兼容 |
| NNCase版本 | **2.8.3** | 其他版本 → 可能KPU运行失败 |
| opset | **11** | 高版本opset不被K230支持 |

> **关于 `calibrate_method`**：NNCase官方文档提供两种选项 — `"NoClip"`（直接min/max范围）和 `"Kld"`（KL散度，官方默认）。社区大量实践验证 `"NoClip"` 能稳定解决K230上检测框乱飘问题，因此本Skill推荐使用 `"NoClip"`。

---

## 六、阶段4：K230部署

### 6.1 SD卡文件结构

```
SD卡/
├── model/
│   └── model.kmodel      # 转换好的kmodel文件
└── main.py                # 部署主程序
```

### 6.2 完整部署代码模板

```python
"""K230 YOLOv8目标检测部署
适用于庐山派K230-CanMV开发板
"""
from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, gc, math
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo

# ==================== 配置区 ====================
KMODEL_PATH = "/sdcard/model/model.kmodel"

# ★ 类别列表 — 必须与data.yaml中的names顺序完全一致
LABELS = ["drone", "car", "person"]

MODEL_INPUT_SIZE = [320, 320]      # 必须与训练尺寸一致
CONFIDENCE_THRESHOLD = 0.3         # 置信度阈值 (0.1-0.7)
NMS_THRESHOLD = 0.4                # NMS阈值 (0.2-0.6)
MAX_BOXES_NUM = 30                 # 最大检测框数

DISPLAY_MODE = "lcd"               # "lcd" 或 "hdmi"
# ===============================================


class YOLOv8App(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num,
                 confidence_threshold=0.3, nms_threshold=0.4,
                 rgb888p_size=[320, 320], display_size=[1920, 1080],
                 debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num

        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        self.color_four = get_colors(len(self.labels))

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8, np.uint8
        )

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size
            )
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [128, 128, 128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]]
            )

    def preprocess(self, input_np):
        with ScopedTiming("preprocess", self.debug_mode > 0):
            return [nn.from_numpy(input_np)]

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            new_result = results[0][0].transpose()
            det_res = aidemo.yolov8_det_postprocess(
                new_result.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [self.display_size[1], self.display_size[0]],
                len(self.labels),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num
            )
            return det_res

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                for i in range(len(dets[0])):
                    x, y, w, h = map(lambda x: int(round(x, 0)), dets[0][i])
                    class_id = dets[1][i]
                    score = dets[2][i]

                    pl.osd_img.draw_rectangle(
                        x, y, w, h,
                        color=self.color_four[class_id],
                        thickness=4
                    )

                    label = f" {self.labels[class_id]} {score:.2f} "
                    pl.osd_img.draw_string_advanced(
                        x, y - 50, 32,
                        label,
                        color=self.color_four[class_id]
                    )
            else:
                pl.osd_img.clear()


def main():
    print("=" * 60)
    print("YOLOv8 目标检测 — K230部署")
    print("=" * 60)
    print(f"模型: {KMODEL_PATH}")
    print(f"类别数: {len(LABELS)} ({', '.join(LABELS)})")
    print(f"输入尺寸: {MODEL_INPUT_SIZE}")
    print(f"置信度阈值: {CONFIDENCE_THRESHOLD}")
    print("=" * 60)

    pl = PipeLine(
        rgb888p_size=MODEL_INPUT_SIZE,
        display_mode=DISPLAY_MODE,
        display_size=None
    )
    pl.create()
    display_size = pl.get_display_size()

    det = YOLOv8App(
        KMODEL_PATH,
        labels=LABELS,
        model_input_size=MODEL_INPUT_SIZE,
        max_boxes_num=MAX_BOXES_NUM,
        confidence_threshold=CONFIDENCE_THRESHOLD,
        nms_threshold=NMS_THRESHOLD,
        rgb888p_size=MODEL_INPUT_SIZE,
        display_size=display_size,
        debug_mode=0
    )
    det.config_preprocess()

    print("开始检测...")
    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = det.run(img)
                det.draw_result(pl, res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("\n停止检测")
    finally:
        det.deinit()
        pl.destroy()


if __name__ == "__main__":
    main()
```

### 6.3 部署配置说明

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `KMODEL_PATH` | `/sdcard/model/model.kmodel` | SD卡上的kmodel路径 |
| `LABELS` | 与data.yaml一致 | 类别顺序决定class_id |
| `CONFIDENCE_THRESHOLD` | 0.3 | 0.1-0.7，见调参策略 |
| `NMS_THRESHOLD` | 0.4 | 0.2-0.6 |
| `MODEL_INPUT_SIZE` | [320, 320] | 必须与训练imgsz一致 |

### 6.4 部署步骤

1. 将 `model.kmodel` 复制到SD卡 `model/` 目录
2. 将部署脚本保存为 `main.py` 复制到SD卡根目录
3. SD卡插入K230开发板
4. 在CanMV IDE中打开 `main.py`，点击运行

---

## 七、阶段5：FPV仿真训练

### 7.1 仿真平台说明

> **AirSim状态**：微软原始AirSim已于2022年归档停维。替代方案：
> - **Colosseum** — AirSim直接后继，支持PX4/ArduPilot SITL+HITL，UE+Unity，推荐首选
> - **Cosys-AirSim** — UE5适配版，高级传感器支持
> - **Gazebo SITL** — PX4官方维护，兼容性最好
>
> 以下代码基于AirSim API，Colosseum/Cosys-AirSim API兼容。

### 7.2 AirSim / Colosseum 安装

```bash
# Colosseum（推荐）
git clone https://github.com/dronesimulator/Colosseum.git
cd Colosseum
./setup.sh
./build.sh

# 或 Cosys-AirSim（UE5）
git clone https://github.com/FlywardAero/Cosys-AirSim-102024.git
cd Cosys-AirSim-102024
./setup.sh
./build.sh
```

### 7.3 仿真配置文件

`settings.json`:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone": {
      "VehicleType": "SimpleFlight",
      "AutoStart": true
    }
  },
  "CameraDefaults": {
    "CaptureSettings": [
      {
        "ImageType": 0,
        "Width": 640,
        "Height": 480,
        "FOV_Degrees": 90
      }
    ]
  }
}
```

### 7.4 FPV数据采集脚本

```python
import airsim
import cv2
import os
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()

output_dir = "fpv_data/images"
os.makedirs(output_dir, exist_ok=True)

count = 0
try:
    while True:
        responses = client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        ])
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
        cv2.imwrite(f"{output_dir}/frame_{count:06d}.png", img_rgb)
        count += 1
        if count % 100 == 0:
            print(f"已采集 {count} 帧")
except KeyboardInterrupt:
    print(f"采集结束，共 {count} 帧")
```

---

## 八、阶段6：FPV自主追踪闭环

### 8.1 图像坐标误差（PID视觉伺服）

```python
# 图像中心 (cx, cy)；画面中心 (center_x, center_y)
error_x = cx - center_x   # 水平像素误差
error_y = cy - center_y   # 垂直像素误差

# P控制（可扩展为PID）
kp_x, kp_y = 0.005, 0.005
yaw_cmd   = kp_x * error_x     # 偏航角速度指令
pitch_cmd = kp_y * error_y     # 俯仰角速度指令
```

### 8.2 单目距离估计（小孔成像）

```python
def estimate_distance(box_width_px, target_real_width=0.4, focal_length=700):
    """
    利用已知目标宽度和焦距估计距离

    参数:
        box_width_px: 检测框像素宽度
        target_real_width: 目标真实宽度（米），如无人机约0.4m
        focal_length: 相机焦距（像素），需标定，默认700

    返回:
        距离（米）
    """
    if box_width_px <= 0:
        return float('inf')
    return target_real_width * focal_length / box_width_px
```

### 8.3 完整追踪控制循环

```python
import airsim
import cv2
import numpy as np
from ultralytics import YOLO

# 初始化
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.takeoffAsync().join()

model = YOLO('best.pt')   # 训练好的YOLOv8模型

CENTER_X, CENTER_Y = 320, 240   # 640x480画面中心
KP_YAW, KP_PITCH = 0.005, 0.005
TARGET_DISTANCE = 5.0            # 期望跟踪距离（米）

try:
    while True:
        # 获取FPV图像
        responses = client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        ])
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img = img1d.reshape(responses[0].height, responses[0].width, 3)

        # YOLOv8检测
        results = model(img, verbose=False)
        if len(results[0].boxes) > 0:
            # 取置信度最高的检测框
            box = results[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            box_w = x2 - x1

            # 误差计算
            error_x = cx - CENTER_X
            error_y = cy - CENTER_Y

            # 距离估计
            dist = estimate_distance(box_w)
            dist_error = dist - TARGET_DISTANCE

            # PID控制
            yaw_cmd = KP_YAW * error_x
            pitch_cmd = KP_PITCH * error_y
            throttle_cmd = -0.02 * dist_error  # 距离保持

            # 发送控制指令
            client.moveByVelocityBodyFrameAsync(
                2.0,              # vx 前向速度
                0,                # vy 侧向速度（用yaw替代）
                throttle_cmd,     # vz 垂直速度
                0.5,              # 持续时间
                yaw_mode=airsim.YawMode(True, yaw_cmd)
            )
        else:
            # 无目标时悬停
            client.hoverAsync()

except KeyboardInterrupt:
    client.hoverAsync()
    client.enableApiControl(False)
```

---

## 九、MAVLink控制

### 9.1 MAVLink连接

```python
from pymavlink import mavutil

# 连接飞控（串口或UDP）
master = mavutil.mavlink_connection('COM5', baud=57600)
# 或UDP：master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# 等待心跳
master.wait_heartbeat()
print("MAVLink已连接")
```

### 9.2 速度控制指令

```python
def send_velocity(vx, vy, vz, yaw_rate=0):
    """发送NED坐标系速度控制指令"""
    master.mav.set_position_target_local_ned_send(
        0,                              # 时间戳（0=立即）
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,             # 只控制vx, vy, vz, yaw_rate
        0, 0, 0,                        # 位置（忽略）
        vx, vy, vz,                     # 速度 (m/s) NED坐标系
        0, 0, 0,                        # 加速度（忽略）
        yaw_rate, 0                     # 偏航角速度
    )
```

### 9.3 AI检测 + MAVLink闭环

```python
def ai_track_and_control(detection_result, image_center):
    """AI检测结果转MAVLink控制"""
    if detection_result is None:
        # 无目标 — 悬停
        send_velocity(0, 0, 0, 0)
        return

    cx, cy, box_w = detection_result
    error_x = cx - image_center[0]
    error_y = cy - image_center[1]

    # PID参数
    KP_YAW, KP_PITCH = 0.005, 0.005
    yaw_cmd   = KP_YAW * error_x
    pitch_cmd = KP_PITCH * error_y

    # 距离控制
    dist = estimate_distance(box_w)
    vz_cmd = -0.02 * (dist - 5.0)  # 保持5米距离

    send_velocity(2.0, 0, vz_cmd, yaw_cmd)
```

---

## 十、AirSim + PX4 + Mission Planner 联合仿真

### 10.1 架构

```
AirSim / Colosseum
       │
       ▼
  PX4 SITL        ← MAVLink →  Mission Planner (地面站)
       │
       ▼
  MAVLink消息
       │
       ▼
  AI视觉程序 (YOLOv8)
```

### 10.2 功能矩阵

| 功能 | 实现方式 |
|------|----------|
| 目标检测 | YOLOv8 → K230 / PC端推理 |
| 视觉跟踪 | PID误差 → MAVLink速度指令 |
| 自主拦截 | 距离估计 + 速度前馈 |
| 避障 | 深度图/检测框大小预警 |
| 自动返航 | MAVLink RTL指令 |

---

## 十一、调参策略速查

| 问题 | 操作 |
|------|------|
| 误检太多 | 提高置信度阈值：0.3 → 0.4 → 0.5 |
| 漏检太多 | 降低置信度阈值：0.3 → 0.25 → 0.2 |
| 框重叠/重复 | 降低NMS阈值：0.4 → 0.35 → 0.3 |
| 框太松散 | 提高NMS阈值：0.4 → 0.45 → 0.5 |
| 类别识别错误 | 检查LABELS顺序是否与data.yaml一致 |
| 小目标检测差 | 提高输入尺寸：320 → 416 → 640 |
| 帧率太低 | 降低输入尺寸：320 → 224；或用yolov8n |

---

## 十二、常见问题排查

### 12.1 检测框乱飘

**症状**：框位置不稳定，在画面中随机移动

**原因**：模型转换配置错误（最常见）

**解决**：
1. `compile_options.preprocess = True`
2. `compile_options.input_type = "uint8"`
3. `ptq_options.calibrate_method = "NoClip"`
4. 校准数据使用 uint8 格式（0-255原始像素，不归一化）

### 12.2 KPU run failed

**症状**：程序报错 `RuntimeError: KPU run failed.`

**排查**：
- NNCase版本是否为 2.8.3
- Python版本是否为 3.10
- 模型文件是否完整（重新复制kmodel）
- 输入尺寸是否满足16/32对齐要求

### 12.3 无检测框

**排查步骤**：
1. 确认 `LABELS` 与 `data.yaml` 中 `names` 顺序一致
2. 确认 `nc`（类别数）正确
3. 临时降低 `CONFIDENCE_THRESHOLD = 0.1` 测试
4. 检查模型输入尺寸是否与训练一致

### 12.4 精度下降（量化后）

**解决**：
- 增加量化样本数（100 → 200张）
- 使用更具代表性的量化图片
- 尝试 `calibrate_method = "Kld"`（仅当NoClip效果不佳时）

### 12.5 程序卡死

**排查**：
- 设置 `debug_mode = 0`（关闭调试输出）
- 主循环中 `gc.collect()` 不要调用太频繁
- 确认模型大小在K230内存范围内

---

## 十三、项目文件结构

```
drone_ai_project/
├── dataset/                    # 训练数据集
│   ├── images/
│   │   ├── train/
│   │   └── val/
│   ├── labels/
│   │   ├── train/
│   │   └── val/
│   └── data.yaml
├── quant_dataset/              # 量化图片（100-200张）
│   └── *.jpg
├── fpv_data/                   # FPV仿真采集数据
│   └── images/
├── train_yolov8.py             # 训练脚本
├── convert_to_kmodel.py        # ONNX→KModel转换脚本
├── deploy_main.py              # K230部署脚本
├── fpv_collect.py              # FPV数据采集脚本
├── fpv_track.py                # FPV自主追踪脚本
├── mavlink_control.py          # MAVLink控制脚本
├── runs/                       # 训练输出（自动生成）
│   └── detect/
│       └── drone_detect/
│           ├── weights/
│           │   ├── best.pt
│           │   └── best.onnx
│           └── results.csv
└── output/
    └── model.kmodel            # 最终部署模型
```

---

## 十四、配置速查总表

### 训练配置

| 配置项 | 推荐值 | 说明 |
|--------|--------|------|
| `imgsz` | 320 | K230最佳尺寸（16/32对齐） |
| `epochs` | 200 | 充分训练 |
| `batch` | 16 | 根据GPU显存调整 |
| `optimizer` | SGD | |
| `lr0` | 0.01 | 初始学习率 |
| `opset` | 11 | K230支持版本 |

### 转换配置（★ = 必须严格遵守）

| 配置项 | 正确值 | ★ |
|--------|--------|---|
| nncase版本 | 2.8.3 | ✅ |
| Python版本 | 3.10 | ✅ |
| `preprocess` | True | ✅ |
| `input_type` | "uint8" | ✅ |
| `calibrate_method` | "NoClip" | ✅ |
| 校准数据格式 | uint8 (0-255) | ✅ |

### 部署配置

| 配置项 | 默认值 | 可调范围 |
|--------|--------|----------|
| `CONFIDENCE_THRESHOLD` | 0.3 | 0.1 - 0.7 |
| `NMS_THRESHOLD` | 0.4 | 0.2 - 0.6 |
| `MODEL_INPUT_SIZE` | [320, 320] | [224, 224] / [416, 416] |

---

## 十五、K230 CanMV 性能优化实战 ⚡

> 以下经验来自 K230 庐山派实机测试，基于经典CV矩形检测场景。

### 15.1 核心原则

在 K230 CanMV MicroPython 环境下，代码写法对帧率影响极大：

| 原则 | 快速写法 | 慢速写法 | 原因 |
|------|---------|---------|------|
| 热路径零分支 | `kf_update = lambda: ...` 注入 | `if kf_enabled: kf.update()` | MicroPython分支无JIT优化 |
| 全局变量直读 | `WHITE_LOW` 模块级变量 | `self.white_low` 类属性 | 对象属性查找慢5-10倍 |
| 返回元组 | `return best, count, area` | `return {"box": ..., "count": ...}` | 避免dict分配触发GC |
| 函数无封装 | `def find_paper_box(...)` | `class Detector: def detect(...)` | 方法调用有额外开销 |
| 字符串用% | `"FPS:%d" % fps` | `f"FPS:{fps}"` | f-string在MicroPython中未优化 |

### 15.2 Lambda注入模式

可选功能（卡尔曼、UART）通过lambda注入，热路径中无 `if` 判断：

```python
# 模块加载时决定功能开关
if KALMAN_ENABLED:
    _kf = _Kalman(alpha=0.3)
    kf_update = _kf.update      # 真实EMA滤波
else:
    kf_update = lambda mx, my: (float(mx), float(my))  # 透传

# 主循环中直接调用 — 无论开关, 语法一致
fdx, fdy = kf_update(dx, dy)
```

### 15.3 降采样检测

在低分辨率上运行检测可减少像素量（320×240 仅 640×480 的 1/4），但 `cv2.resize` 本身有开销：

- **默认关闭**，与初版帧率一致
- 开启后需实测对比帧率：
  - 若 resize 耗时 > 检测节省的时间 → 关闭
  - 若 resize 耗时 < 检测节省的时间 → 开启
- 2倍降采样（640→320）效果最好，非整数比例 resize 更慢

### 15.4 GC控制

```python
GC_EVERY = 30  # 每30帧执行一次gc.collect()
```
- 太频繁 → 每帧都GC → 帧率降低
- 太稀疏 → 内存堆积 → 周期性卡顿（GC被迫触发时暂停更长）
- 推荐值：每1-2秒一次（30-60帧）

### 15.5 打印限流

```python
PRINT_EVERY = 60  # 每60帧才print一次
```
串口 `print()` 在 MicroPython 中非常慢，高频打印会严重拖慢帧率。

---

## 十六、云台串口通信协议 🔌

### 16.1 数据帧格式

K230 通过 UART 向 STM32 云台发送偏差数据：

```
dx,dy,dist,status\n
```

| 字段 | 类型 | 含义 | 示例 |
|------|------|------|------|
| `dx` | int | X方向像素偏差（正值=目标在画面右侧） | `-25` |
| `dy` | int | Y方向像素偏差（正值=目标在画面下方） | `18` |
| `dist` | float | 目标到画面中心的欧氏距离（像素） | `31` |
| `status` | int | 状态码: 0=追踪 1=对准 404=丢失 | `0` |

### 16.2 帧示例

```
-25,18,31,0\n     追踪中: 目标偏左25px, 偏下18px, 距离中心31px
-2,1,2,1\n        已对准: 偏差在容差范围内, status=1
404,404,0,0\n     目标丢失: 云台应执行搜索策略
```

### 16.3 STM32端解析（C语言）

```c
int dx, dy, dist, status;
if (sscanf(uart_buf, "%d,%d,%d,%d", &dx, &dy, &dist, &status) == 4) {
    if (status == 404) {
        // 目标丢失 → 执行搜索策略
        gimbal_search();
    } else if (status == 1) {
        // 已对准 → 触发激光或保持
        laser_trigger();
    } else {
        // 追踪中 → PID闭环控制
        gimbal_pid_update(dx, dy);
    }
}
```

### 16.4 距离计算

```python
dist = math.sqrt(dx*dx + dy*dy)
```

- 单位：像素
- 用于云台判断目标远近、调整追踪速度
- 结合已知靶标尺寸可估算物理距离（小孔成像模型）

### 16.5 配置项

```json
{
  "uart": {
    "enabled": true,
    "port": 2,
    "baud": 115200,
    "tx_pin": 5,
    "rx_pin": 6,
    "format": "csv"
  }
}
```

- `enabled`: 接云台时改为 `true`，否则保持 `false`（防止TX阻塞）
- `port`: K230 UART端口号（本项目用 UART2）
- `baud`: 必须与STM32端一致（默认 115200，8N1）
- `tx_pin` / `rx_pin`: K230 GPIO 引脚号，经 FPIOA 映射为 UART 功能
- `format`: 输出格式，`csv` = 文本 `"dx,dy,dist,status\n"`（UTF-8 编码）

### 16.6 物理引脚映射

> 引脚定义详见 §1.2 庐山派 40-Pin 引脚表

| config.json 字段 | GPIO 号 | 物理 Pin | 板上功能 | 方向 |
|------------------|---------|----------|----------|------|
| `tx_pin` | 5 | **Pin 11** | UART2_TX | 输出（K230→外部） |
| `rx_pin` | 6 | **Pin 13** | UART2_RX | 输入（外部→K230） |

### 16.7 接线图

**方案A：K230 → USB-TTL → 电脑（测试用）**

```
   K230 庐山派                    USB-TTL 模块 (CH340/CP2102)
   ┌──────────────┐               ┌──────────────┐
   │  Pin11 GPIO5 │── UART2_TX ──→│ RXD          │
   │  (UART2_TX)  │               │              │
   │  Pin13 GPIO6 │── UART2_RX ←──│ TXD (可不接) │
   │  (UART2_RX)  │               │              │
   │  Pin9  GND   │── GND ────────│ GND          │
   └──────────────┘               └──────┬───────┘
                                         │ USB
                                   ┌─────┴─────┐
                                   │   电脑    │
                                   └───────────┘

   ★ 只需3根线: TX→RX, GND↔GND (单向测试)
   ★ 电平: 3.3V TTL, 直连无需电平转换
   ★ 串口参数: 115200, 8N1, UTF-8 文本
```

**方案B：K230 → STM32F407（部署用）**

```
   K230 庐山派                    STM32F407
   ┌──────────────┐               ┌──────────────┐
   │  Pin11 GPIO5 │── UART2_TX ──→│ PC11 UART4_RX│
   │  (UART2_TX)  │               │              │
   │  Pin13 GPIO6 │── UART2_RX ←──│ PC10 UART4_TX│
   │  (UART2_RX)  │               │              │
   │  Pin9  GND   │── GND ────────│ GND          │
   └──────────────┘               └──────────────┘

   ★ TX↔RX 交叉连接, 共地
   ★ 双方均为 3.3V TTL, 直连
   ★ 波特率必须一致 (115200)
```

---

## 十七、参考资料

- [K230 官方文档](https://developer.canaan-creative.com/k230_rtos/)
- [K230 NNCase开发指南](https://developer.canaan-creative.com/k230_rtos/zh/v0.1/app_develop_guide/ai/nncase.html)
- [K230 CanMV用户指南](https://developer.canaan-creative.com/k230_canmv/main/zh/userguide/)
- [NNCase API手册](https://developer.canaan-creative.com/k230_rtos/zh/main/api_reference/nncase/)
- [YOLOv8文档](https://docs.ultralytics.com/)
- [Colosseum仿真器](https://github.com/dronesimulator/Colosseum) — AirSim后继
- [Cosys-AirSim](https://github.com/FlywardAero/Cosys-AirSim-102024) — UE5版AirSim
- [PX4 SITL文档](https://docs.px4.io/main/en/simulation/)
- [MAVLink协议](https://mavlink.io/)
- [Mission Planner](https://missionplanner.org/)
