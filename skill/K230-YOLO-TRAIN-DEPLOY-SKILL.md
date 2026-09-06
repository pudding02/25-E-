---
name: "k230-yolo-train-deploy"
description: "K230 YOLO模型训练与转换全流程：数据集准备与标注规范、YOLOv8训练调参（输入尺寸16/32对齐约束）、ONNX导出、NNCase转KModel完整脚本（检测框乱飘配置速查）、训练效果排查与部署检查清单。当需要训练YOLO模型、转换KModel或排查训练/量化问题时调用。"
---

# K230 YOLO 训练与模型转换 Skill

> 本 Skill 合并自原 `02_训练配置指南.md` 与 `K230-DRONE-AI-SKILL.md` 的阶段1-3（数据集/训练/转换），是 K230 视觉项目的模型生产流水线。

## 0. 适用场景与边界

**适用：**
- 数据集准备、YOLO 标注规范、采集建议
- YOLOv8 训练、调参、效果评估与排查
- ONNX 导出 + NNCase 转 KModel（含量化校准、检测框乱飘修复）

**不适用：**
- K230 实机部署代码、FPV 仿真、MAVLink → 见 [k230-drone-ai skill](file:///c:/Users/12553/Desktop/视觉/复刻/skill/K230-DRONE-AI-SKILL.md)（其「阶段1：K230部署」章节）
- K230 直控舵机云台 → 见 [fashionstar-servo-k230 skill](file:///c:/Users/12553/Desktop/视觉/复刻/.trae/skills/fashionstar-servo-k230/SKILL.md)

**整体流程**：
```
环境准备 → 数据集准备 → YOLOv8训练 → ONNX导出 → KModel转换 → (K230部署, 见 drone skill)
```

---

## 一、环境准备

### 1.1 Python 环境（训练用）

```bash
# 推荐使用Python 3.10（训练使用3.10+均可，但K230模型转换必须3.10）
py -3.10 -m venv yolo_env
# Windows
yolo_env\Scripts\activate
# Linux/Mac
source yolo_env/bin/activate

pip install ultralytics opencv-python numpy pillow
# 如果有GPU（强烈推荐）
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### 1.2 模型转换环境（K230 专用，必须 Python 3.10）

```bash
py -3.10 -m venv k230_env
py -3.10 -m pip install numpy pillow
py -3.10 -m pip install nncase-2.8.3-cp310-cp310-win_amd64.whl
py -3.10 -m pip install nncase_kpu-2.8.3-py2.py3-none-win_amd64.whl
```

---

## 二、数据集准备与标注

### 2.1 目录结构

```
project/
├── dataset/
│   ├── images/
│   │   ├── train/          # 训练图片（建议≥500张）
│   │   ├── val/            # 验证图片（建议≥100张，至少≥50张）
│   │   └── test/           # 测试图片（可选）
│   ├── labels/
│   │   ├── train/          # YOLO格式标注文件（.txt）
│   │   └── val/
│   └── data.yaml           # 数据集配置文件
├── quant_dataset/          # 量化校准图片（100-200张，有代表性）
├── train.py                # 训练脚本（见第四节）
├── convert.py              # ONNX→KModel转换脚本（见第六节）
└── runs/                   # 训练输出（自动生成）
```

### 2.2 data.yaml 配置

```yaml
# 数据集根目录（相对于此yaml文件的路径）
path: ./dataset

# 训练集图片目录（相对path）
train: images/train

# 验证集图片目录（相对path）
val: images/val

# 类别数量
nc: 1

# 类别名称列表 — 顺序决定class_id（class_id从0开始）！
# 训练和部署时必须保持完全一致
names:
  - target
```

> `nc`/`names` 按实际项目修改（如无人机检测：`nc: 3` + `drone/car/person`）。

### 2.3 YOLO 标注格式

每张图片对应一个同名`.txt`文件，每行一个目标：

```
<class_id> <x_center> <y_center> <width> <height>
```

- 所有坐标均为**归一化值（0-1）**，相对于图片宽高
- `x_center = (box_left + box_right) / 2 / image_width`
- `y_center = (box_top + box_bottom) / 2 / image_height`
- `width = (box_right - box_left) / image_width`
- `height = (box_bottom - box_top) / image_height`

**示例**（图片640×480，矩形框左上角(120,100) 右下角(520,380)）：
```
0 0.5 0.5 0.625 0.583
```

### 2.4 标注工具推荐

| 工具 | 平台 | 优点 | 下载 |
|------|------|------|------|
| LabelImg | Win/Mac/Linux | 轻量，YOLO格式原生支持 | `pip install labelImg` |
| Roboflow | Web | 在线协作，自动增强，格式导出 | roboflow.com |
| CVAT | Web/Docker | 专业级，支持团队协作 | cvat.ai |

### 2.5 标注规范

1. **标注目标**：靶标/目标的**外边框**，紧密贴合边缘角点
2. **遮挡处理**：部分遮挡时标注**可见部分的外边框**（不要猜测被遮挡部分）
3. **边界处理**：目标部分超出画面时，标注**画面内的可见部分**
4. **质量检查**：每张标注完后放大检查，确保框紧贴边缘、无偏移
5. **交叉验证**：多人标注时交换检查，发现偏差>5px的重新标注

### 2.6 数据采集建议

**采集场景覆盖矩阵**（每格≥20张）：

| | 正面(0°) | 左偏15° | 右偏15° | 左偏30° | 右偏30° | 左偏45° | 右偏45° |
|------|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| 1m | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| 2m | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| 3m | ✓ | ✓ | ✓ | - | - | - | - |
| 5m | ✓ | ✓ | - | - | - | - | - |

**光照变化**（每个距离×角度组合至少1张）：正常室内灯光 / 关灯暗光 / 侧面打光 / 目标反光。

**负样本**：训练集不需要，但建议采集无目标场景图用于测试误检率。

### 2.7 量化校准数据集（供第六节转换使用）

独立目录 `quant_dataset/`，放入 **100-200 张代表性图片**：
- 覆盖所有距离、角度、光照条件（与训练集同分布）
- 图片分辨率与训练时的 imgsz 一致（如 320×320）
- 文件名按场景命名便于核对：`scene_sunny_001.jpg`、`scene_indoor_003.jpg`...

---

## 三、输入尺寸选择（K230 硬性约束）

| 尺寸 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| **320×320** ★推荐 | 速度与精度最佳平衡；满足KPU 16对齐 + YOLO 32 stride | 极小目标检测稍弱 | 通用场景 |
| 224×224 | 速度最快，内存最低 | 精度下降明显 | 仅大目标/资源极度受限 |
| 416×416 | 精度较好 | 速度下降 | 中等目标 |
| 640×640 | 小目标检测好 | 速度最慢，可能内存溢出 | 远距离极小目标 |

> **硬性约束**：输入尺寸必须整除16（KPU要求）且建议整除32（YOLOv8 stride=32）。**320 = 32×10，同时满足两个约束，为K230最佳选择。** 训练、导出、转换、部署四处必须使用同一尺寸。

---

## 四、模型训练

### 4.1 训练脚本 (`train.py`)

```python
"""
YOLOv8目标检测训练脚本
使用方法: python train.py
修改下方 CONFIG 区域即可适配不同场景
"""
from ultralytics import YOLO
import os

# ==================== 配置区（根据实际情况修改） ====================
CONFIG = {
    # 数据集配置文件路径
    'data': 'dataset/data.yaml',

    # 预训练模型 — yolov8n最轻量适合K230
    # 可选: yolov8n.pt / yolov8s.pt / yolov8m.pt / yolov8l.pt
    'model': 'yolov8n.pt',

    # ★ 输入尺寸 — 必须与K230部署尺寸一致！（见第三节）
    'imgsz': 320,

    # ★ 训练轮数 — 一般100-200轮
    # 数据少→多训练（200+）；数据多→可少训（100）
    'epochs': 200,

    # ★ 批次大小 — 根据GPU显存调整
    # 4GB显存: batch=8；8GB: batch=16；16GB+: batch=32
    'batch': 16,

    # 设备: 0=GPU, 'cpu'=CPU
    'device': 0,

    # 数据加载进程数
    'workers': 4,

    # 预训练权重 — 强烈建议True
    'pretrained': True,

    # ★ 优化器: SGD(推荐) / Adam / AdamW
    # SGD: 收敛慢但泛化好；Adam: 收敛快
    'optimizer': 'SGD',

    # ★ 初始学习率: SGD用0.01；Adam用0.001
    'lr0': 0.01,

    # 余弦学习率衰减 — 推荐True
    'cos_lr': True,

    # ★ 早停耐心值 — 验证集loss连续不降则提前停止
    'patience': 50,

    # 缓存图片到内存加速训练（小数据集推荐True）
    'cache': True,

    # ★ Mosaic增强 — 最后N轮关闭（提高最终精度）
    'close_mosaic': 10,

    # 输出目录
    'project': './runs/detect',
    'name': 'e_target_detect',

    # 生成可视化图表
    'plots': True,

    # 保存每个epoch的检查点（会占用大量磁盘）
    'save': True,
}
# ==================================================================


def main():
    print("=" * 60)
    print("YOLOv8 训练开始")
    print(f"模型: {CONFIG['model']}")
    print(f"数据: {CONFIG['data']}")
    print(f"尺寸: {CONFIG['imgsz']}x{CONFIG['imgsz']}")
    print(f"轮数: {CONFIG['epochs']}, 批次: {CONFIG['batch']}")
    print("=" * 60)

    if not os.path.exists(CONFIG['data']):
        print(f"错误: 找不到数据集配置文件 {CONFIG['data']}")
        return

    model = YOLO(CONFIG['model'])

    results = model.train(
        data=CONFIG['data'],
        epochs=CONFIG['epochs'],
        imgsz=CONFIG['imgsz'],
        batch=CONFIG['batch'],
        device=CONFIG['device'],
        workers=CONFIG['workers'],
        pretrained=CONFIG['pretrained'],
        optimizer=CONFIG['optimizer'],
        lr0=CONFIG['lr0'],
        cos_lr=CONFIG['cos_lr'],
        patience=CONFIG['patience'],
        cache=CONFIG['cache'],
        close_mosaic=CONFIG['close_mosaic'],
        project=CONFIG['project'],
        name=CONFIG['name'],
        plots=CONFIG['plots'],
        save=CONFIG['save'],
    )

    # 训练完直接导出ONNX（用于后续KModel转换）
    print("\n导出ONNX...")
    model.export(
        format='onnx',
        opset=11,           # K230要求opset 11
        simplify=True,      # 简化模型图
        imgsz=CONFIG['imgsz'],
    )

    print(f"\n训练完成！")
    print(f"模型输出: {CONFIG['project']}/{CONFIG['name']}/weights/best.pt")
    print(f"ONNX输出: {CONFIG['project']}/{CONFIG['name']}/weights/best.onnx")


if __name__ == '__main__':
    main()
```

### 4.2 训练效果评估

训练完成后在 `runs/detect/<name>/` 下查看：

| 文件 | 内容 |
|------|------|
| `results.csv` | 每轮训练指标数据 |
| `results.png` | 训练曲线（loss/精度/召回/mAP） |
| `confusion_matrix.png` | 混淆矩阵 |
| `val_batch*.jpg` | 验证集检测效果可视化 |
| `weights/best.pt` | 验证集最佳权重 |
| `weights/best.onnx` | 导出ONNX模型 |

**关键指标解读**：

| 指标 | results.csv 列名 | 目标值 | 说明 |
|------|-----------|--------|------|
| 精确率 | `metrics/precision(B)` | >0.8 | 检测到的目标中有多少是对的 |
| 召回率 | `metrics/recall(B)` | >0.8 | 真实目标中有多少被检测到 |
| mAP@0.5 | `metrics/mAP50(B)` | >0.9 | IoU=0.5时的平均精度 |
| mAP@0.5:0.95 | `metrics/mAP50-95(B)` | >0.6 | 多IoU阈值下的平均精度（更严格） |

### 4.3 数据增强策略

YOLOv8 内置增强（自动应用）：Mosaic、MixUp、HSV色彩抖动、水平翻转、缩放平移。

针对固定靶标/无人机场景的**自定义增强参数**（在 train 参数中追加）：

```python
model.train(
    ...
    hsv_h=0.015,      # 色调变化范围（小——靶标颜色固定）
    hsv_s=0.3,        # 饱和度变化（中）
    hsv_v=0.3,        # ★ 亮度变化（大——模拟不同光照）
    degrees=15.0,     # ★ 旋转角度（大——视角变化）
    translate=0.1,    # 平移比例
    scale=0.3,        # ★ 缩放比例（大——模拟不同距离）
    shear=2.0,        # 剪切角度
    flipud=0.0,       # 垂直翻转概率（0=不翻转——靶标不会倒挂）
    fliplr=0.5,       # 水平翻转概率
)
```

### 4.4 难例挖掘策略

如果训练后的模型在特定场景下表现差：

```
1. 收集模型"犯错"的图片（漏检/误检/框不准）
2. 将这些图片加入训练集
3. 重新训练（可降低lr0=0.001微调）
4. 重复直到满意
```

这比盲目增加训练数据量更高效。

---

## 五、ONNX 导出（单独执行时）

若训练脚本未自动导出，运行：

```python
from ultralytics import YOLO

model = YOLO('runs/detect/e_target_detect/weights/best.pt')

model.export(
    format='onnx',
    opset=11,          # K230支持的最稳定版本
    simplify=True,     # 简化计算图
    imgsz=320,         # 与训练时一致
)
```

输出文件：`runs/detect/e_target_detect/weights/best.onnx`

---

## 六、ONNX 转 KModel

### 6.1 环境变量

```python
import os
# Windows — 替换<用户名>为实际用户名
os.environ['NNCASE_PLUGIN_PATH'] = r"C:\Users\<用户名>\AppData\Local\Python\PythonCore-3.10-64\Lib\site-packages\nncase"

# 查实际路径：
import nncase
print(nncase.__file__)
```

### 6.2 完整转换脚本 (`convert.py`)

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

### 6.3 关键配置速查表（★ = 检测框乱飘的常见元凶）

| 配置项 | 正确值 | 错误值（会导致问题） |
|--------|--------|---------------------|
| `preprocess` | **True** | False → 框乱飘 |
| `input_type` | **"uint8"** | "float32" → 框乱飘 |
| `calibrate_method` | **"NoClip"** ★ | 官方默认 "Kld" 也可用，但NoClip经社区验证更稳定 |
| 校准数据格式 | **uint8 (0-255)** | 归一化到0-1 → 框乱飘 |
| Python版本 | **3.10** | 3.11+ → NNCase不兼容 |
| NNCase版本 | **2.8.3** | 其他版本 → 可能KPU运行失败 |
| opset | **11** | 高版本opset不被K230支持 |
| `input_shape` | [1,3,320,320] | 与训练尺寸不一致 → 尺寸不匹配 |

> **关于 `calibrate_method`**：NNCase官方提供 `"NoClip"`（直接min/max范围）和 `"Kld"`（KL散度，官方默认）两种。社区大量实践验证 `"NoClip"` 能稳定解决K230上检测框乱飘问题，因此本Skill推荐 `"NoClip"`。

---

## 七、训练效果不佳的排查

| 问题 | 可能原因 | 解决 |
|------|---------|------|
| loss不收敛 | 学习率太高 | 降低lr0→0.001 |
| | 标注错误多 | 抽查10%标注，修正错误 |
| | 数据太少 | 增加训练数据 |
| 训练集精度高/验证集差 | 过拟合 | 增加数据增强强度；增加dropout |
| | 数据集太少 | 增加数据量 |
| | 训练/验证集分布不同 | 重新随机划分数据集 |
| 小目标漏检 | 目标在画面中太小 | 提高imgsz: 320→416→640 |
| 特定角度漏检 | 该角度训练数据不足 | 补充该角度的数据 |
| 特定光照漏检 | 光照数据不足 | 补充光照变化数据 + 加强HSV增强 |
| 假阳性太多 | 负样本不够 | 添加无目标背景图到训练集 |

---

## 八、完整训练→部署检查清单

- [ ] data.yaml中`nc`和`names`是否正确
- [ ] 训练集和验证集中所有图片都有对应`.txt`标注文件
- [ ] 标注坐标都是0-1的归一化值
- [ ] class_id从0开始，与names顺序对应
- [ ] imgsz设置为320（K230推荐尺寸，16/32对齐）
- [ ] ONNX导出时opset=11
- [ ] 量化数据集覆盖所有场景变化
- [ ] 转换时preprocess=True, input_type="uint8", calibrate_method="NoClip"
- [ ] 部署时LABELS列表顺序与data.yaml中names完全一致
- [ ] K230部署代码中MODEL_INPUT_SIZE与训练imgsz一致
- [ ] 置信度阈值从0.3开始，根据现场情况微调

---

## 参考资料

- [K230 NNCase开发指南](https://developer.canaan-creative.com/k230_rtos/zh/v0.1/app_develop_guide/ai/nncase.html)
- [NNCase API手册](https://developer.canaan-creative.com/k230_rtos/zh/main/api_reference/nncase/)
- K230 部署与后处理 → [k230-drone-ai skill](file:///c:/Users/12553/Desktop/视觉/复刻/skill/K230-DRONE-AI-SKILL.md)
