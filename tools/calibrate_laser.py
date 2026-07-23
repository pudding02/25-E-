"""
激光/光轴偏移标定工具
====================
用于标定激光安装点与摄像头光轴之间的像素偏移。

使用方法:
  1. 将靶标放置在画面正中心（手动对准十字线）
  2. 打开激光，观察激光点在画面中的位置
  3. 测量激光点相对于画面中心的像素偏移:
     - 激光在中心右侧 → offset_x 为正值
     - 激光在中心下方 → offset_y 为正值
  4. 将测量的 (offset_x, offset_y) 填入 k230/main.py 的 LASER_OFFSET_X/Y

或者运行此脚本进行自动标定:
  python calibrate_laser.py --image calibration.jpg
"""

import argparse
import math


def manual_calibrate():
    """手动标定引导"""
    print("=" * 50)
    print("激光光轴偏移标定")
    print("=" * 50)
    print()
    print("步骤:")
    print("  1. 在 K230 画面中手动将靶标对准画面中心十字线")
    print("  2. 打开激光")
    print("  3. 观察激光点在画面中的位置")
    print()
    print("测量:")
    print("  激光点在十字线右侧 → offset_x 为正")
    print("  激光点在十字线左侧 → offset_x 为负")
    print("  激光点在十字线下方 → offset_y 为正")
    print("  激光点在十字线上方 → offset_y 为负")
    print()

    try:
        ox = int(input("offset_x (像素): "))
        oy = int(input("offset_y (像素): "))
    except (ValueError, EOFError):
        print("输入无效，使用默认值 0,0")
        ox, oy = 0, 0

    print()
    print("=" * 50)
    print("标定结果:")
    print("  LASER_OFFSET_X = %d" % ox)
    print("  LASER_OFFSET_Y = %d" % oy)
    print()
    print("请将以上值填入 k230/main.py 的配置区:")
    print("  LASER_OFFSET_X = %d" % ox)
    print("  LASER_OFFSET_Y = %d" % oy)
    print("=" * 50)

    return ox, oy


def image_calibrate(image_path):
    """基于标定图片的半自动标定"""
    print("图片标定暂不支持（需要 OpenCV）")
    print("请使用手动标定模式: python calibrate_laser.py --manual")


def main():
    parser = argparse.ArgumentParser(description="激光/光轴偏移标定")
    parser.add_argument("--image", "-i", type=str, help="标定图片路径")
    parser.add_argument("--manual", "-m", action="store_true",
                        default=True, help="手动标定模式")
    args = parser.parse_args()

    if args.image:
        image_calibrate(args.image)
    else:
        manual_calibrate()


if __name__ == "__main__":
    main()
