"""V7 视觉纯函数主机侧测试 —— 提取 main.py 的纯函数并断言其行为。

背景: V7 功能扩展把「目标选择 / 画圈几何 / 上电搜索 / 模式引脚去抖」做成模块级
纯函数, 便于主机侧(不接 K230 硬件)测试:
  - select_target      三档优先度: 矩形中有红点 > 矩形 > 红点 > 无
  - point_in_quad      凸四边形包含判定(红点∈矩形 关联用)
  - classify_blob      红色块分流: 小实心=红点 / 大空心近方=红环 / 其余=None
  - circle_target_point   画圈目标点(环心+半径+θ -> 像素坐标)
  - circle_theta_step     画圈 θ 推进(误差门控 + 方向 + 360° 环绕)
  - search_step           上电搜索状态机(未捕获->spin, 已捕获->idle, 超时->timeout)
  - mode_pin_step         模式引脚去抖(连续 N 次一致才切换)

原理: 从 k230/main.py 中用 AST 提取上述函数源码, 在 stub 环境里执行(math +
stub time + 阈值常量), 逐条断言行为与边界。

运行: python tools/test_circle_logic.py
说明: 纯主机侧运行, 不需要 K230 硬件; 不依赖任何第三方库(stdlib only)。
      覆盖边界: 只覆盖上述纯函数; 检测链(cv2)/画圈伺服/搜索接线等集成面
      不在主机测试范围内, 由实机验证覆盖(见计划文件「验证清单」)。
"""
import ast
import math
import os
import types
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))

_FUNC_NAMES = (
    "select_target", "point_in_quad", "classify_blob",
    "circle_target_point", "circle_theta_step", "search_step", "mode_pin_step",
)


def _extract_func(src, name):
    """返回顶层函数源码; 找不到时抛 AssertionError"""
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到函数 %s" % name)


class _FakeTime(types.ModuleType):
    """stub MicroPython 的 time 接口"""

    def __init__(self):
        super().__init__("time")

    def ticks_diff(self, a, b):
        return a - b


def load_funcs():
    """加载并返回 main.py 里的 7 个纯函数"""
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    env = {
        "math": math,
        "time": _FakeTime(),
        # classify_blob 引用的阈值常量(与 config.json [red_target] 默认一致)
        "RING_MIN_DIAM": 40,
        "RING_MAX_DIAM": 900,
        "RING_MAX_ASPECT": 1.6,
        "HOLLOW_MAX_HITS": 2,
        "DOT_MIN_AREA": 9,
        "DOT_MAX_AREA": 2500,
    }
    fns = {}
    missing = []
    for name in _FUNC_NAMES:
        try:
            exec(_extract_func(src, name), env)
        except AssertionError:
            missing.append(name)
            continue
        fns[name] = env[name]
    if missing:
        raise AssertionError("k230/main.py 缺少函数: %s (未实现或改名)" % ", ".join(missing))
    return fns


_FNS = None


def F(name):
    """懒加载 7 个纯函数(全部加载成功才缓存)"""
    global _FNS
    if _FNS is None:
        _FNS = load_funcs()
    return _FNS[name]


class TestSelectTarget(unittest.TestCase):
    """三档优先度: 矩形中有红点 > 矩形 > 红点 > 无"""

    def test_red_in_rect_highest(self):
        self.assertEqual(F("select_target")(True, 100, 100, True, 120, 130, True),
                         ("red+rect", 120, 130))

    def test_dot_outside_rect_falls_to_rect(self):
        self.assertEqual(F("select_target")(True, 100, 100, True, 500, 500, False),
                         ("rect", 100, 100))

    def test_rect_only(self):
        self.assertEqual(F("select_target")(True, 100, 100, False, 0, 0, False),
                         ("rect", 100, 100))

    def test_dot_only(self):
        self.assertEqual(F("select_target")(False, 0, 0, True, 300, 220, False),
                         ("red", 300, 220))

    def test_none(self):
        self.assertEqual(F("select_target")(False, 0, 0, False, 0, 0, False),
                         ("none", 0, 0))


class TestPointInQuad(unittest.TestCase):
    """凸四边形包含判定(叉积同号法); 点恰在边上视为在内"""

    QUAD = [(10, 10), (110, 10), (110, 110), (10, 110)]

    def test_inside(self):
        self.assertTrue(F("point_in_quad")(60, 60, self.QUAD))

    def test_outside_four_sides(self):
        for px, py in ((-10, 60), (120, 60), (60, -5), (60, 115)):
            self.assertFalse(F("point_in_quad")(px, py, self.QUAD), (px, py))

    def test_on_edge_is_inside(self):
        self.assertTrue(F("point_in_quad")(10, 60, self.QUAD))

    def test_tilted_diamond(self):
        quad = [(50, 10), (110, 60), (50, 110), (0, 60)]
        self.assertTrue(F("point_in_quad")(50, 60, quad))
        self.assertFalse(F("point_in_quad")(15, 15, quad))


class TestClassifyBlob(unittest.TestCase):
    """红色块分流: 大空心近方->ring; 小面积->dot; 其余->None"""

    def test_big_hollow_ring(self):
        self.assertEqual(F("classify_blob")(60, 60, 3000, 0), "ring")

    def test_small_solid_dot(self):
        self.assertEqual(F("classify_blob")(30, 30, 700, 8), "dot")

    def test_big_solid_rejected(self):
        self.assertEqual(F("classify_blob")(60, 60, 3600, 8), None)

    def test_tiny_rejected(self):
        self.assertEqual(F("classify_blob")(3, 3, 5, 0), None)

    def test_elongated_rejected(self):
        self.assertEqual(F("classify_blob")(100, 50, 4000, 0), None)

    def test_over_size_limit_rejected(self):
        self.assertEqual(F("classify_blob")(1000, 1000, 50000, 1), None)


class TestCircleGeometry(unittest.TestCase):
    """画圈几何: 环上目标点 + θ 推进"""

    def test_target_point_cardinals(self):
        f = F("circle_target_point")
        self.assertEqual(f(100, 100, 50, 0), (150, 100))
        self.assertEqual(f(100, 100, 50, 90), (100, 150))
        self.assertEqual(f(100, 100, 50, 180), (50, 100))
        self.assertEqual(f(100, 100, 50, 270), (100, 50))

    def test_target_point_negative_angle(self):
        self.assertEqual(F("circle_target_point")(100, 100, 50, -90), (100, 50))

    def test_target_point_zero_radius(self):
        self.assertEqual(F("circle_target_point")(100, 100, 0, 123), (100, 100))

    def test_theta_advance(self):
        self.assertAlmostEqual(F("circle_theta_step")(10.0, 60, 1000, 1, 0, 120), 70.0)

    def test_theta_frozen_when_error_large(self):
        self.assertAlmostEqual(F("circle_theta_step")(10.0, 60, 1000, 1, 200, 120), 10.0)

    def test_theta_direction(self):
        self.assertAlmostEqual(F("circle_theta_step")(10.0, 60, 1000, -1, 0, 120), 310.0)

    def test_theta_wrap(self):
        self.assertAlmostEqual(F("circle_theta_step")(350.0, 60, 1000, 1, 0, 120), 50.0)

    def test_theta_zero_dt_frozen(self):
        self.assertAlmostEqual(F("circle_theta_step")(10.0, 60, 0, 1, 0, 120), 10.0)


class TestSearchStep(unittest.TestCase):
    """上电搜索状态机: 未捕获->spin; 已捕获->idle(不再进入); 超时->timeout"""

    def test_disabled_idle(self):
        self.assertEqual(F("search_step")(1000, False, 0, False, False, 0),
                         ("idle", False, 0))

    def test_acquired_never_enters(self):
        self.assertEqual(F("search_step")(1000, True, 500, True, True, 0),
                         ("idle", False, 0))

    def test_start(self):
        self.assertEqual(F("search_step")(5000, False, 0, True, False, 0),
                         ("spin", True, 5000))

    def test_continue_keeps_start(self):
        self.assertEqual(F("search_step")(6000, True, 5000, True, False, 8000),
                         ("spin", True, 5000))

    def test_timeout_at_boundary(self):
        self.assertEqual(F("search_step")(13000, True, 5000, True, False, 8000),
                         ("timeout", False, 0))

    def test_no_timeout_when_unlimited(self):
        self.assertEqual(F("search_step")(99999999, True, 5000, True, False, 0),
                         ("spin", True, 5000))


class TestModePinStep(unittest.TestCase):
    """模式引脚去抖: 连续 need 次一致才切换(有效电平换算在调用方完成)"""

    def test_switch_after_debounce(self):
        active, streak = False, 0
        active, streak = F("mode_pin_step")(active, streak, True, 3)
        self.assertEqual((active, streak), (False, 1))
        active, streak = F("mode_pin_step")(active, streak, True, 3)
        self.assertEqual((active, streak), (False, 2))
        active, streak = F("mode_pin_step")(active, streak, True, 3)
        self.assertEqual((active, streak), (True, 0))

    def test_bounce_resets_streak(self):
        active, streak = F("mode_pin_step")(False, 2, False, 3)
        self.assertEqual((active, streak), (False, 0))
        active, streak = F("mode_pin_step")(active, streak, True, 3)
        self.assertEqual((active, streak), (False, 1))

    def test_stable_keeps_value(self):
        self.assertEqual(F("mode_pin_step")(True, 0, True, 3), (True, 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
