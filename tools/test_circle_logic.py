"""V7 视觉纯函数主机侧测试 —— 提取 main.py 的纯函数并断言其行为。

背景: V7 功能扩展把「画圈几何 / 上电搜索 / 模式引脚去抖」做成模块级
纯函数, 便于主机侧(不接 K230 硬件)测试:
  - point_in_quad      凸四边形包含判定(红环∈矩形 关联用)
  - classify_blob      红色块分流: 大空心近方=红环 / 其余=None
  - circle_target_point   画圈目标点(环心+半径+θ -> 像素坐标)
  - circle_theta_step     画圈 θ 推进(误差门控 + 方向 + 360° 环绕)
  - search_step           上电搜索状态机(未捕获->spin, 已捕获->idle, 超时->timeout)
  - mode_pin_step         模式引脚去抖(连续 N 次一致才切换)

原理: 从 k230/main.py 中用 AST 提取上述函数源码, 连同它们引用的模块级阈值常量
(_get / 配置段变量 / RING_MIN_DIAM 等) 一并从 main.py 源码中提取, 在 stub 环境里
执行(math + stub time + CFG={}), 逐条断言行为与边界。
      阈值常量一律取自 main.py 本身(测试不注入任何值), 因此 main.py 若漏定义常量
      (classify_blob 生产会 NameError), 这里会直接报错而非虚假绿灯。

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
    "point_in_quad", "classify_blob",
    "circle_target_point", "circle_theta_step", "search_step", "mode_pin_step",
    "ring_memory_step",
)

# classify_blob 引用的模块级阈值常量 —— 必须由 main.py 定义; 本测试**不注入**其取值,
# 而是从 main.py 源码里提取执行, 缺任何一个都会在此暴露(classify_blob 生产会 NameError)。
_CONST_NAMES = (
    "RING_MIN_DIAM", "RING_MAX_DIAM", "RING_MAX_ASPECT",
    "HOLLOW_MAX_HITS",
)
# 上述常量取值依赖的模块级名字(也一律从 main.py 提取, 测试不自造值):
#   _get = 配置取值 helper;  rt = [red_target] 配置段变量
_PREAMBLE_FUNCS = ("_get",)
_PREAMBLE_VARS = ("rt",)


def _extract_func(src, name):
    """返回顶层函数源码; 找不到时抛 AssertionError"""
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到函数 %s" % name)


def _extract_assign(src, name):
    """返回顶层 `name = ...` 赋值源码; 找不到时抛 AssertionError"""
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id == name:
                    return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到模块级赋值 %s = ..." % name)


class _FakeTime(types.ModuleType):
    """stub MicroPython 的 time 接口"""

    def __init__(self):
        super().__init__("time")

    def ticks_diff(self, a, b):
        return a - b


def load_funcs():
    """加载并返回 main.py 里的 6 个纯函数。

    阈值常量与配置段变量一律从 main.py 源码中提取执行 —— 测试不硬注入任何取值,
    因此 main.py 若漏定义 classify_blob 引用的常量(生产会 NameError), 这里会直接
    报错暴露, 而不是靠注入掩盖。
    """
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    env = {
        "math": math,
        "time": _FakeTime(),
        "CFG": {},          # 主机侧无 config.json: 常量取 main.py 内默认值
    }
    # 1) 常量取值依赖的 helper / 配置段变量(同样只从 main.py 取, 缺席即留给第 2 步报错)
    for fname in _PREAMBLE_FUNCS:
        exec(_extract_func(src, fname), env)
    for vname in _PREAMBLE_VARS:
        try:
            exec(_extract_assign(src, vname), env)
        except AssertionError:
            pass
    # 2) classify_blob 引用的阈值常量(缺任一个即报错)
    missing_const = []
    for cname in _CONST_NAMES:
        try:
            exec(_extract_assign(src, cname), env)
        except AssertionError:
            missing_const.append(cname)
    if missing_const:
        raise AssertionError(
            "k230/main.py 缺少模块级常量: %s (classify_blob 引用但未定义 → 生产会 NameError)"
            % ", ".join(missing_const))
    # 3) 纯函数
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
    """懒加载 6 个纯函数(全部加载成功才缓存)"""
    global _FNS
    if _FNS is None:
        _FNS = load_funcs()
    return _FNS[name]


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
    """红色块分流: 大空心近方->ring; 小尺寸/实心/畸形->None(红点识别已删除, 小团不再判 dot)"""

    def test_big_hollow_ring(self):
        self.assertEqual(F("classify_blob")(60, 60, 0), "ring")

    def test_hollow_boundary_rejected(self):
        self.assertEqual(F("classify_blob")(60, 60, 3), None)

    def test_small_solid_rejected(self):
        self.assertEqual(F("classify_blob")(30, 30, 8), None)

    def test_big_solid_rejected(self):
        self.assertEqual(F("classify_blob")(60, 60, 8), None)

    def test_tiny_rejected(self):
        self.assertEqual(F("classify_blob")(3, 3, 0), None)

    def test_elongated_rejected(self):
        self.assertEqual(F("classify_blob")(100, 50, 0), None)

    def test_over_size_limit_rejected(self):
        self.assertEqual(F("classify_blob")(1000, 1000, 1), None)


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


class TestRingMemory(unittest.TestCase):
    """ring_memory_step: 画圈环记忆兜底(参考代码"几何优先"思想的混合落地)。

    规则: 环可见 → 直通并刷新记忆; 环丢失但 fallback_ms 窗口内且框中心可用
    → 用 (框中心, 记忆半径) 继续画圈; 过期/无历史/关断 → None(调用方退原行为)。
    """

    def test_ring_visible_refreshes_memory(self):
        ring = (100, 80, 30)
        use, mem = F("ring_memory_step")([0, 0, 0, 0], 5000, ring, (90, 70), 800)
        self.assertEqual(use, ring)
        self.assertEqual(mem, [100, 80, 30, 5000])

    def test_fallback_within_window(self):
        mem = [100, 80, 30, 5000]
        use, new_mem = F("ring_memory_step")(mem, 5300, None, (90, 70), 800)
        self.assertEqual(use, (90, 70, 30))       # 框中心 + 记忆半径
        self.assertEqual(new_mem, mem)            # 兜底不改写记忆(时间戳保持)
        self.assertEqual(new_mem[3], 5000)

    def test_fallback_at_window_boundary(self):
        mem = [100, 80, 30, 5000]
        use, _ = F("ring_memory_step")(mem, 5800, None, (90, 70), 800)
        self.assertEqual(use, (90, 70, 30))       # 恰在窗口边界(<=)仍有效

    def test_fallback_expired(self):
        mem = [100, 80, 30, 5000]
        use, new_mem = F("ring_memory_step")(mem, 5801, None, (90, 70), 800)
        self.assertIsNone(use)
        self.assertEqual(new_mem, mem)

    def test_fallback_disabled_returns_none(self):
        mem = [100, 80, 30, 5000]
        use, _ = F("ring_memory_step")(mem, 5100, None, (90, 70), 0)
        self.assertIsNone(use)                    # fallback_ms=0 → 原行为(不兜底)

    def test_no_history_not_fallback(self):
        use, _ = F("ring_memory_step")([0, 0, 0, 0], 5000, None, (90, 70), 800)
        self.assertIsNone(use)                    # 从未见过环 → 无从兜底

    def test_no_box_not_fallback(self):
        mem = [100, 80, 30, 5000]
        use, _ = F("ring_memory_step")(mem, 5100, None, None, 800)
        self.assertIsNone(use)                    # 框也丢了 → 退原行为


class TestModuleThresholdConstants(unittest.TestCase):
    """回归守门: classify_blob 引用的阈值常量必须在 main.py 中真实定义。

    原缺陷: classify_blob 引用未定义的模块级常量 → 生产调用即 NameError;
    旧测试把常量取值硬注入 exec 环境, 结构性掩盖了该缺陷。本用例直接断言
    main.py 的模块级赋值里确实出现这些名字(与上面 TestClassifyBlob 的行为断言互补)。
    """

    def test_constants_defined_in_main(self):
        with open(_MAIN_PY, encoding="utf-8") as f:
            tree = ast.parse(f.read())
        defined = set()
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for tgt in node.targets:
                    if isinstance(tgt, ast.Name):
                        defined.add(tgt.id)
        missing = [n for n in _CONST_NAMES if n not in defined]
        self.assertEqual(
            missing, [],
            "k230/main.py 未定义 classify_blob 引用的模块级常量: %s" % missing)


if __name__ == "__main__":
    unittest.main(verbosity=2)
