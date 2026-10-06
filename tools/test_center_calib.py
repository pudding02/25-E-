"""V7.7 触屏中心校准 —— 主机侧测试（AST 提取 main.py 纯函数并断言行为）。

覆盖：
  - calib_hit    触摸点 → 按钮命中判定（边界含）
  - calib_step   校准 UI 触摸状态机（隐藏态唤醒边沿 / 新按即发 / 方向键连发 / mode·dot·save 单发）
  - _replace_json_int  行内第一个整数 token 替换（保留键/缩进/尾逗号）
  - calib_save_text    config.json 文本级替换（只动 laser_offset_x/y 两行, 其余逐字节保留）
  - _calib_rects  按钮命中矩形生成（8 键: 方向×4 + mode 追踪/画圈切换 + dot + save + param 调参页入口; 布局与不重叠约束）

原理：从 k230/main.py AST 提取上述函数源码在最小环境执行（纯函数, 无 cfg 依赖）, 逐条断言。
运行：python tools/test_center_calib.py
"""
import ast
import json
import os
import types
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))
_CFG_JSON = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "config.json"))

_FUNC_NAMES = ("calib_hit", "calib_step", "_replace_json_int", "calib_save_text", "_calib_rects")


def _extract_func(src, name):
    """返回顶层函数源码；找不到时抛 AssertionError"""
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到函数 %s" % name)


class _FakeTime(types.ModuleType):
    """stub MicroPython time（ticks_diff 语义 = 差值; calib_step 连发计时用）"""

    def __init__(self):
        super().__init__("time")

    def ticks_diff(self, a, b):
        return a - b


def load_funcs():
    """从 main.py 提取 5 个纯函数（calib_step 依赖 time.ticks_diff → 注入 stub）"""
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    env = {"time": _FakeTime()}
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
    """懒加载 5 个函数（全部加载成功才缓存）"""
    global _FNS
    if _FNS is None:
        _FNS = load_funcs()
    return _FNS[name]


class TestCalibHit(unittest.TestCase):
    """触摸点 → 按钮名；边界含, 按钮间隙不含"""

    RECTS = (("up", 100, 100, 150, 150), ("down", 100, 160, 150, 210))

    def test_center(self):
        self.assertEqual(F("calib_hit")(120, 120, self.RECTS), "up")

    def test_edges_inclusive(self):
        f = F("calib_hit")
        self.assertEqual(f(100, 100, self.RECTS), "up")
        self.assertEqual(f(150, 150, self.RECTS), "up")

    def test_outside(self):
        f = F("calib_hit")
        for x, y in ((99, 120), (151, 120), (120, 99), (120, 151)):
            self.assertIsNone(f(x, y, self.RECTS), (x, y))

    def test_gap_between(self):
        self.assertIsNone(F("calib_hit")(120, 155, self.RECTS))

    def test_second_rect(self):
        self.assertEqual(F("calib_hit")(120, 180, self.RECTS), "down")

    def test_empty(self):
        self.assertIsNone(F("calib_hit")(0, 0, ()))


class TestCalibStep(unittest.TestCase):
    """签名: calib_step(vis, btn, hold_t0, last_btn, now, pressed, was_pressed, hit, repeat_ms, guard_ms)

    规则: 隐藏态仅在"新触摸边沿"唤出(本次消费, 不发动作); 可见态新按/换钮立即发一次;
          方向键按住每 repeat_ms 连发; dot/save 不连发; 松开清空 btn。
    防抖(guard_ms): 与"最近一次发出动作的同一按钮"、间隔 < guard_ms 的再次按下沿被吞
          (防触摸抖动"一下跳两项"); 换钮不受限。本类用例按默认 300ms 守卫跑。
    """

    def test_hidden_edge_press_shows_only(self):
        self.assertEqual(F("calib_step")(False, None, 0, None, 1000, True, False, "up", 120, 300),
                         (True, None, 1000, None, None))

    def test_hidden_held_press_stays_hidden(self):
        # 点 ● 隐藏后同一触摸尚未松开 → 不重新唤醒
        self.assertEqual(F("calib_step")(False, "dot", 900, "dot", 1000, True, True, "dot", 120, 300),
                         (False, None, 900, "dot", None))

    def test_hidden_no_press_stays_hidden(self):
        self.assertEqual(F("calib_step")(False, None, 0, None, 1000, False, False, None, 120, 300),
                         (False, None, 0, None, None))

    def test_visible_release_clears_btn(self):
        self.assertEqual(F("calib_step")(True, "up", 900, "up", 1000, False, True, None, 120, 300),
                         (True, None, 900, "up", None))

    def test_new_press_fires_once(self):
        self.assertEqual(F("calib_step")(True, None, 0, None, 1000, True, False, "up", 120, 300),
                         (True, "up", 1000, "up", "up"))

    def test_hold_before_repeat_no_act(self):
        self.assertEqual(F("calib_step")(True, "up", 1000, "up", 1090, True, True, "up", 120, 300),
                         (True, "up", 1000, "up", None))

    def test_hold_repeat_fires(self):
        self.assertEqual(F("calib_step")(True, "up", 1000, "up", 1120, True, True, "up", 120, 300),
                         (True, "up", 1120, "up", "up"))

    def test_dot_no_repeat(self):
        self.assertEqual(F("calib_step")(True, "dot", 1000, "dot", 1500, True, True, "dot", 120, 300),
                         (True, "dot", 1000, "dot", None))

    def test_save_no_repeat(self):
        self.assertEqual(F("calib_step")(True, "save", 1000, "save", 1500, True, True, "save", 120, 300),
                         (True, "save", 1000, "save", None))

    def test_mode_single_fire_no_repeat(self):
        # T/C 模式切换: 新按发一次, 按住不连发(防误切)
        self.assertEqual(F("calib_step")(True, None, 0, None, 1000, True, False, "mode", 120, 300),
                         (True, "mode", 1000, "mode", "mode"))
        self.assertEqual(F("calib_step")(True, "mode", 1000, "mode", 1500, True, True, "mode", 120, 300),
                         (True, "mode", 1000, "mode", None))

    def test_switch_button_immediate(self):
        # 换钮立即响应: 上一次动作按钮是 up, 80ms 后按 down 不受防抖限制
        self.assertEqual(F("calib_step")(True, "up", 1000, "up", 1050, True, True, "down", 120, 300),
                         (True, "down", 1050, "down", "down"))

    def test_press_off_buttons_no_act(self):
        self.assertEqual(F("calib_step")(True, None, 0, None, 1000, True, False, None, 120, 300),
                         (True, None, 1000, None, None))


class TestCalibDebounce(unittest.TestCase):
    """防抖(guard_ms): 同一按钮 < guard_ms 的重复按沿被吞; 换钮/超窗放行; guard=0 禁用"""

    GD = 300

    def test_same_button_within_guard_swallowed(self):
        f = F("calib_step")
        # 真点击 up(动作) → 松开 → 160ms 后同钮伪按沿(触摸抖动) → 吞
        _, btn, hold, last, act = f(True, None, 0, None, 1000, True, False, "up", 120, self.GD)
        self.assertEqual((btn, act), ("up", "up"))
        _, btn, hold, last, act = f(True, btn, hold, last, 1060, False, True, None, 120, self.GD)
        self.assertIsNone(act)
        _, btn, hold, last, act = f(True, btn, hold, last, 1160, True, False, "up", 120, self.GD)
        self.assertIsNone(act)                    # 距上次动作 160ms < 300 → 吞
        self.assertEqual((btn, last), ("up", "up"))
        # 距上次动作 > guard_ms 的同钮按下 → 放行(用户真点击第二下)
        _, _, _, _, act = f(True, None, hold, last, 1310, True, False, "up", 120, self.GD)
        self.assertEqual(act, "up")

    def test_swallowed_then_hold_repeats(self):
        # 被吞后按住不放: 连发通道不受防抖影响(仍以最近一次动作为计时基准)
        f = F("calib_step")
        _, btn, hold, last, act = f(True, None, 0, None, 1000, True, False, "up", 120, self.GD)
        self.assertEqual(act, "up")
        _, btn, hold, last, _ = f(True, btn, hold, last, 1060, False, True, None, 120, self.GD)
        _, btn, hold, last, act = f(True, btn, hold, last, 1160, True, False, "up", 120, self.GD)
        self.assertIsNone(act)                    # 伪按沿被吞
        _, _, _, _, act = f(True, btn, hold, last, 1180, True, True, "up", 120, self.GD)
        self.assertEqual(act, "up")               # 按住延续 → 连发起步(1160-1000=160 ≥ 120)

    def test_different_button_not_blocked(self):
        # 换钮不受防抖限制: up 动作后 80ms 按 down → 立即响应
        f = F("calib_step")
        _, _, _, last, act = f(True, None, 0, None, 1000, True, False, "up", 120, self.GD)
        self.assertEqual(act, "up")
        _, _, _, _, act = f(True, None, 0, last, 1080, True, False, "down", 120, self.GD)
        self.assertEqual(act, "down")

    def test_guard_zero_disables(self):
        # guard=0: 冷却禁用, 同钮快速重复按沿也放行(旧行为)
        f = F("calib_step")
        _, _, _, last, act = f(True, None, 0, None, 1000, True, False, "up", 120, 0)
        self.assertEqual(act, "up")
        _, _, _, _, act = f(True, None, 0, last, 1160, True, False, "up", 120, 0)
        self.assertEqual(act, "up")


class TestCalibFlow(unittest.TestCase):
    """多帧序列: 完整交互流程(新按→连发→松开→隐藏→未松手不弹回→唤醒→保存)逐帧断言"""

    def test_full_interaction_sequence(self):
        f = F("calib_step")
        REP = 120
        GD = 300
        vis, btn, hold, last = True, None, 0, None
        # 帧1: 按下 up(新触摸) → 立即一步
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1000, True, False, "up", REP, GD)
        self.assertEqual((vis, btn, act), (True, "up", "up"))
        # 帧2: 按住 50ms(未到连发间隔) → 无动作
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1050, True, True, "up", REP, GD)
        self.assertIsNone(act)
        # 帧3: 按住 130ms → 连发一步
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1130, True, True, "up", REP, GD)
        self.assertEqual((vis, btn, act), (True, "up", "up"))
        # 帧4: 松开 → 清按钮
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1200, False, True, None, REP, GD)
        self.assertEqual((vis, btn, act), (True, None, None))
        # 帧5: 按 dot → 动作 dot; 外部执行隐藏(模拟主循环: vis=False)
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1300, True, False, "dot", REP, GD)
        self.assertEqual(act, "dot")
        vis = False
        # 帧6: dot 未松手 → 保持隐藏(不弹回)
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1350, True, True, "dot", REP, GD)
        self.assertEqual((vis, act), (False, None))
        # 帧7: 松开 → 保持隐藏
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1400, False, True, None, REP, GD)
        self.assertEqual((vis, act), (False, None))
        # 帧8: 新触摸(任意位置) → 仅唤醒, 本次触摸不发动作
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1500, True, False, None, REP, GD)
        self.assertEqual((vis, act), (True, None))
        # 帧9: 点击 save → 动作 save(距上次动作 300ms 且换钮, 防抖放行)
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1550, False, True, None, REP, GD)
        vis, btn, hold, last, act = f(vis, btn, hold, last, 1600, True, False, "save", REP, GD)
        self.assertEqual(act, "save")


class TestReplaceJsonInt(unittest.TestCase):
    """行内第一个整数 token 替换；保留缩进/键/尾逗号；无整数 token 返回 None"""

    def test_basic(self):
        self.assertEqual(F("_replace_json_int")('    "laser_offset_x": 0,', 15),
                         '    "laser_offset_x": 15,')

    def test_replace_negative_source(self):
        self.assertEqual(F("_replace_json_int")('  "v": -12,', 5), '  "v": 5,')

    def test_to_negative(self):
        self.assertEqual(F("_replace_json_int")('  "v": 3,', -7), '  "v": -7,')

    def test_no_space_after_colon(self):
        self.assertEqual(F("_replace_json_int")('"v":-7,', 2), '"v":2,')

    def test_last_line_without_comma(self):
        self.assertEqual(F("_replace_json_int")('  "v": 9', 1), '  "v": 1')

    def test_no_int_returns_none(self):
        self.assertIsNone(F("_replace_json_int")('  "v": ,', 1))
        self.assertIsNone(F("_replace_json_int")('  "v": null', 1))


class TestCalibSaveText(unittest.TestCase):
    """config.json 文本级替换: 只动 laser_offset_x/y 两行, 其余逐字节保留"""

    def _src(self):
        with open(_CFG_JSON, encoding="utf-8") as f:
            return f.read()

    def test_real_config_roundtrip(self):
        out = F("calib_save_text")(self._src(), 12, -8)
        self.assertIsNotNone(out)
        cfg = json.loads(out)                       # 替换后仍为合法 JSON
        self.assertEqual(cfg["center"]["laser_offset_x"], 12)
        self.assertEqual(cfg["center"]["laser_offset_y"], -8)

    def test_only_two_lines_changed(self):
        src = self._src()
        out = F("calib_save_text")(src, 12, -8)
        a, b = src.split("\n"), out.split("\n")
        self.assertEqual(len(a), len(b))
        diff = [i for i in range(len(a)) if a[i] != b[i]]
        self.assertEqual(len(diff), 2, "应恰好 2 行变化, 实际 %d 行: %s" % (len(diff), diff))
        for i in diff:
            s = a[i].strip()
            self.assertTrue(s.startswith('"laser_offset_x"') or s.startswith('"laser_offset_y"'),
                            "变化的行必须是 laser_offset 键: %r" % a[i])

    def test_missing_keys_none(self):
        self.assertIsNone(F("calib_save_text")('{"center": {"x": 1}}', 1, 1))

    def test_idempotent(self):
        src = self._src()
        once = F("calib_save_text")(src, 12, -8)
        twice = F("calib_save_text")(once, 12, -8)
        self.assertEqual(once, twice)


class TestCalibRects(unittest.TestCase):
    """按钮矩形生成: 8 键 / 全部在屏内 / 两两不重叠 / 左右组对称"""

    def _rects(self):
        return F("_calib_rects")(80, 0, 640, 480, 56)

    def test_names_and_count(self):
        self.assertEqual([r[0] for r in self._rects()],
                         ["up", "down", "left", "right", "mode", "dot", "save", "param"])

    def test_all_inside_screen(self):
        for name, x0, y0, x1, y1 in self._rects():
            self.assertTrue(0 <= x0 < x1 <= 800, (name, x0, x1))
            self.assertTrue(0 <= y0 < y1 <= 480, (name, y0, y1))

    def test_no_overlap(self):
        rects = self._rects()
        for i in range(len(rects)):
            for j in range(i + 1, len(rects)):
                a, b = rects[i], rects[j]
                overlap = a[1] < b[3] and b[1] < a[3] and a[2] < b[4] and b[2] < a[4]
                self.assertFalse(overlap, "按钮重叠: %s x %s" % (a[0], b[0]))

    def test_up_down_group_centered_on_midline(self):
        d = dict((r[0], r[1:]) for r in self._rects())
        upy, dny = d["up"][1], d["down"][3]          # 组顶 / 组底(y0 与 y1)
        self.assertAlmostEqual((upy + dny) / 2.0, 240, delta=40)


if __name__ == "__main__":
    unittest.main(verbosity=2)
