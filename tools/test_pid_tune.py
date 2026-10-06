"""V7.9 PID 速度控制 + 触屏调参 —— 主机侧测试（AST 提取 main.py 纯函数并断言行为）。

覆盖：
  - pid_step    一阶 PID 步进(速度式): P/I/D 计算 / 首帧兜底 / 长帧截断 / 限幅 / 抗饱和冻结
  - pid_speed   PID 输出 → (方向, 转速); |u|<min_rpm 抬到 min_rpm
  - fmt_num     数值 → JSON 文本(int 原样 / float 去尾零)
  - fmt_disp    触屏显示值(pid → ON/OFF; float 2 位小数)
  - param_next / param_calc   调参项循环 / 步进(clamp + 网格对齐; pid toggle)
  - param_value / param_apply 读/写运行时全局(acc 同步实例); 注入 env 断言
  - param_save_items          写回项收集(键集与真 config 对齐, 防"保存永远失败")
  - _replace_json_token       行内标量 token 替换(数字/true/false)
  - param_save_text           段内键替换(段隔离: motor 与 pitch_motor 的同名键不串)
  - _param_rects / param_step 调参页按钮布局 / 触摸状态机(plus/minus 连发)

原理：从 k230/main.py AST 提取上述函数源码，在最小环境执行
（触屏调参读写的 M_*/P_* 变量以注入初值模拟; 其余为纯函数）。
运行：python tools/test_pid_tune.py
"""
import ast
import json
import os
import types
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))
_CFG_JSON = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "config.json"))

_FUNC_NAMES = (
    "pid_step", "pid_speed",
    "fmt_num", "fmt_disp", "param_next", "param_calc",
    "param_value", "param_apply", "param_save_items",
    "_replace_json_token", "param_save_text",
    "param_step", "_param_rects",
)

# 被提取函数引用的模块级常量表(一并从 main.py 提取执行 —— 测试不自造取值, 缺定义即暴露)
_CONST_NAMES = ("PARAM_ITEMS", "PARAM_NAMES", "PARAM_SPEC")

# 触屏调参读写的运行时全局(注入初值 = config 默认量级); param_apply 会写回本 env
_TUNE_VARS = {
    "M_PID_EN": False, "M_PID_KP": 0.28, "M_PID_KI": 0.0, "M_PID_KD": 0.0,
    "M_MAX_RPM": 60, "M_MIN_RPM": 5, "M_DEADBAND": 5, "M_ACC": 200, "M_DX_FULL": 200,
    "P_PID_EN": True, "P_PID_KP": 0.14, "P_PID_KI": 0.0, "P_PID_KD": 0.0,
    "P_MAX_RPM": 30, "P_MIN_RPM": 3, "P_DEADBAND": 5, "P_ACC": 200, "P_DY_FULL": 200,
}
# main.py 必须模块级定义的调参相关名字(静态守门用; _pid_* 为 PID 运行状态)
_STATE_VARS = ("_pid_yaw_i", "_pid_yaw_e", "_pid_yaw_t",
               "_pid_pit_i", "_pid_pit_e", "_pid_pit_t")


def _extract_func(src, name):
    """返回顶层函数源码；找不到时抛 AssertionError"""
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到函数 %s" % name)


def _extract_assign(src, name):
    """返回顶层 `name = ...` 赋值源码；找不到时抛 AssertionError"""
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id == name:
                    return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到模块级赋值 %s = ..." % name)


class _FakeTime(types.ModuleType):
    """stub MicroPython time（ticks_diff 语义 = 差值）"""

    def __init__(self):
        super().__init__("time")

    def ticks_diff(self, a, b):
        return a - b


_ENV = None


def load_funcs():
    """从 main.py 提取 13 个函数（触屏调参变量注入 _TUNE_VARS 初值）"""
    global _ENV
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    env = {"time": _FakeTime(), "_yaw": None, "_pitch": None}
    env.update(_TUNE_VARS)
    # 常量表从 main.py 提取执行(缺定义直接报错暴露, 测试不自造取值)
    for cname in _CONST_NAMES:
        exec(_extract_assign(src, cname), env)
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
    _ENV = env
    return fns


def reset_env():
    """把触屏调参变量恢复为初值(每个用例前调用, 防跨用例污染)"""
    _ENV.update(_TUNE_VARS)
    _ENV["_yaw"] = None
    _ENV["_pitch"] = None


_FNS = None


def F(name):
    global _FNS
    if _FNS is None:
        _FNS = load_funcs()
    return _FNS[name]


def ENV():
    F("pid_step")            # 确保已加载
    return _ENV


class TestPidStep(unittest.TestCase):
    """签名: pid_step(err, now_ms, i_acc, e_prev, t_prev, kp, ki, kd, lim) → (u, i_acc, e_prev)"""

    def test_p_only_first_frame(self):
        # 首帧: dt=0.033 兜底; 微分为 0(无历史); u = kp*err
        u, i, e = F("pid_step")(100, 1000, 0.0, 0, 0, 0.3, 0.0, 0.0, 60)
        self.assertAlmostEqual(u, 30.0)
        self.assertAlmostEqual(i, 100 * 0.033)
        self.assertEqual(e, 100)

    def test_p_plus_i_second_frame(self):
        u, i, e = F("pid_step")(90, 1033, 3.3, 100, 1000, 0.3, 0.1, 0.0, 60)
        self.assertAlmostEqual(i, 3.3 + 90 * 0.033)
        self.assertAlmostEqual(u, 0.3 * 90 + 0.1 * (3.3 + 90 * 0.033))
        self.assertEqual(e, 90)

    def test_d_term(self):
        # dt=(1033-1000)/1000=0.033; d=(90-100)/0.033; u=kp*90+kd*d
        u, _i, _e = F("pid_step")(90, 1033, 0.0, 100, 1000, 0.0, 0.0, 0.5, 1000)
        self.assertAlmostEqual(u, 0.5 * (90 - 100) / 0.033)

    def test_negative_symmetric(self):
        u, _i, _e = F("pid_step")(-100, 1000, 0.0, 0, 0, 0.3, 0.0, 0.0, 60)
        self.assertAlmostEqual(u, -30.0)

    def test_clamp_positive(self):
        u, i, _e = F("pid_step")(500, 1000, 0.0, 0, 0, 1.0, 1.0, 0.0, 10)
        self.assertEqual(u, 10)
        # 正饱和 + 正误差 → 积分冻结(不累积)
        self.assertEqual(i, 0.0)

    def test_clamp_negative(self):
        u, i, _e = F("pid_step")(-500, 1000, 0.0, 0, 0, 1.0, 1.0, 0.0, 10)
        self.assertEqual(u, -10)
        self.assertEqual(i, 0.0)

    def test_antiwindup_releases_when_err_reverses(self):
        # 饱和但误差反向(在退出饱和): 积分不冻结, 正常累积
        u, i, _e = F("pid_step")(-5, 1033, 100.0, -5, 1000, 0.3, 1.0, 0.0, 10)
        self.assertEqual(u, 10)
        self.assertAlmostEqual(i, 100.0 + (-5) * 0.033)

    def test_long_frame_clamped_dt(self):
        # dt=4.0s → 截到 0.5s: d=(10-0)/0.5=20; i=10*0.5=5; u=0.3*10+20=23
        u, i, _e = F("pid_step")(10, 5000, 0.0, 0, 1000, 0.3, 0.0, 1.0, 60)
        self.assertAlmostEqual(u, 23.0)
        self.assertAlmostEqual(i, 5.0)

    def test_same_tick_fallback(self):
        # 同刻(回绕/0 时长) → dt 用 0.033 兜底, 不负向放大微分
        u, i, _e = F("pid_step")(10, 1000, 0.0, 0, 1000, 0.3, 0.0, 0.0, 60)
        self.assertAlmostEqual(u, 3.0)
        self.assertAlmostEqual(i, 10 * 0.033)


class TestPidSpeed(unittest.TestCase):
    """pid_speed(u, min_rpm) → (方向 1/-1/0, 绝对转速)"""

    def test_zero(self):
        self.assertEqual(F("pid_speed")(0.0, 5), (0, 0))

    def test_below_min_raised(self):
        self.assertEqual(F("pid_speed")(3, 5), (1, 5))
        self.assertEqual(F("pid_speed")(-3, 5), (-1, 5))

    def test_at_above_min_kept(self):
        self.assertEqual(F("pid_speed")(5, 5), (1, 5))
        self.assertEqual(F("pid_speed")(10, 5), (1, 10))
        self.assertEqual(F("pid_speed")(-10, 5), (-1, 10))


class TestFmt(unittest.TestCase):
    def test_fmt_num_int(self):
        self.assertEqual(F("fmt_num")(60), "60")
        self.assertEqual(F("fmt_num")(0), "0")
        self.assertEqual(F("fmt_num")(-12), "-12")

    def test_fmt_num_float(self):
        self.assertEqual(F("fmt_num")(0.28), "0.28")
        self.assertEqual(F("fmt_num")(0.3), "0.3")
        self.assertEqual(F("fmt_num")(1.0), "1")
        self.assertEqual(F("fmt_num")(100.0), "100")
        self.assertEqual(F("fmt_num")(0.14), "0.14")

    def test_fmt_disp(self):
        self.assertEqual(F("fmt_disp")("kp", 0.28), "0.28")
        self.assertEqual(F("fmt_disp")("max", 60), "60")
        self.assertEqual(F("fmt_disp")("pid", 1), "ON")
        self.assertEqual(F("fmt_disp")("pid", 0), "OFF")


class TestParamNextCalc(unittest.TestCase):
    def test_next_cycle(self):
        f = F("param_next")
        self.assertEqual(f("pid"), "kp")
        self.assertEqual(f("kp"), "ki")
        self.assertEqual(f("kd"), "max")
        self.assertEqual(f("full"), "pid")
        self.assertEqual(f("unknown"), "pid")

    def test_calc_float_step_and_round(self):
        f = F("param_calc")
        self.assertEqual(f("kp", 0.28, 1), 0.3)        # 0.28+0.02 浮点尾数被舍入
        self.assertEqual(f("kp", 0.28, -1), 0.26)
        self.assertEqual(f("ki", 0.0, 1), 0.01)
        self.assertEqual(f("kd", 0.5, -1), 0.49)

    def test_calc_int_step(self):
        f = F("param_calc")
        self.assertEqual(f("max", 60, 1), 61)
        self.assertEqual(f("acc", 200, -1), 195)
        self.assertEqual(f("full", 200, 1), 210)
        v = f("max", 60, 1)
        self.assertIsInstance(v, int)

    def test_calc_clamp(self):
        f = F("param_calc")
        self.assertEqual(f("kp", 5.0, 1), 5.0)
        self.assertEqual(f("ki", 0.0, -1), 0.0)
        self.assertEqual(f("max", 500, 1), 500)
        self.assertEqual(f("min", 0, -1), 0)

    def test_calc_pid_toggle(self):
        f = F("param_calc")
        self.assertEqual(f("pid", 0, 1), 1)
        self.assertEqual(f("pid", 1, 1), 1)
        self.assertEqual(f("pid", 1, -1), 0)
        self.assertEqual(f("pid", 0, -1), 0)


class TestParamValueApply(unittest.TestCase):
    """param_value / param_apply 读写注入 env 的运行时全局"""

    def setUp(self):
        reset_env()

    def test_value_reads(self):
        f = F("param_value")
        self.assertEqual(f(0, "kp"), 0.28)
        self.assertEqual(f(0, "max"), 60)
        self.assertEqual(f(1, "kp"), 0.14)
        self.assertEqual(f(1, "full"), 200)
        self.assertEqual(f(1, "pid"), 1)
        self.assertEqual(f(0, "pid"), 0)

    def test_apply_writes_global(self):
        f = F("param_apply")
        self.assertEqual(f(0, "kp", 1), 0.3)
        self.assertEqual(ENV()["M_PID_KP"], 0.3)
        self.assertEqual(f(1, "max", -1), 29)
        self.assertEqual(ENV()["P_MAX_RPM"], 29)

    def test_apply_pid_toggle(self):
        f = F("param_apply")
        self.assertEqual(f(0, "pid", 1), 1)
        self.assertIs(ENV()["M_PID_EN"], True)
        self.assertEqual(f(1, "pid", -1), 0)
        self.assertIs(ENV()["P_PID_EN"], False)

    def test_apply_acc_syncs_instance(self):
        class FakeMotor(object):
            def __init__(self):
                self.acc = 200

        m = FakeMotor()
        ENV()["_yaw"] = m
        v = F("param_apply")(0, "acc", 1)
        self.assertEqual(v, 205)
        self.assertEqual(m.acc, 205)

    def test_apply_clamped_noop(self):
        f = F("param_apply")
        self.assertEqual(f(0, "min", -1), 4)     # 5 → 4 正常步进
        reset_env()
        ENV()["M_MIN_RPM"] = 0
        self.assertEqual(f(0, "min", -1), 0)     # 下限处不再降


class TestParamSaveItems(unittest.TestCase):
    """param_save_items 收集 18 项; 键集必须与真 config 对齐(防 S 保存永远失败)"""

    def setUp(self):
        reset_env()

    def test_collect_and_values(self):
        items = F("param_save_items")()
        self.assertEqual(len(items), 18)
        d = dict(((sec + "." + k), t) for sec, k, t in items)
        self.assertEqual(d["motor.pid_enabled"], "false")
        self.assertEqual(d["pitch_motor.pid_enabled"], "true")
        self.assertEqual(d["motor.pid_kp"], "0.28")
        self.assertEqual(d["pitch_motor.pid_kp"], "0.14")
        self.assertEqual(d["motor.max_rpm"], "60")
        self.assertEqual(d["pitch_motor.dy_full_px"], "200")
        self.assertEqual(d["motor.acc"], "200")

    def test_keys_exist_in_real_config(self):
        with open(_CFG_JSON, encoding="utf-8") as f:
            cfg = json.load(f)
        for sec, key, _t in F("param_save_items")():
            self.assertIn(sec, cfg, "config.json 缺段: %s" % sec)
            self.assertIn(key, cfg[sec], "config.json 缺键: %s.%s" % (sec, key))

    def test_save_items_roundtrip_real_config(self):
        """集成: param_save_items 的 18 项文本对真 config 必须全部命中(S 键真实路径)"""
        with open(_CFG_JSON, encoding="utf-8") as f:
            src = f.read()
        out = F("param_save_text")(src, F("param_save_items")())
        self.assertIsNotNone(out, "18 项对真 config 有未命中键(键名/段名不符)")
        cfg = json.loads(out)
        self.assertEqual(cfg["motor"]["pid_kp"], 0.28)
        self.assertEqual(cfg["pitch_motor"]["pid_kp"], 0.14)
        self.assertIs(cfg["motor"]["pid_enabled"], False)


class TestReplaceJsonToken(unittest.TestCase):
    def test_int_replace(self):
        self.assertEqual(F("_replace_json_token")('    "v": 5,', "7"),
                         '    "v": 7,')

    def test_negative_to_float(self):
        self.assertEqual(F("_replace_json_token")('  "v": -12,', "0.5"),
                         '  "v": 0.5,')

    def test_bool_replace(self):
        self.assertEqual(F("_replace_json_token")('  "v": false,', "true"),
                         '  "v": true,')

    def test_no_space_and_tail(self):
        self.assertEqual(F("_replace_json_token")('"v":7,', "2"), '"v":2,')
        self.assertEqual(F("_replace_json_token")('  "v": 9', "1"), '  "v": 1')

    def test_empty_returns_none(self):
        self.assertIsNone(F("_replace_json_token")('  "v": ,', "1"))


_MOCK_CFG = """{
  "motor": {
    "min_rpm": 5,
    "max_rpm": 60,
    "dx_full_px": 200,
    "deadband_px": 5,
    "acc": 200,
    "pid_enabled": false,
    "pid_kp": 0.28,
    "pid_ki": 0.0,
    "pid_kd": 0.0
  },
  "pitch_motor": {
    "min_rpm": 3,
    "max_rpm": 30,
    "dy_full_px": 200,
    "deadband_px": 5,
    "acc": 200,
    "pid_enabled": false,
    "pid_kp": 0.14,
    "pid_ki": 0.0,
    "pid_kd": 0.0
  },
  "other": {
    "max_rpm": 99
  }
}"""

_MOCK_ITEMS = (
    ("motor", "pid_enabled", "true"),
    ("motor", "pid_kp", "0.5"),
    ("motor", "pid_ki", "0.02"),
    ("motor", "pid_kd", "0.01"),
    ("motor", "min_rpm", "6"),
    ("motor", "max_rpm", "70"),
    ("motor", "dx_full_px", "210"),
    ("motor", "deadband_px", "6"),
    ("motor", "acc", "205"),
    ("pitch_motor", "pid_enabled", "true"),
    ("pitch_motor", "pid_kp", "0.2"),
    ("pitch_motor", "pid_ki", "0.01"),
    ("pitch_motor", "pid_kd", "0.01"),
    ("pitch_motor", "min_rpm", "4"),
    ("pitch_motor", "max_rpm", "40"),
    ("pitch_motor", "dy_full_px", "220"),
    ("pitch_motor", "deadband_px", "7"),
    ("pitch_motor", "acc", "210"),
)


class TestParamSaveText(unittest.TestCase):
    """段内键替换: motor 与 pitch_motor 同名键不互相串; 全命中才通过"""

    def test_mock_roundtrip_and_section_isolation(self):
        out = F("param_save_text")(_MOCK_CFG, _MOCK_ITEMS)
        self.assertIsNotNone(out)
        cfg = json.loads(out)
        self.assertEqual(cfg["motor"]["max_rpm"], 70)
        self.assertEqual(cfg["pitch_motor"]["max_rpm"], 40)      # 段隔离: 不串
        self.assertEqual(cfg["motor"]["pid_kp"], 0.5)
        self.assertEqual(cfg["pitch_motor"]["pid_kp"], 0.2)
        self.assertIs(cfg["motor"]["pid_enabled"], True)
        self.assertIs(cfg["pitch_motor"]["pid_enabled"], True)
        self.assertEqual(cfg["other"]["max_rpm"], 99)            # 段外同名键不动

    def test_only_target_lines_changed(self):
        out = F("param_save_text")(_MOCK_CFG, _MOCK_ITEMS)
        a, b = _MOCK_CFG.split("\n"), out.split("\n")
        self.assertEqual(len(a), len(b))
        diff = [i for i in range(len(a)) if a[i] != b[i]]
        self.assertEqual(len(diff), len(_MOCK_ITEMS), "应恰好 %d 行变化: %s"
                         % (len(_MOCK_ITEMS), diff))

    def test_idempotent(self):
        once = F("param_save_text")(_MOCK_CFG, _MOCK_ITEMS)
        twice = F("param_save_text")(once, _MOCK_ITEMS)
        self.assertEqual(once, twice)

    def test_missing_key_none(self):
        self.assertIsNone(F("param_save_text")(_MOCK_CFG,
                                               (("motor", "not_exist", "1"),)))

    def test_real_config_roundtrip(self):
        with open(_CFG_JSON, encoding="utf-8") as f:
            src = f.read()
        out = F("param_save_text")(src, _MOCK_ITEMS)
        self.assertIsNotNone(out)
        cfg = json.loads(out)                                    # 替换后仍为合法 JSON
        self.assertEqual(cfg["motor"]["max_rpm"], 70)
        self.assertEqual(cfg["pitch_motor"]["max_rpm"], 40)
        self.assertEqual(cfg["motor"]["pid_kp"], 0.5)


class TestParamRects(unittest.TestCase):
    def _rects(self):
        return F("_param_rects")(80, 0, 640, 480, 56)

    def test_names_and_count(self):
        self.assertEqual([r[0] for r in self._rects()],
                         ["plus", "minus", "item", "axis", "save", "back"])

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


class TestParamStep(unittest.TestCase):
    """签名: param_step(vis, btn, hold_t0, now, pressed, was_pressed, hit, repeat_ms)

    规则同 calib_step; 连发按钮 = plus/minus; item/axis/save/back 单发。
    """

    def test_plus_repeat(self):
        f = F("param_step")
        self.assertEqual(f(True, None, 0, 1000, True, False, "plus", 120),
                         (True, "plus", 1000, "plus"))
        self.assertEqual(f(True, "plus", 1000, 1090, True, True, "plus", 120),
                         (True, "plus", 1000, None))
        self.assertEqual(f(True, "plus", 1000, 1120, True, True, "plus", 120),
                         (True, "plus", 1120, "plus"))

    def test_minus_repeat(self):
        f = F("param_step")
        self.assertEqual(f(True, "minus", 1000, 1120, True, True, "minus", 120),
                         (True, "minus", 1120, "minus"))

    def test_single_fire_buttons(self):
        f = F("param_step")
        for name in ("item", "axis", "save", "back"):
            self.assertEqual(f(True, None, 0, 1000, True, False, name, 120),
                             (True, name, 1000, name), name)
            self.assertEqual(f(True, name, 1000, 1500, True, True, name, 120),
                             (True, name, 1000, None), name)

    def test_hidden_wake_edge(self):
        f = F("param_step")
        self.assertEqual(f(False, None, 0, 1000, True, False, None, 120),
                         (True, None, 1000, None))
        self.assertEqual(f(False, "back", 900, 1000, True, True, "back", 120),
                         (False, None, 900, None))

    def test_release_clears(self):
        self.assertEqual(F("param_step")(True, "plus", 900, 1000, False, True, None, 120),
                         (True, None, 900, None))


class TestMainStaticGuards(unittest.TestCase):
    """静态守门: main.py 必须定义调参引用的模块级变量, 且 gimbal_send 真的接线到 pid_step"""

    def _src(self):
        with open(_MAIN_PY, encoding="utf-8") as f:
            return f.read()

    def test_tune_vars_defined(self):
        tree = ast.parse(self._src())
        defined = set()
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for t in node.targets:
                    if isinstance(t, ast.Name):
                        defined.add(t.id)
        for name in list(_TUNE_VARS.keys()) + list(_STATE_VARS):
            self.assertIn(name, defined, "main.py 缺少模块级定义: %s" % name)

    def test_pid_wired_into_gimbal(self):
        src = self._src()
        self.assertIn("pid_step(", src)
        self.assertIn("pid_speed(", src)
        self.assertIn("M_PID_EN", src)
        self.assertIn("P_PID_EN", src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
