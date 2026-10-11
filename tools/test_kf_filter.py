"""V8 卡尔曼滤波(视觉误差平滑 + 速度前馈) —— 主机侧测试（AST 提取 main.py 纯函数并断言行为）。

覆盖：
  - kf_step    2 状态 KF(位置+速度, 观测位置): 首帧观测初始化 / 常值收敛 / 噪声抑制 /
               阶跃跟随 / dt 兜底与截断 / 变 dt 无 NaN / lead 速度前馈 / 输出限幅
  - _kf_clear  按轴重置滤波器状态(丢靶/搜索/死区时与 _pid_clear 同点调用)
  - 静态守门: main.py 模块级定义 KF 参数与状态; gimbal_send 接线到 kf_step(缺则红灯)

原理：从 k230/main.py AST 提取上述函数源码，在最小环境执行
（时间以 _FakeTime 注入, ticks_diff = 差值; KF 状态以 _KF_INIT 注入初值）。
运行：python tools/test_kf_filter.py
"""
import ast
import math
import os
import random
import types
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))

_FUNC_NAMES = ("kf_step", "_kf_clear")

# main.py 必须模块级定义的 KF 参数/状态名(静态守门用)
_KF_PARAM_VARS = ("KF_Y_EN", "KF_Y_QPOS", "KF_Y_QVEL", "KF_Y_R", "KF_Y_LEAD", "KF_Y_LIM",
                  "KF_P_EN", "KF_P_QPOS", "KF_P_QVEL", "KF_P_R", "KF_P_LEAD", "KF_P_LIM")
_KF_STATE_VARS = ("_kf_yaw", "_kf_pit")

# 滤波器初始状态(与 main.py 一致: P=diag(5,5), t=0 表首帧)
_KF_INIT = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]


def _extract_func(src, name):
    """返回顶层函数源码；找不到时抛 AssertionError"""
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到函数 %s" % name)


class _FakeTime(types.ModuleType):
    """stub MicroPython time（ticks_diff 语义 = 差值）"""

    def __init__(self):
        super().__init__("time")

    def ticks_diff(self, a, b):
        return a - b


_ENV = None


def load_funcs():
    """从 main.py 提取 kf_step / _kf_clear（KF 状态注入 _KF_INIT 初值）"""
    global _ENV
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    env = {"time": _FakeTime()}
    env["_kf_yaw"] = list(_KF_INIT)
    env["_kf_pit"] = list(_KF_INIT)
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
    """恢复 KF 状态初值(每个用例前调用, 防跨用例污染; 唯一加载入口, 防 _ENV 被二次覆盖)"""
    _ensure_loaded()
    _ENV["_kf_yaw"] = list(_KF_INIT)
    _ENV["_kf_pit"] = list(_KF_INIT)


_FNS = None


def _ensure_loaded():
    """唯一惰性加载入口: _FNS 与 _ENV 同步只初始化一次"""
    global _FNS
    if _FNS is None:
        _FNS = load_funcs()


def F(name):
    _ensure_loaded()
    return _FNS[name]


class TestKfStep(unittest.TestCase):
    """kf_step(z, now_ms, st, q_pos, q_vel, r, lead_s, lim) -> 输出(lead 修正+限幅后)"""

    QP, QV, R = 0.5, 5.0, 2.0
    LIM = 150.0
    DT = 33          # 帧间隔 ms(≈30fps)

    def setUp(self):
        reset_env()

    def _run(self, zs, st=None, lead_s=0.0, lim=None, dt=None):
        """喂观测序列, 返回 (输出列表, st, 结束时刻)"""
        f = F("kf_step")
        st = st if st is not None else list(_KF_INIT)
        dt = self.DT if dt is None else dt
        lim = self.LIM if lim is None else lim
        t = 1000
        outs = []
        for z in zs:
            outs.append(f(z, t, st, self.QP, self.QV, self.R, lead_s, lim))
            t += dt
        return outs, st, t

    # ---- 初始化与收敛 ----

    def test_first_frame_initializes(self):
        # 首帧(未初始化): 直接用观测, 速度=0, 记时刻; lead 无效(v=0)
        st = list(_KF_INIT)
        out = F("kf_step")(50.0, 12345, st, self.QP, self.QV, self.R, 0.5, self.LIM)
        self.assertAlmostEqual(out, 50.0, places=6)
        self.assertEqual(st[0], 50.0)
        self.assertEqual(st[1], 0.0)
        self.assertEqual(st[6], 12345)
        self.assertEqual(st[2], 5.0)          # P 复位为初值 diag(5,5)
        self.assertEqual(st[5], 5.0)

    def test_constant_converges(self):
        # 常值观测: 输出趋近观测, 速度衰减趋零(不漂移)
        outs, st, _ = self._run([80.0] * 80)
        self.assertAlmostEqual(outs[-1], 80.0, delta=2.0)
        self.assertLess(abs(st[1]), 1.0)

    def test_negative_converges_symmetric(self):
        outs, st, _ = self._run([-60.0] * 80)
        self.assertAlmostEqual(outs[-1], -60.0, delta=2.0)

    def test_noise_suppressed(self):
        # ±3px 均匀噪声: 输出方差显著小于输入方差(平滑有效)
        rnd = random.Random(42)
        zs = [50.0 + rnd.uniform(-3.0, 3.0) for _ in range(120)]
        outs, _, _ = self._run(zs)
        ins = zs[20:]                          # 跳过瞬态
        ous = outs[20:]
        var_in = sum((z - 50.0) ** 2 for z in ins) / len(ins)
        var_out = sum((o - 50.0) ** 2 for o in ous) / len(ous)
        self.assertLess(var_out, var_in * 0.6,
                        "噪声未被抑制: var_in=%.3f var_out=%.3f" % (var_in, var_out))

    def test_step_response_follows(self):
        # 0 → 100 阶跃: ~1.3s(40 帧)内跟上大部分(不因滤波僵死)
        outs, _, _ = self._run([0.0] + [100.0] * 40)
        self.assertGreater(outs[-1], 85.0)

    # ---- dt 边界 ----

    def test_same_tick_no_nan(self):
        # dt=0(同 tick 重入): 兜底 0.033, 数值有限
        st = list(_KF_INIT)
        f = F("kf_step")
        f(10.0, 1000, st, self.QP, self.QV, self.R, 0.0, self.LIM)
        out = f(12.0, 1000, st, self.QP, self.QV, self.R, 0.0, self.LIM)
        self.assertTrue(math.isfinite(out))

    def test_huge_gap_clamped_no_explode(self):
        # 长时间断流(dt 巨大): 截断到 0.5s, 预测不爆炸, 输出由观测拉回
        st = list(_KF_INIT)
        f = F("kf_step")
        f(10.0, 1000, st, self.QP, self.QV, self.R, 0.0, self.LIM)
        out = f(20.0, 10 ** 7, st, self.QP, self.QV, self.R, 0.0, self.LIM)
        self.assertTrue(math.isfinite(out))
        self.assertLess(abs(out), 1000.0)

    def test_variable_dt_finite(self):
        # 变 dt 序列(含抖动): 全程数值有限
        f = F("kf_step")
        st = list(_KF_INIT)
        t = 1000
        for i, dt in enumerate([33, 16, 500, 1, 100, 33, 800]):
            out = f(float(i * 10), t, st, self.QP, self.QV, self.R, 0.0, self.LIM)
            self.assertTrue(math.isfinite(out), "dt=%d 时输出非有限值" % dt)
            t += dt

    # ---- lead 速度前馈与限幅 ----

    def test_lead_advances_by_v_times_lead(self):
        # 构造 v=100px/s 的状态: lead=0.1s 的输出应领先 lead=0 约 10px
        base = [0.0, 100.0, 5.0, 0.0, 0.0, 5.0, 1000]
        st1, st2 = list(base), list(base)
        f = F("kf_step")
        out1 = f(0.0, 1033, st1, self.QP, self.QV, self.R, 0.0, 1e9)
        out2 = f(0.0, 1033, st2, self.QP, self.QV, self.R, 0.1, 1e9)
        self.assertGreater(out2 - out1, 5.0)
        # 两条滤波器除 lead 外状态应一致(lead 不写入状态)
        self.assertAlmostEqual(st1[0], st2[0], places=6)
        self.assertAlmostEqual(st1[1], st2[1], places=6)

    def test_lead_zero_is_identity(self):
        # lead=0: 输出即滤波位置(不加偏移)
        base = [30.0, 50.0, 5.0, 0.0, 0.0, 5.0, 1000]
        st = list(base)
        out = F("kf_step")(30.0, 1033, st, self.QP, self.QV, self.R, 0.0, 1e9)
        self.assertAlmostEqual(out, st[0], places=6)

    def test_output_clamped(self):
        # 大观测: 输出被钳到 ±lim(前馈峰值防护)
        st = list(_KF_INIT)
        out = F("kf_step")(1000.0, 1000, st, self.QP, self.QV, self.R, 0.0, 150.0)
        self.assertEqual(out, 150.0)
        st = list(_KF_INIT)
        out = F("kf_step")(-1000.0, 1000, st, self.QP, self.QV, self.R, 0.0, 150.0)
        self.assertEqual(out, -150.0)


class TestKfClear(unittest.TestCase):
    """_kf_clear(axis): 0=yaw / 1=pitch; 重置为初值(与 _pid_clear 对称)"""

    def setUp(self):
        reset_env()

    def test_clear_yaw_resets_to_init(self):
        _ENV["_kf_yaw"] = [9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 999]
        F("_kf_clear")(0)
        self.assertEqual(_ENV["_kf_yaw"], _KF_INIT)

    def test_clear_pitch_resets_to_init(self):
        _ENV["_kf_pit"] = [7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 555]
        F("_kf_clear")(1)
        self.assertEqual(_ENV["_kf_pit"], _KF_INIT)

    def test_clear_one_axis_leaves_other(self):
        _ENV["_kf_yaw"] = [9.0, 0.0, 5.0, 0.0, 0.0, 5.0, 1]
        _ENV["_kf_pit"] = [9.0, 0.0, 5.0, 0.0, 0.0, 5.0, 2]
        F("_kf_clear")(0)
        self.assertEqual(_ENV["_kf_yaw"], _KF_INIT)
        self.assertEqual(_ENV["_kf_pit"], [9.0, 0.0, 5.0, 0.0, 0.0, 5.0, 2])

    def test_reinit_after_clear_no_velocity_jump(self):
        # 清后首帧: 观测初始化, 速度=0(无残速突跳)
        st = _ENV["_kf_yaw"]
        F("kf_step")(10.0, 1000, st, 0.5, 5.0, 2.0, 0.0, 150.0)
        F("kf_step")(90.0, 1033, st, 0.5, 5.0, 2.0, 0.0, 150.0)
        F("_kf_clear")(0)
        out = F("kf_step")(40.0, 2000, _ENV["_kf_yaw"], 0.5, 5.0, 2.0, 0.0, 150.0)
        self.assertAlmostEqual(out, 40.0, places=6)
        self.assertEqual(_ENV["_kf_yaw"][1], 0.0)


class TestMainStaticGuards(unittest.TestCase):
    """静态守门: main.py 必须定义 KF 参数/状态, 且 gimbal_send 真的接线到 kf_step"""

    def _src(self):
        with open(_MAIN_PY, encoding="utf-8") as f:
            return f.read()

    def test_kf_vars_defined(self):
        tree = ast.parse(self._src())
        defined = set()
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for t in node.targets:
                    if isinstance(t, ast.Name):
                        defined.add(t.id)
        for name in _KF_PARAM_VARS + _KF_STATE_VARS:
            self.assertIn(name, defined, "main.py 缺少模块级定义: %s" % name)

    def test_kf_wired_into_gimbal(self):
        src = self._src()
        self.assertIn("kf_step(", src)
        self.assertIn("KF_Y_EN", src)
        self.assertIn("KF_P_EN", src)
        self.assertIn("_kf_clear(", src)

    def test_kf_default_off_guard(self):
        # 默认关的回归锚点: config 中 kf_enabled 必须为 false(避免误开)
        cfg_path = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "config.json"))
        import json
        with open(cfg_path, encoding="utf-8") as f:
            cfg = json.load(f)
        self.assertFalse(cfg["motor"].get("kf_enabled", False), "motor.kf_enabled 默认必须为 false")
        self.assertFalse(cfg["pitch_motor"].get("kf_enabled", False), "pitch_motor.kf_enabled 默认必须为 false")


if __name__ == "__main__":
    unittest.main(verbosity=2)
