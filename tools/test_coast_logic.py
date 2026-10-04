"""丢靶预判状态机主机侧测试 —— 提取 main.py 的 coast_step 纯函数并断言其状态迁移。

背景: 「丢靶预判」= 目标从画面左右两侧丢失时, 云台不立即停, 而是朝最后方向
继续转一段时间去找(无目标追踪), 超时或重新捕获即终止。核心状态机做成
模块级纯函数 coast_step, 便于主机侧测试。

原理: 从 k230/main.py 中用 AST 提取 coast_step 函数源码, 在 stub time 环境里
执行, 逐条断言状态迁移表:
  - last_dx 未达门槛 → idle(常规停转)
  - 首次丢失且 |last_dx| >= 门槛 → start(启动预判, 记录启动时刻)
  - 预判中且未超时 → run(继续续追; 启动时刻保持不变)
  - 预判中且到点 → stop(停转, 并将 last_dx 清零以防重复触发)
  - stop 之后继续丢靶 → idle(不会反复重启, 直到下次真正捕获目标刷新 last_dx)

运行: python tools/test_coast_logic.py
说明: 纯主机侧运行, 不需要 K230 硬件; 不依赖任何第三方库(stdlib only)。
"""
import ast
import os
import types
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))


# ---------------------------------------------------------------------------
# 从 main.py 提取 coast_step, 并在 stub 环境里执行
# ---------------------------------------------------------------------------

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


def load_coast_step():
    """加载并返回 main.py 里的 coast_step 函数"""
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    fn_src = _extract_func(src, "coast_step")
    env = {"time": _FakeTime()}
    exec(fn_src, env)
    return env["coast_step"]


class TestCoastStep(unittest.TestCase):
    EDGE = 160      # 与 config [motor].coast_edge_px 默认一致
    MS = 800        # 与 config [motor].coast_ms 默认一致

    def setUp(self):
        self.fn = load_coast_step()

    def step(self, now, active, start, last_dx):
        return self.fn(now, active, start, last_dx, self.EDGE, self.MS)

    # ---- 触发条件: |last_dx| 与门槛 ----

    def test_idle_when_last_dx_small(self):
        """最后偏差在画面中部(未达门槛) → 不启动, last_dx 保留"""
        mode, active, start, last = self.step(1000, False, 0, 100)
        self.assertEqual(mode, "idle")
        self.assertFalse(active)
        self.assertEqual(last, 100)

    def test_idle_when_last_dx_zero(self):
        """从未捕获过目标(last_dx=0) → 不启动"""
        mode, active, _, _ = self.step(1000, False, 0, 0)
        self.assertEqual(mode, "idle")
        self.assertFalse(active)

    def test_start_when_at_edge_positive(self):
        """最后偏差在右侧边缘 → 启动, 启动时刻=当前"""
        mode, active, start, last = self.step(5000, False, 0, 200)
        self.assertEqual(mode, "start")
        self.assertTrue(active)
        self.assertEqual(start, 5000)
        self.assertEqual(last, 200)

    def test_start_when_at_edge_negative(self):
        """左侧边缘(负偏差)同样触发"""
        mode, active, start, last = self.step(5000, False, 0, -200)
        self.assertEqual(mode, "start")
        self.assertTrue(active)
        self.assertEqual(last, -200)

    def test_start_at_exact_edge_boundary(self):
        """|last_dx| == 门槛 → 触发(含等于)"""
        mode, _, _, _ = self.step(5000, False, 0, 160)
        self.assertEqual(mode, "start")
        mode, _, _, _ = self.step(5000, False, 0, -160)
        self.assertEqual(mode, "start")

    def test_idle_just_below_edge(self):
        """|last_dx| = 门槛-1 → 不触发"""
        mode, _, _, _ = self.step(5000, False, 0, 159)
        self.assertEqual(mode, "idle")

    # ---- 续追与超时 ----

    def test_run_within_duration(self):
        """预判中, 未到时长 → run, 启动时刻保持不变"""
        mode, active, start, last = self.step(10799, True, 10000, 200)
        self.assertEqual(mode, "run")
        self.assertTrue(active)
        self.assertEqual(start, 10000)
        self.assertEqual(last, 200)

    def test_stop_at_duration_boundary(self):
        """预判中, 恰好到时长 → stop(到点即停), last_dx 清零"""
        mode, active, start, last = self.step(10800, True, 10000, 200)
        self.assertEqual(mode, "stop")
        self.assertFalse(active)
        self.assertEqual(start, 0)
        self.assertEqual(last, 0)

    def test_no_restart_after_stop(self):
        """超时停止后, 继续丢靶(active=False, last_dx=0) → idle, 不会反复重启"""
        _, active, start, last = self.step(10800, True, 10000, 200)   # 这一帧 stop
        mode2, active2, _, last2 = self.step(10900, active, start, last)
        self.assertEqual(mode2, "idle")
        self.assertFalse(active2)
        self.assertEqual(last2, 0)

    # ---- 完整时序(启动→续追→到点停止→保持静止) ----

    def test_full_sequence(self):
        now, active, start, last = 10000, False, 0, 250
        mode, active, start, last = self.step(now, active, start, last)
        self.assertEqual(mode, "start")          # t+0:   启动
        now += 300
        mode, active, start, last = self.step(now, active, start, last)
        self.assertEqual(mode, "run")            # t+300: 续追
        now += 499
        mode, active, start, last = self.step(now, active, start, last)
        self.assertEqual(mode, "run")            # t+799: 仍在窗口内
        now += 1
        mode, active, start, last = self.step(now, active, start, last)
        self.assertEqual(mode, "stop")           # t+800: 到点停
        mode, active, start, last = self.step(now + 100, active, start, last)
        self.assertEqual(mode, "idle")           # 之后: 保持静止


if __name__ == "__main__":
    unittest.main(verbosity=2)
