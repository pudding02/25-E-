"""采集端守门测试 —— 防止「采集初始化」被重构改回旧写法（画面视野偏移回归）。

背景（V7.6 修复 · 实机对照官方例程）:
  现象: 画面视野偏移(偏左)。与官方例程 官方代码/rgb888_find_circles_1.py 同板对照,
        官方例程画面正常、本程序画面偏 —— 差异在采集端:
  官方: Sensor(id=2, width=1280, height=720, fps=90)   <- 显式 CSI 端口 + 16:9 模式提示
  旧版: Sensor(width=1280, height=960, fps=FPS)        <- v5(36b9e38) 重构丢 id, 且提示 4:3 模式
  事实: 庐山派板载 GC2093 挂在 CSI2; GC2093 为 16:9 传感器, 应用 16:9 模式提示
        (README 第三节 YOLO 规则同款约束: 不建议对 GC2093 设 4:3 分辨率)。

本测试（AST 静态检查, stdlib only, 不需要 K230 硬件）:
  1. main.py 的 Sensor(...) 构造必须显式传 id=(不得依赖固件默认端口);
  2. 该构造的探测提示分辨率必须是 1280x720(16:9);
  3. main.py 顶层必须定义 CSI_ID(缺定义则 Sensor(id=CSI_ID) 生产会 NameError);
  4. config.json 的 camera.csi_id 必须存在且为 2(配置与代码一致)。
若确需变更这些值: 先理解画面几何影响(对照 官方代码/rgb888_find_circles_1.py 实机验证),
再同步修改本测试 —— 不要静默改掉守门。

运行: python tools/test_camera_init.py
"""
import ast
import json
import os
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))
_CFG_JSON = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "config.json"))


def _load_main_tree():
    with open(_MAIN_PY, encoding="utf-8") as f:
        return ast.parse(f.read())


def _find_sensor_calls(tree):
    """模块内所有 Sensor(...) 构造调用"""
    calls = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == "Sensor"):
            calls.append(node)
    return calls


def _sensor_kwargs():
    calls = _find_sensor_calls(_load_main_tree())
    assert len(calls) == 1, "k230/main.py 应恰好有 1 处 Sensor(...) 构造, 实际 %d 处" % len(calls)
    return {k.arg: k.value for k in calls[0].keywords}


class TestCameraInit(unittest.TestCase):
    """采集初始化守门（防回归: 缺 id / 4:3 提示 曾导致画面视野偏移）"""

    def test_sensor_call_exists_once(self):
        calls = _find_sensor_calls(_load_main_tree())
        self.assertEqual(
            len(calls), 1,
            "k230/main.py 应恰好有 1 处 Sensor(...) 构造(采集端), 实际 %d 处" % len(calls))

    def test_sensor_call_has_explicit_id(self):
        kw = _sensor_kwargs()
        self.assertIn(
            "id", kw,
            "Sensor(...) 必须显式传 id=(取 config.camera.csi_id); 缺省会依赖固件默认端口, "
            "曾导致采集视野偏移")
        v = kw["id"]
        self.assertFalse(
            isinstance(v, ast.Constant) and v.value is None,
            "Sensor(id=None) 等于没传 —— 必须指定 CSI 端口")

    def test_sensor_probe_resolution_is_16x9(self):
        kw = _sensor_kwargs()
        for name, want in (("width", 1280), ("height", 720)):
            self.assertIn(name, kw, "Sensor(...) 缺少 %s= 参数" % name)
            self.assertIsInstance(
                kw[name], ast.Constant,
                "Sensor(%s=...) 应为字面量常量(与官方例程一致)" % name)
            self.assertEqual(
                kw[name].value, want,
                "采集探测提示分辨率应为 1280x720(16:9); 旧 1280x960(4:3) 曾致画面视野偏移")

    def test_csi_id_defined_in_main(self):
        tree = _load_main_tree()
        defined = set()
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for tgt in node.targets:
                    if isinstance(tgt, ast.Name):
                        defined.add(tgt.id)
        self.assertIn(
            "CSI_ID", defined,
            "k230/main.py 顶层必须定义 CSI_ID(从 config.camera.csi_id 读取); "
            "缺失则 Sensor(id=CSI_ID) 生产会 NameError")

    def test_config_csi_id(self):
        with open(_CFG_JSON, encoding="utf-8") as f:
            cfg = json.load(f)
        self.assertEqual(
            cfg.get("camera", {}).get("csi_id"), 2,
            "k230/config.json 的 camera.csi_id 应为 2(庐山派板载 GC2093 在 CSI2)")


if __name__ == "__main__":
    unittest.main(verbosity=2)
