"""采集端守门测试 —— 防止「采集初始化」被重构改回旧写法（画面几何回归）。

背景（实机多轮迭代, 方向/比例/视野/占屏须同时满足, 勿回退）:
  V7.6: 「画面偏左」→ 显式 CSI id=CSI_ID（config.camera.csi_id=2, 勿再丢）+ 提示改 1280x720。
  V7.7b: 1280x720 提示缺裁切 → 16:9 源配 4:3 输出被非等比缩放「横向压缩」→ 曾改回 1280x960。
  V7.7c: 1280x960 实机「偏左」复发（该模式本身视野偏左）→ 720 + set_framesize(crop=True)
         （居中裁 4:3 再缩）同时保方向与比例。
  V7.7d: 4:3 输出只用 720 中央 75% 视野, 观感「放大」→ 输出改 640x360（与 720 提示同 16:9,
         全幅最大视野）; crop=True 保留作防呆。
  V7.7f: 1:1 显示致画面观感变小（用户: "不是把屏幕缩小, 是让摄像头照出更多"）→ scale_view:
         显示前将采集帧等比放大满屏（640x360→800x450 满宽, 上下各 15px）, 视野不变、占屏最大。
  事实: 庐山派板载 GC2093 挂在 CSI2; 支持 1280x960@90 与 1280x720@90 两种模式。

本测试（AST 静态检查 + config 读值 + 纯函数提取, stdlib only, 不需要 K230 硬件）:
  1. main.py 的 Sensor(...) 构造必须显式传 id=(不得依赖固件默认端口);
  2. 该构造的探测提示分辨率必须是 1280x720(16:9, 实测方向正);
  3. set_framesize 必须带 crop=True（防呆: 输出比例若变 4:3, 居中裁切防非等比压缩）;
  4. main.py 顶层必须定义 CSI_ID(缺定义则 Sensor(id=CSI_ID) 生产会 NameError);
  5. config.json 的 camera.csi_id 必须存在且为 2(配置与代码一致);
  6. config 几何链: 输出宽高比 = 16:9(全幅视野) / center=输出尺寸一半 / 1:1 偏移居中;
  7. view_geom 纯函数行为（scale 开 = 等比放大先满宽; 关 = 1:1 居中）;
  8. config display.scale_view=true（画面填屏: 保持"照出更多"的同时显示不缩小）。
若确需变更这些值: 先理解画面几何影响（实机观察方向/比例/视野/占屏）, 再同步修改本测试
—— 不要静默改掉守门。

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


def _framesize_kwargs():
    tree = _load_main_tree()
    calls = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "set_framesize"):
            calls.append(node)
    assert len(calls) == 1, "k230/main.py 应恰好有 1 处 set_framesize(...), 实际 %d 处" % len(calls)
    return {k.arg: k.value for k in calls[0].keywords}


def _load_view_geom():
    """提取并执行 main.py 的 view_geom 纯函数（无外部依赖）"""
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "view_geom":
            env = {}
            exec(ast.get_source_segment(src, node), env)
            return env["view_geom"]
    raise AssertionError("k230/main.py 中未找到函数 view_geom")


class TestCameraInit(unittest.TestCase):
    """采集初始化守门（防回归: 缺 id / 960 模式偏左 / 16:9 无裁切压缩）"""

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

    def test_sensor_probe_resolution_is_1280x720(self):
        kw = _sensor_kwargs()
        for name, want in (("width", 1280), ("height", 720)):
            self.assertIn(name, kw, "Sensor(...) 缺少 %s= 参数" % name)
            self.assertIsInstance(
                kw[name], ast.Constant,
                "Sensor(%s=...) 应为字面量常量" % name)
            self.assertEqual(
                kw[name].value, want,
                "采集提示分辨率应为 1280x720(16:9, 实测方向正); 1280x960(4:3) 实测视野偏左, "
                "比例问题由 set_framesize(crop=True) 解决")

    def test_framesize_uses_center_crop(self):
        """16:9 提示→4:3 输出必须 crop=True 居中裁切, 否则非等比缩放横向压缩"""
        kw = _framesize_kwargs()
        self.assertIn(
            "crop", kw,
            "set_framesize 必须带 crop=True: 16:9 源配 4:3 输出时居中裁切再缩放, 缺则横向压缩")
        self.assertIsInstance(kw["crop"], ast.Constant,
                              "set_framesize(crop=...) 应为字面量 True")
        self.assertIs(kw["crop"].value, True,
                      "set_framesize(crop=...) 必须为 True")

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


class TestCameraConfigChain(unittest.TestCase):
    """config 几何链守门: 输出宽高比 / 追踪中心 / 显示居中 三者联动。

    约定: 输出须与 1280x720 提示同为 16:9（全幅最大视野; 4:3 输出会触发 crop 只剩中央
    75%, 观感"放大"）; center = 输出尺寸/2（改分辨率必须同步, 否则追踪停偏）; 画面垂直居中。
    """

    def _cfg(self):
        with open(_CFG_JSON, encoding="utf-8") as f:
            return json.load(f)

    def test_output_aspect_16x9_full_fov(self):
        cam = self._cfg().get("camera", {})
        w, h = cam.get("sensor_width"), cam.get("sensor_height")
        self.assertIsNotNone(w, "config.camera.sensor_width 缺失")
        self.assertIsNotNone(h, "config.camera.sensor_height 缺失")
        self.assertEqual(
            w * 720, h * 1280,
            "输出 %sx%s 宽高比 != 16:9（与 1280x720 提示一致才全幅、不裁切）; "
            "4:3 输出会触发 crop 只取中央 75%%（观感放大）" % (w, h))

    def test_center_equals_half_output(self):
        cfg = self._cfg()
        cam, ctr = cfg.get("camera", {}), cfg.get("center", {})
        self.assertEqual(ctr.get("x"), cam.get("sensor_width") // 2,
                         "center.x 须 = sensor_width/2（改分辨率必须同步）")
        self.assertEqual(ctr.get("y"), cam.get("sensor_height") // 2,
                         "center.y 须 = sensor_height/2（改分辨率必须同步）")

    def test_display_y_offset_centers_frame(self):
        cfg = self._cfg()
        cam, disp = cfg.get("camera", {}), cfg.get("display", {})
        want = (disp.get("lcd_height") - cam.get("sensor_height")) // 2
        self.assertEqual(
            disp.get("y_offset"), want,
            "画面垂直居中: y_offset 须 = (lcd_height - sensor_height)/2")

    def test_display_x_offset_centers_frame(self):
        cfg = self._cfg()
        cam, disp = cfg.get("camera", {}), cfg.get("display", {})
        want = (disp.get("lcd_width") - cam.get("sensor_width")) // 2
        self.assertEqual(
            disp.get("x_offset"), want,
            "画面水平居中: x_offset 须 = (lcd_width - sensor_width)/2")

    def test_scale_view_enabled(self):
        disp = self._cfg().get("display", {})
        self.assertIs(
            disp.get("scale_view"), True,
            "display.scale_view 应为 true: 采集帧等比放大填屏(照出更多的同时显示不缩小)")


class TestViewGeom(unittest.TestCase):
    """显示视图几何（纯函数 view_geom）:
    scale 开 = 等比放大（先满宽, 放不下则满高）; 关 = 1:1 居中。"""

    def test_scale_on_16x9(self):
        self.assertEqual(_load_view_geom()(True, 800, 480, 640, 360), (800, 450, 0, 15))

    def test_scale_on_wide_frame_falls_back_to_fit_height(self):
        # 4:3 帧满宽会超高(500>480) → 退化为满高适配
        self.assertEqual(_load_view_geom()(True, 800, 480, 640, 400), (768, 480, 16, 0))

    def test_scale_off_centers_1to1(self):
        self.assertEqual(_load_view_geom()(False, 800, 480, 640, 360), (640, 360, 80, 60))

    def test_scale_on_1280_source(self):
        self.assertEqual(_load_view_geom()(True, 800, 480, 1280, 720), (800, 450, 0, 15))


if __name__ == "__main__":
    unittest.main(verbosity=2)
