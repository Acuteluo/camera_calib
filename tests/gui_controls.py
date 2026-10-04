#!/usr/bin/env python3
"""真实 Tk 窗口验证：缺模板启动、只读日志、点击导出失败及成功。需合成相机。"""
import json
import re
from pathlib import Path
import subprocess
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import ace_json
import calib_gui

out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
captured = {}
errors = []
checked = []
failures = []
save_heartbeats = []
selection = {"path": str(out / "exported_uav.json")}
original_button = ttk.Button
original_tk = tk.Tk


def button_factory(*args, **kwargs):
    button = original_button(*args, **kwargs)
    if kwargs.get("text") == "导出 ACE json":
        captured["app"] = kwargs["command"].__self__
        captured["button"] = button
    return button


def poll(root):
    app = captured.get("app")
    if not app or not app.saved:
        root.after(500, lambda: poll(root))
        return
    try:
        if captured.get("stage") == "recalibrate":
            assert app.session != captured["first_session"]
            assert (Path(captured["first_session"]) / "camera_info.yaml").read_bytes() == captured["first_yaml"]
            assert len(app.result_samples) == len(app.samples)
            assert len(save_heartbeats) >= 2, save_heartbeats
            checked.append("重新标定不覆盖旧目录；后台保存时 GUI 仍响应")
            app._refresh()
            app._drain_log()
            root.update_idletasks()
            try:
                subprocess.run(["xwd", "-silent", "-id", str(root.winfo_id()),
                                "-out", str(out / "gui.xwd")], timeout=8, check=True)
            except Exception:
                pass
            app.quit()
            return
        app._drain_log()
        text = app.log_text.get("1.0", "end-1c")
        assert "ACE 参考模板缺失" in text, text
        assert "已输出到" in text
        assert str(app.log_text.cget("state")) == "disabled"
        app.log_text.insert("end", "unauthorized edit")
        app.log_text.delete("1.0", "end")
        assert app.log_text.get("1.0", "end-1c") == text
        checked.append("日志自动更新且禁止输入、删除")

        captured["button"].invoke()
        assert errors and "不存在" in errors[-1][1], errors
        assert not (out / "exported_uav.json").exists()
        checked.append("缺模板时启动/标定/保存正常；点击导出明确报错")

        app.ace_template = str(HERE / "reference_json" / "uav.json")
        reference_before = Path(app.ace_template).read_bytes()
        selection["path"] = app.ace_template
        captured["button"].invoke()
        assert "不能覆盖参考模板" in errors[-1][1]
        assert Path(app.ace_template).read_bytes() == reference_before
        selection["path"] = str(out / "nonexistent-directory" / "export.json")
        captured["button"].invoke()
        assert "错误：" in errors[-1][1]
        selection["path"] = str(out / "exported_uav.json")
        # After-calibration edits must not change the exported calibration metadata.
        app.vars["sq"].set(100)
        app.vars["sw"].set(9)
        captured["button"].invoke()
        text = (out / "exported_uav.json").read_text()
        assert re.search(r"8x6 内角点 / 25(?:\.0)? mm", text), text
        app.vars["sq"].set(25)
        app.vars["sw"].set(8)
        checked.append("保护参考模板、报告写入失败及参数编辑后的正确元数据")
        exported = json.loads(ace_json.strip_comments((out / "exported_uav.json").read_text()))
        template = json.loads(ace_json.strip_comments(Path(app.ace_template).read_text()))
        diffs = ace_json._leaf_diffs(template, exported)
        allowed = {"image_width", "image_height", "distortion_model", "camera_matrix",
                   "distortion_coefficients", "projection_matrix"}
        assert diffs and all(len(p) > 3 and p[0] == "cameras" and p[2] in allowed for p in diffs)
        assert abs(exported["cameras"]["camera_0"]["camera_matrix"]["data"][0] - app.res["K"][0, 0]) < 1e-4
        checked.append("点击导出使用项目 reference_json/uav.json，仅内参改变")
        captured["first_session"] = app.session
        captured["first_yaml"] = (Path(app.session) / "camera_info.yaml").read_bytes()
        captured["stage"] = "recalibrate"
        count = len(app.samples)
        app._demo_pending = True
        app.do_calibrate()
        assert app.res is None and app.session is None and not app.saved
        app.apply_size()   # Must be rejected while calibration is still running.
        assert len(app.samples) == count
        root.after(500, lambda: poll(root))
    except Exception as exc:
        import traceback
        failures.append(traceback.format_exc())
        app.quit()


def root_factory(*args, **kwargs):
    root = original_tk(*args, **kwargs)
    def heartbeat():
        app = captured.get("app")
        if app and getattr(app, "_save_busy", False):
            save_heartbeats.append(time.monotonic())
        root.after(20, heartbeat)
    root.after(20, heartbeat)
    root.after(500, lambda: poll(root))
    return root


ttk.Button = button_factory
tk.Tk = root_factory
filedialog.asksaveasfilename = lambda **kwargs: selection["path"]
messagebox.showerror = lambda title, message, **kwargs: errors.append((title, message))
sys.argv = ["calib_gui.py", "--demo", "--size", "8x6", "--square", "0.025",
            "--topic", "/calib_demo/image_raw", "--results-dir", str(out / "results"),
            "--ace-template", str(out / "missing-template.json"),
            "--demo-calibrate-after", "35", "--demo-exit-after", "75"]
result = calib_gui.main()
assert result == 0 and len(checked) == 5 and not failures, (result, checked, failures)
for item in checked:
    print("[通过] " + item)
