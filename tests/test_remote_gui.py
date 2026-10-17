"""Remote-source GUI lifecycle checks without Tk, ROS, SSH, or a camera."""
import ast
import queue
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import calib_gui


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


def load_app_class():
    """Extract the nested App class while avoiding Tk construction in main()."""
    tree = ast.parse((HERE / "calib_gui.py").read_text())
    main = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == "main")
    app_node = next(node for node in main.body
                    if isinstance(node, ast.ClassDef) and node.name == "App")
    namespace = dict(calib_gui.__dict__)
    exec(compile(ast.Module(body=[app_node], type_ignores=[]),
                 str(HERE / "calib_gui.py"), "exec"), namespace)
    return namespace["App"], namespace


class RemoteGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.App, cls.namespace = load_app_class()

    def make_app(self):
        app = self.App.__new__(self.App)
        app.source = "ssh"
        app.source_var = Value("本地")
        app.lock = calib_gui.threading.Lock()
        app.state = dict(generation=4, bgr="old", gray="old", corners="old",
                         disp_rgb="old", ncorner=12, stamp=10.0,
                         native=(640, 480), decode_err="old", image_encoding="rgb8",
                         maps=None, size=(11, 8), square=.025)
        app._native_size = (640, 480)
        app._last_stamp = 10.0
        app._rate = [1.0]
        app._last_corner_ts = 1.0
        app._pub_warn = "old warning"
        app._img_item = None
        app._photo = None
        app.img_canvas = mock.Mock()
        app.remote = None
        app.remote_thread = None
        app.cam = None
        app.driver_proc = None
        app.driver_identity = {"status": "confirmed", "devices": [{"serial": "OLD"}]}
        app.driver_env = {"stale": True}
        app.ssh_password = ""
        app.remember_ssh_password = Value(False)
        app.ssh_host_var = Value("old-camera")
        app.ssh_port = 22
        app.ssh_key = "old-key"
        app.topic = "/camera/image_raw"
        app.topic_var = Value(app.topic)
        app.driver = "hik"
        app.drv_var = Value("Hik")
        app.remote_dir = "~/old-calib"
        app.remote_camera_root = "/old/cameras"
        app.btn_start = mock.Mock()
        app.btn_stop = mock.Mock()
        app.log = mock.Mock()
        app.root = mock.Mock()
        app.vars = {"sw": Value(11), "sh": Value(8), "sq": Value(25)}
        app.samples = ["old sample"]
        app.sample_params = ["old params"]
        app.sample_bytes = 99
        app._cap_warned = True
        app.res = {"old": True}
        app.result_samples = ["old result sample"]
        app.session = "old session"
        app.mapx = app.mapy = app._maps = "old map"
        app._last_corners = "old corners"
        app._last_param_sent = {}
        app.checked = app.saved = True
        app.res_var = Value("old result")
        app._calib_busy = app._save_busy = False
        return app

    def test_start_hint_distinguishes_local_ros_topic_from_ssh_device_ownership(self):
        local = calib_gui.source_start_hint("local", "/camera/image_raw")
        remote = calib_gui.source_start_hint("ssh", "/camera/image_raw")

        self.assertIn("本机同一 ROS 域", local)
        self.assertIn("/camera/image_raw", local)
        self.assertIn("仅在其他软件中打开相机，不等于发布了此话题", local)
        self.assertIn("先停止远端占用相机的 ACE", remote)
        self.assertIn("ACE 内部画面不会自动接入本工具", remote)

    def test_apply_ssh_profile_stops_old_connection_clears_samples_and_honors_busy(self):
        app = self.make_app()
        app.source = "local"
        app.source_var = Value("本地")
        old_connection = mock.Mock()
        old_thread = mock.Mock()
        processor = mock.Mock()
        processor.is_alive.return_value = False
        app.remote = old_connection
        app.remote_thread = old_thread
        app.cam = processor
        app._busy = mock.Mock(return_value=False)
        app.vars = {"sw": Value(11), "sh": Value(8), "sq": Value(25)}
        profile = {
            "host": "new-camera", "port": 2202, "identity_file": "/keys/new",
            "remote_dir": "~/calib new", "camera_root": "/opt/cameras",
            "driver": "mv", "topic": "/new/image_raw", "password": "test-secret",
        }

        self.assertTrue(app._apply_ssh_profile(profile))

        old_connection.stop.assert_called_once_with()
        old_thread.join.assert_called_once_with(timeout=1.5)
        processor.stop.assert_called()
        self.assertIsNone(app.remote)
        self.assertIsNone(app.remote_thread)
        self.assertIsNone(app.cam)
        self.assertEqual(app.source, "ssh")
        self.assertEqual(app.source_var.get(), "SSH 远程")
        self.assertEqual(app.ssh_host_var.get(), "new-camera")
        self.assertEqual(app.ssh_port, 2202)
        self.assertEqual(app.ssh_key, "/keys/new")
        self.assertEqual(app.ssh_password, "test-secret")
        self.assertTrue(app.remember_ssh_password.get())
        self.assertEqual(app.remote_dir, "~/calib new")
        self.assertEqual(app.remote_camera_root, "/opt/cameras")
        self.assertEqual(app.driver, "mv")
        self.assertEqual(app.topic, "/new/image_raw")
        self.assertEqual(app.topic_var.get(), "/new/image_raw")
        self.assertEqual(app.samples, [])
        self.assertEqual(app.sample_params, [])
        self.assertEqual(app.sample_bytes, 0)
        self.assertIsNone(app.driver_identity)
        self.assertIsNone(app.driver_env)

        before = (app.source, app.ssh_host_var.get(), app.ssh_port,
                  app.topic, list(app.samples), app.state["generation"])
        app._busy.return_value = True
        changed = dict(profile, host="must-not-apply", port=2222, password="other")
        self.assertFalse(app._apply_ssh_profile(changed))
        self.assertEqual((app.source, app.ssh_host_var.get(), app.ssh_port,
                          app.topic, list(app.samples), app.state["generation"]), before)

    def test_switching_from_remote_stops_workers_clears_samples_and_starts_local_ros(self):
        app = self.make_app()
        connection = mock.Mock()
        remote_thread = mock.Mock()
        processor = mock.Mock()
        processor.is_alive.return_value = False
        app.remote = connection
        app.remote_thread = remote_thread
        app.cam = processor

        class LocalCamera:
            def __init__(self, topic, state, lock):
                self.args = (topic, state, lock)
                self.started = False

            def start(self):
                self.started = True

        fake_rclpy = SimpleNamespace(ok=mock.Mock(return_value=False), init=mock.Mock())
        with mock.patch.dict(sys.modules, {"rclpy": fake_rclpy}), \
                mock.patch.dict(self.namespace, {"CameraThread": LocalCamera}):
            app.on_source_change()

        connection.stop.assert_called_once_with()
        remote_thread.join.assert_called_once_with(timeout=1.5)
        processor.stop.assert_called()
        self.assertIsNone(app.remote)
        self.assertIsNone(app.remote_thread)
        self.assertEqual(app.source, "local")
        self.assertEqual(app.samples, [])
        self.assertEqual(app.sample_params, [])
        self.assertIsNone(app.res)
        self.assertIsNone(app.camera_identity)
        self.assertIsNone(app.driver_identity)
        self.assertIsNone(app.driver_env)
        self.assertEqual(app.state["generation"], 7)
        for key in ("bgr", "gray", "corners", "disp_rgb", "native", "decode_err"):
            self.assertIsNone(app.state[key], key)
        fake_rclpy.init.assert_called_once_with()
        self.assertIsInstance(app.cam, LocalCamera)
        self.assertTrue(app.cam.started)

    def test_remote_worker_callbacks_are_scoped_to_connection_and_disconnect_clears_frame(self):
        app = self.make_app()
        app.source = "ssh"
        app.source_var = Value("SSH 远程")
        app.ssh_host_var = Value("camera-box")
        app.ssh_port = 2222
        app.ssh_key = "/local/key"
        app.remote_dir = "~/camera_calib"
        app.remote_camera_root = "/opt/cameras"
        app.topic_var = Value("/remote/image_raw")
        app.apply_size = mock.Mock()

        image = SimpleNamespace(encoding="rgb8", height=1, width=1, step=3, data=b"\x01\x02\x03")
        identity = {"status": "confirmed", "devices": [{"serial": "REMOTE-1"}]}
        seen = {}

        class FakeConnection:
            def __init__(self, **kwargs):
                seen["config"] = kwargs

            def run(self, on_image, on_identity, log):
                seen["on_image"] = on_image
                seen["on_identity"] = on_identity
                on_identity(identity)
                on_image(image)

        class FakeProcessor:
            def __init__(self, topic, state, lock):
                self.received = []

            def on_image(self, msg):
                self.received.append(msg)
                with app.lock:
                    app.state["bgr"] = "fresh frame"
                    app.state["gray"] = "fresh gray"

        class InlineThread:
            def __init__(self, target, daemon):
                self.target = target
                self.daemon = daemon

            def start(self):
                self.target()

        remote_module = SimpleNamespace(RemoteConnection=FakeConnection)
        with mock.patch.dict(sys.modules, {"remote_camera": remote_module}), \
                mock.patch.dict(self.namespace, {
                    "CameraThread": FakeProcessor,
                    "threading": SimpleNamespace(Thread=InlineThread),
                }):
            app._start_remote_camera()

        self.assertEqual(seen["config"], {
            "host": "camera-box", "port": 2222, "identity_file": "/local/key",
            "remote_dir": "~/camera_calib", "driver": "hik",
            "topic": "/remote/image_raw", "camera_root": "/opt/cameras", "fps": 0,
            "password": "",
        })
        self.assertEqual(app.cam.received, [image])
        self.assertIs(app.driver_identity, identity)
        # The worker's finally block invalidates the last frame after a disconnect.
        self.assertEqual(app.state["generation"], 6)
        self.assertIsNone(app.state["bgr"])
        self.assertIsNone(app.state["gray"])
        self.assertIsNone(app.state["corners"])
        self.assertIsNone(app.state["disp_rgb"])
        # A callback from an obsolete connection cannot repopulate the preview.
        app.remote = None
        seen["on_image"](image)
        seen["on_identity"]({"status": "confirmed", "devices": [{"serial": "STALE"}]})
        self.assertEqual(app.cam.received, [image])
        self.assertIs(app.driver_identity, identity)

    def test_remote_identity_fallback_never_reads_local_usb_identity(self):
        app = self.make_app()
        app.source = "ssh"
        app.camera_identity = None
        app.driver_identity = None
        app.state["image_encoding"] = "rgb8"
        read_local = mock.Mock(side_effect=AssertionError("local sysfs must not be read"))
        with mock.patch.dict(self.namespace, {"read_camera_identity": read_local}):
            app._remember_camera_identity()
        read_local.assert_not_called()
        self.assertEqual(app.camera_identity, {
            "status": "unavailable", "devices": [], "image_encoding": "rgb8",
        })

    def test_ssh_grab_requests_original_sample_and_rejects_stale_generation(self):
        app = self.make_app()
        app.samples = []
        app.sample_bytes = 0
        app.max_samples = 10
        app.remote_sampleq = queue.Queue()
        app._sample_pending = None
        app._busy = mock.Mock(return_value=False)
        app._driver_running = mock.Mock(return_value=True)
        app.state["size"] = (11, 8)
        app.state["gray"] = np.zeros((3, 4), dtype=np.uint8)  # lossy preview's gray image
        app.state["corners"] = "preview corners"
        sample_gray = np.full((3, 4), 173, dtype=np.uint8)
        sample_corners = np.array([[[1., 2.]], [[3., 4.]]], dtype=np.float32)
        callback_holder = {}

        class FakeConnection:
            def request_sample(self, callback):
                callback_holder["callback"] = callback

        app.remote = FakeConnection()
        app.grab_one()
        self.assertEqual(app.samples, [], "SSH grab must not sample JPEG preview state")
        self.assertIsNotNone(app._sample_pending)
        self.assertIn("callback", callback_holder)
        requested_generation = app.state["generation"]

        class InlineThread:
            def __init__(self, target, **_kwargs):
                self.target = target

            def start(self):
                self.target()

        detect = mock.Mock(return_value=sample_corners)
        msg = SimpleNamespace(encoding="mono8", height=3, width=4, data=sample_gray.tobytes())
        app.state["generation"] = requested_generation + 1  # source/config changed while in flight
        with mock.patch.dict(self.namespace, {
                "threading": SimpleNamespace(Thread=InlineThread),
                "detect_preview": detect,
        }):
            callback_holder["callback"](msg)

        detected_gray, detected_board = detect.call_args.args
        np.testing.assert_array_equal(detected_gray, sample_gray)
        self.assertEqual(detected_board, (11, 8))
        app._poll_remote_sample()
        self.assertEqual(app.samples, [], "late sample from an old generation must be rejected")
        self.assertIsNone(app._sample_pending)


if __name__ == "__main__":
    unittest.main()
