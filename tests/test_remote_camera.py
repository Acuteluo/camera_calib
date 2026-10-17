"""Offline tests for the SSH bridge protocol; no SSH server or camera is used."""
import io
import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import cv2
import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import remote_camera


def png_packet(rgb):
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".png", bgr, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise RuntimeError("could not encode synthetic test frame")
    return encoded.tobytes()


def protocol_packet(kind, payload, **extra):
    header = {"type": kind, "length": len(payload), **extra}
    return json.dumps(header, separators=(",", ":")).encode() + b"\n" + payload


class ShortReadStream(io.BytesIO):
    """A stream that may return fewer bytes than requested, as pipes can."""
    def read(self, size=-1):
        if size is not None and size > 3:
            size = 3
        return super().read(size)


class FakeProcess:
    def __init__(self, stdout_bytes):
        self.stdin = RecordingPipe()
        self.stdout = io.BytesIO(stdout_bytes)
        self.stderr = io.BytesIO()
        self.pid = 43210
        self.wait_calls = []

    def poll(self):
        return 0 if self.stdout.tell() >= len(self.stdout.getvalue()) else None

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        return 0

    def terminate(self):
        pass

    def kill(self):
        pass


class RecordingPipe(io.BytesIO):
    def close(self):
        self.contents = super().getvalue()
        super().close()

    def getvalue(self):
        if self.closed:
            return self.contents
        return super().getvalue()


class RemoteCameraTests(unittest.TestCase):
    def test_ssh_argv_and_remote_shell_keep_host_and_metacharacters_as_data(self):
        marker = None
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            marker = root / "shell-ran"
            remote_dir = root / "camera dir; touch shell-ran"
            remote_dir.mkdir()
            topic = "/image_raw; touch " + str(marker) + "; echo injected"
            camera_root = "/opt/camera; touch " + str(marker)
            command = remote_camera.build_remote_command(
                str(remote_dir), "hik", topic, 10, camera_root)

            # Replace only the remote Python entry point with a local argv dumper. The
            # generated shell string is otherwise evaluated exactly as SSH would evaluate it.
            code = "import json,sys; print(json.dumps(sys.argv[1:]))"
            inert_python = "/usr/bin/python3 -c " + shlex.quote(code)
            command = command.replace("/usr/bin/python3 -s", inert_python, 1)
            proc = subprocess.run(["bash", "-c", command], cwd="/", capture_output=True,
                                  text=True, timeout=5, check=True,
                                  env={**os.environ, "HOME": str(root)})
            remote_args = json.loads(proc.stdout)
            self.assertEqual(remote_args[0], "./remote_camera.py")
            self.assertEqual(remote_args[remote_args.index("--topic") + 1], topic)
            self.assertEqual(remote_args[remote_args.index("--camera-root") + 1], camera_root)
            self.assertFalse(marker.exists(), "shell metacharacters must stay in argument values")

        host = "lab;touch-shell-marker"
        argv = remote_camera.build_ssh_argv(
            host, port=2222, identity_file="~/.ssh/test key", remote_dir="~/camera_calib",
            driver="mv", topic="/camera/image_raw")
        self.assertEqual(argv[0], "ssh")
        self.assertIn("-p", argv)
        self.assertEqual(argv[argv.index("-p") + 1], "2222")
        self.assertEqual(argv[-2], host)
        self.assertEqual(argv[-1], remote_camera.build_remote_command(
            "~/camera_calib", "mv", "/camera/image_raw", 0, ""))

    def test_unlimited_fps_is_default_and_invalid_fps_values_are_rejected(self):
        self.assertEqual(remote_camera._parse_args(["--serve"]).fps, 0)
        self.assertEqual(remote_camera._parse_args(["--serve", "--fps", "0"]).fps, 0)
        argv = remote_camera.build_ssh_argv("camera-box")
        self.assertEqual(argv[-1], remote_camera.build_remote_command(
            ".", "hik", "/image_raw", 0, ""))
        # Explicit finite positive caps remain supported without the old 60 FPS ceiling.
        self.assertIn("--fps 120.0", remote_camera.build_remote_command(
            ".", "hik", "/image_raw", 120, ""))

        with mock.patch.object(remote_camera, "_RemoteService") as service, \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            for invalid in ("-1", "nan", "inf"):
                with self.subTest(fps=invalid):
                    self.assertEqual(remote_camera.main(["--serve", "--fps", invalid]), 2)
            service.assert_not_called()

    def test_ssh_rejects_option_injection_and_invalid_connection_values(self):
        for host in ("", "-oProxyCommand=touch /tmp/pwn", "camera box", "camera\nbox"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                remote_camera.build_ssh_argv(host)
        for port in (0, 65536, "not-a-port"):
            with self.subTest(port=port), self.assertRaises(ValueError):
                remote_camera.build_ssh_argv("camera-box", port=port)
        with self.assertRaises(ValueError):
            remote_camera.build_remote_command("/tmp/camera", "hik", "", 10)

    def test_camera_occupancy_hint_names_process_and_returns_none_when_unused(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            sysfs_root = root / "sysfs"
            proc_root = root / "proc"
            usb = sysfs_root / "1-2"
            usb.mkdir(parents=True)
            (usb / "busnum").write_text("1\n", encoding="utf-8")
            (usb / "devnum").write_text("7\n", encoding="utf-8")
            proc = proc_root / "2468"
            proc.mkdir(parents=True)
            (proc / "comm").write_text("ACE\n", encoding="utf-8")
            identity = {"devices": [{"usb_port": "1-2"}]}
            fuser = mock.Mock(return_value=SimpleNamespace(stdout="2468\n", returncode=0))

            with mock.patch("calib_gui.read_camera_identity", return_value=identity) as read_identity, \
                    mock.patch.object(remote_camera.subprocess, "run", fuser):
                hint = remote_camera.camera_occupancy_hint(
                    "hik", sysfs_root=str(sysfs_root), proc_root=str(proc_root))
                self.assertIn("相机被占用：ACE（PID 2468）", hint)
                self.assertIn("停止占用程序", hint)
                read_identity.assert_called_once_with("hik", sysfs_root=str(sysfs_root))
                fuser.assert_called_once_with(
                    ["fuser", "/dev/bus/usb/001/007"],
                    capture_output=True, text=True, timeout=1)

                fuser.return_value = SimpleNamespace(stdout="", returncode=1)
                self.assertIsNone(remote_camera.camera_occupancy_hint(
                    "hik", sysfs_root=str(sysfs_root), proc_root=str(proc_root)))

    def test_password_uses_sshpass_environment_and_never_enters_argv(self):
        password = "pw ; $(touch should-not-run)"
        with mock.patch.object(remote_camera.shutil, "which", return_value="/usr/bin/sshpass") as which:
            conn = remote_camera.RemoteConnection("camera-box", password=password)
        which.assert_called_once_with("sshpass")
        self.assertEqual(conn.argv[:2], ["sshpass", "-e"])
        self.assertIn("BatchMode=no", conn.argv)
        self.assertNotIn("BatchMode=yes", conn.argv)
        self.assertNotIn(password, conn.argv)
        self.assertEqual(conn._ssh_env["SSHPASS"], password)
        self.assertEqual(remote_camera.ssh_environment(password)["SSHPASS"], password)
        self.assertTrue(all(password not in str(value)
                            for key, value in conn._ssh_env.items() if key != "SSHPASS"))

    def test_png_decode_preserves_every_rgb_pixel_and_protocol_dimensions(self):
        rgb = np.array([
            [[255, 0, 0], [0, 255, 0], [0, 0, 255], [7, 17, 29]],
            [[13, 19, 23], [41, 43, 47], [101, 103, 107], [251, 253, 255]],
            [[0, 0, 0], [255, 255, 255], [128, 64, 32], [5, 250, 125]],
        ], dtype=np.uint8)
        payload = png_packet(rgb)
        msg = remote_camera.RemoteConnection("camera-box")._decode_image(
            {"width": rgb.shape[1], "height": rgb.shape[0]}, payload)
        self.assertEqual((msg.width, msg.height, msg.step, msg.encoding), (4, 3, 12, "rgb8"))
        np.testing.assert_array_equal(
            np.frombuffer(msg.data, dtype=np.uint8).reshape(rgb.shape), rgb)
        with self.assertRaisesRegex(ValueError, "尺寸与协议头不一致"):
            remote_camera.RemoteConnection("camera-box")._decode_image(
                {"width": 5, "height": 3}, payload)

    def test_reader_handles_fragmented_png_payload(self):
        rgb = np.arange(5 * 7 * 3, dtype=np.uint8).reshape(5, 7, 3)
        payload = png_packet(rgb)
        header = json.dumps({"type": "image", "length": len(payload),
                             "width": 7, "height": 5, "topic": "/image_raw"}).encode() + b"\n"
        conn = remote_camera.RemoteConnection("camera-box")
        conn._events = queue.Queue()
        conn._process = type("Process", (), {"stdout": ShortReadStream(header + payload)})()
        conn._stdout_reader()
        self.assertEqual(conn._frames.qsize(), 1)
        frame = conn._frames.get_nowait()
        self.assertEqual(frame[1], payload)
        decoded = conn._decode_image(*frame)
        np.testing.assert_array_equal(
            np.frombuffer(decoded.data, dtype=np.uint8).reshape(rgb.shape), rgb)

    def test_reader_retains_only_the_latest_frame_during_a_burst(self):
        frames = []
        for seq, value in enumerate((17, 83, 241), start=1):
            rgb = np.full((2, 3, 3), value, dtype=np.uint8)
            payload = png_packet(rgb)
            header = json.dumps({"type": "image", "length": len(payload),
                                 "width": 3, "height": 2, "seq": seq,
                                 "topic": "/image_raw"}).encode() + b"\n"
            frames.append(header + payload)
        conn = remote_camera.RemoteConnection("camera-box")
        conn._process = type("Process", (), {"stdout": io.BytesIO(b"".join(frames))})()
        conn._stdout_reader()
        self.assertEqual(conn._frames.qsize(), 1)
        header, payload = conn._frames.get_nowait()
        self.assertEqual(header["seq"], 3)
        msg = conn._decode_image(header, payload)
        self.assertTrue(np.all(np.frombuffer(msg.data, dtype=np.uint8) == 241))

    def test_reader_drops_frames_from_the_previous_topic(self):
        packets = []
        for topic, seq in (("/old/image", 1), ("/new/image", 2)):
            payload = png_packet(np.full((2, 2, 3), seq, dtype=np.uint8))
            header = json.dumps({"type": "image", "length": len(payload),
                                 "width": 2, "height": 2, "seq": seq,
                                 "topic": topic}).encode() + b"\n"
            packets.append(header + payload)
        conn = remote_camera.RemoteConnection("camera-box", topic="/old/image")
        conn.set_topic("/new/image")
        conn._process = type("Process", (), {"stdout": io.BytesIO(b"".join(packets))})()
        conn._stdout_reader()
        header, payload = conn._frames.get_nowait()
        self.assertEqual(header["seq"], 2)
        self.assertEqual(conn._commands.get_nowait(), {
            "cmd": "set_topic", "topic": "/new/image",
        })
        msg = conn._decode_image(header, payload)
        self.assertTrue(np.all(np.frombuffer(msg.data, dtype=np.uint8) == 2))

    def test_run_dispatches_framed_image_and_closes_pipes_after_eof(self):
        rgb = np.array([[[3, 7, 11], [101, 151, 201]],
                        [[13, 17, 19], [211, 223, 227]]], dtype=np.uint8)
        payload = png_packet(rgb)
        packet = (json.dumps({"type": "image", "length": len(payload),
                              "width": 2, "height": 2,
                              "topic": "/image_raw"}).encode() + b"\n" + payload)
        process = FakeProcess(packet)
        conn = remote_camera.RemoteConnection("camera-box")
        images, identities, logs = [], [], []
        with mock.patch.object(remote_camera.subprocess, "Popen", return_value=process) as popen:
            conn.run(images.append, identities.append, logs.append)

        popen.assert_called_once_with(conn.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, bufsize=0, start_new_session=True,
                                      env=conn._ssh_env)
        self.assertEqual(len(images), 1)
        np.testing.assert_array_equal(
            np.frombuffer(images[0].data, dtype=np.uint8).reshape(rgb.shape), rgb)
        self.assertEqual(process.stdin.getvalue(), b'{"cmd":"shutdown"}\n')
        self.assertTrue(process.stdin.closed)
        self.assertTrue(process.stdout.closed)
        self.assertTrue(process.stderr.closed)
        self.assertEqual(process.wait_calls, [1.0])
        self.assertIsNone(conn._process)

    def test_stop_is_nonblocking_and_leaves_pipe_writes_to_run_thread(self):
        entered = threading.Event()
        release = threading.Event()

        class BlockingPipe:
            def write(self, _data):
                entered.set()
                release.wait(timeout=2)

        conn = remote_camera.RemoteConnection("camera-box")
        conn._process = SimpleNamespace(stdin=BlockingPipe())
        worker = threading.Thread(target=conn.stop, daemon=True)
        worker.start()
        worker.join(timeout=0.25)
        try:
            self.assertFalse(worker.is_alive(), "stop() must return without touching SSH stdin")
            self.assertTrue(conn._stop.is_set())
            self.assertFalse(entered.is_set())
        finally:
            release.set()
            worker.join(timeout=0.5)

    def test_remote_sample_is_exact_gray_png_while_preview_is_jpeg(self):
        rng = np.random.default_rng(20261017)
        rgb = rng.integers(0, 256, (24, 32, 3), dtype=np.uint8)
        topic = "/camera/image_raw"
        service = remote_camera._RemoteService(SimpleNamespace(topic=topic, fps=10))
        service.topic = topic
        service.latest_rgb = rgb
        service.latest_seq = 23
        service.sample_requests.put({"id": 987, "topic": topic})
        sent = []

        def capture(kind, data=None, payload=None, extra=None, stop_event=None):
            sent.append((kind, data, payload, dict(extra or {})))

        with mock.patch.object(remote_camera, "_wire_send", side_effect=capture), \
                mock.patch.object(remote_camera.time, "sleep",
                                  side_effect=lambda _delay: service.stop.set()):
            service._send_frames()

        sample = next(item for item in sent if item[0] == "sample")
        preview = next(item for item in sent if item[0] == "image")
        self.assertEqual(sample[3]["topic"], topic)
        self.assertEqual(sample[3]["id"], 987)
        self.assertEqual(sample[3]["encoding"], "mono8")
        self.assertTrue(sample[2].startswith(b"\x89PNG\r\n\x1a\n"))
        gray = cv2.imdecode(np.frombuffer(sample[2], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        expected_gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        np.testing.assert_array_equal(gray, expected_gray)

        self.assertEqual(preview[3]["format"], "jpeg")
        self.assertEqual(preview[3]["topic"], topic)
        self.assertTrue(preview[2].startswith(b"\xff\xd8"))
        preview_bgr = cv2.imdecode(np.frombuffer(preview[2], dtype=np.uint8), cv2.IMREAD_COLOR)
        preview_rgb = cv2.cvtColor(preview_bgr, cv2.COLOR_BGR2RGB)
        self.assertFalse(np.array_equal(preview_rgb, rgb), "the lossy preview is not a sample")

        # Feed the actual server-produced packets through the client parser and decoder.
        conn = remote_camera.RemoteConnection("camera-box", topic=topic)
        sample_callbacks, previews = [], []
        conn.request_sample(lambda msg, error=None: sample_callbacks.append((msg, error)))
        sample[3]["id"] = conn._pending_sample[0]
        packet = b"".join(protocol_packet(kind, payload, **extra)
                          for kind, _data, payload, extra in sent)
        process = FakeProcess(packet)
        with mock.patch.object(remote_camera.subprocess, "Popen", return_value=process):
            conn.run(previews.append, lambda _identity: None, lambda _message: None)

        self.assertEqual(len(sample_callbacks), 1)
        sample_msg, error = sample_callbacks[0]
        self.assertIsNone(error)
        self.assertEqual((sample_msg.encoding, sample_msg.width, sample_msg.height,
                          sample_msg.step), ("mono8", 32, 24, 32))
        np.testing.assert_array_equal(
            np.frombuffer(sample_msg.data, dtype=np.uint8).reshape(expected_gray.shape),
            expected_gray)
        self.assertEqual(len(previews), 1)
        self.assertEqual(previews[0].encoding, "rgb8")
        np.testing.assert_array_equal(
            np.frombuffer(previews[0].data, dtype=np.uint8).reshape(rgb.shape), preview_rgb)
        self.assertIn(b'"cmd":"sample"', process.stdin.getvalue())

    def test_topic_switch_cancels_old_sample_and_late_response_cannot_complete_new_one(self):
        conn = remote_camera.RemoteConnection("camera-box", topic="/old/image")
        old_results, new_results = [], []
        conn.request_sample(lambda msg, error=None: old_results.append((msg, error)))
        old_id = conn._pending_sample[0]
        conn.set_topic("/new/image")
        self.assertEqual(len(old_results), 1)
        self.assertIsNone(old_results[0][0])
        self.assertIn("话题已切换", old_results[0][1])

        conn.request_sample(lambda msg, error=None: new_results.append((msg, error)))
        new_id = conn._pending_sample[0]
        old_gray = np.full((3, 4), 19, dtype=np.uint8)
        new_gray = np.full((3, 4), 211, dtype=np.uint8)
        ok, old_encoded = cv2.imencode(".png", old_gray)
        self.assertTrue(ok)
        ok, new_encoded = cv2.imencode(".png", new_gray)
        self.assertTrue(ok)
        packet = protocol_packet("sample", old_encoded.tobytes(), id=old_id,
                                 topic="/old/image", width=4, height=3, encoding="mono8")
        packet += protocol_packet("sample", new_encoded.tobytes(), id=new_id,
                                  topic="/new/image", width=4, height=3, encoding="mono8")
        process = FakeProcess(packet)
        with mock.patch.object(remote_camera.subprocess, "Popen", return_value=process):
            conn.run(lambda _msg: None, lambda _identity: None, lambda _message: None)

        self.assertEqual(len(old_results), 1, "topic change settles the old request only once")
        self.assertEqual(len(new_results), 1)
        new_msg, error = new_results[0]
        self.assertIsNone(error)
        self.assertEqual(new_msg.encoding, "mono8")
        np.testing.assert_array_equal(
            np.frombuffer(new_msg.data, dtype=np.uint8).reshape(3, 4), new_gray)

    def test_disconnect_reports_sample_error_instead_of_returning_preview(self):
        conn = remote_camera.RemoteConnection("camera-box")
        results = []
        conn.request_sample(lambda msg, error=None: results.append((msg, error)))
        process = FakeProcess(b"")
        with mock.patch.object(remote_camera.subprocess, "Popen", return_value=process):
            conn.run(lambda _msg: self.fail("no preview should be emitted"),
                     lambda _identity: None, lambda _message: None)
        self.assertEqual(len(results), 1)
        self.assertIsNone(results[0][0])
        self.assertIn("连接已结束", results[0][1])


if __name__ == "__main__":
    unittest.main()
