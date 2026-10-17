#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SSH bridge for a remote ROS camera. The stdout protocol carries JSON headers, JPEG previews and lossless PNG samples."""

import argparse
import json
import math
import os
import queue
import re
import shlex
import shutil
import signal
import select
import subprocess
import sys
import threading
import time
from types import SimpleNamespace


MAX_HEADER_BYTES = 64 * 1024
MAX_CONTROL_BYTES = 16 * 1024
MAX_PACKET_BYTES = 128 * 1024 * 1024
MAX_IMAGE_PIXELS = 50_000_000
DRIVER_NAMES = ("hik", "galaxy", "mv")
DRIVER_SELECTED_RE = re.compile(
    r"CALIB_SELECTED_CAMERA\s+serial=(\S+)\s+transport=(\S+)\s+model=(.+)"
)


def _quote_remote_path(path):
    """Quote a remote path while allowing a leading ~/ to expand on the remote host."""
    path = str(path)
    if path == "~":
        return '"$HOME"'
    if path.startswith("~/"):
        return '"$HOME"/' + shlex.quote(path[2:])
    return shlex.quote(path)


def build_remote_command(remote_dir, driver, topic, fps, camera_root=""):
    """Build the single remote shell command passed as ssh's final argv item."""
    if driver not in DRIVER_NAMES:
        raise ValueError("driver 必须是 hik、galaxy 或 mv")
    if not isinstance(topic, str) or not topic.strip():
        raise ValueError("topic 不能为空")
    fps = float(fps)
    if not math.isfinite(fps) or fps < 0:
        raise ValueError("fps 必须是非负有限数，0 表示不限速")
    remote_dir = str(remote_dir).strip()
    if not remote_dir:
        raise ValueError("remote_dir 不能为空")
    args = [
        "--serve", "--driver", driver, "--topic", topic.strip(), "--fps", str(fps),
    ]
    if camera_root:
        args.extend(("--camera-root", str(camera_root)))
    words = ["/usr/bin/python3", "-s", "./remote_camera.py", *args]
    quoted = []
    expand_next_path = False
    for i, word in enumerate(words):
        if i == 2 or expand_next_path:
            quoted.append(_quote_remote_path(word))
            expand_next_path = False
        else:
            quoted.append(shlex.quote(word))
            expand_next_path = word == "--camera-root"
    return "cd -- {} && exec {}".format(
        _quote_remote_path(remote_dir),
        " ".join(quoted),
    )


def build_ssh_argv(host, port=22, identity_file=None, remote_dir=".", driver="hik",
                   topic="/image_raw", camera_root="", fps=0, password=None):
    """Return an argv-safe SSH invocation (host may be an ssh_config alias)."""
    host = str(host).strip()
    if not host or host.startswith("-") or any(c.isspace() or ord(c) < 32 for c in host):
        raise ValueError("SSH 主机不能为空、不能以 - 开头，也不能含空白字符")
    try:
        port = int(port)
    except (TypeError, ValueError):
        raise ValueError("SSH 端口必须是 1 到 65535 的整数")
    if not 1 <= port <= 65535:
        raise ValueError("SSH 端口必须是 1 到 65535 的整数")

    if password and not shutil.which("sshpass"):
        raise ValueError("密码登录需要本机安装 sshpass，也可使用 SSH 密钥或 ssh-agent")
    argv = (["sshpass", "-e"] if password else []) + [
        "ssh", "-T", "-o", "BatchMode=no" if password else "BatchMode=yes", "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3",
    ]
    if identity_file:
        key = os.path.abspath(os.path.expanduser(str(identity_file)))
        argv.extend(("-i", key, "-o", "IdentitiesOnly=yes"))
    if port != 22:
        argv.extend(("-p", str(port)))
    argv.extend((host, build_remote_command(remote_dir, driver, topic, fps, camera_root)))
    return argv


def ssh_environment(password=None):
    """Pass an optional password in the child environment, never in argv or a file."""
    env = os.environ.copy()
    if password:
        env["SSHPASS"] = password
    else:
        env.pop("SSHPASS", None)
    return env


class RemoteConnection:
    """Run an SSH camera session. ``run`` is blocking and is intended for a worker thread."""

    def __init__(self, host, port=22, identity_file=None, remote_dir=".", driver="hik",
                 topic="/image_raw", camera_root="", fps=0, password=None):
        self.argv = build_ssh_argv(host, port, identity_file, remote_dir, driver, topic,
                                   camera_root, fps, password=password)
        self._ssh_env = ssh_environment(password)
        self.host = str(host).strip()
        self.port = int(port)
        self.identity_file = identity_file
        self.remote_dir = remote_dir
        self.driver = driver
        self.topic = topic
        self.camera_root = camera_root
        self.fps = float(fps)
        self._commands = queue.Queue()
        self._events = queue.Queue(maxsize=128)
        self._logs = queue.Queue(maxsize=256)
        self._frames = queue.Queue(maxsize=1)
        self._protocol_ended = threading.Event()
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._process = None
        self._stdin_closed = False
        self._log = None
        self._sample_lock = threading.Lock()
        self._pending_sample = None

    def request_sample(self, callback):
        """Fetch a new lossless gray frame; callback(msg, error) runs on the worker."""
        with self._sample_lock:
            if self._stop.is_set():
                raise RuntimeError("SSH 连接已停止")
            if self._pending_sample is not None:
                raise RuntimeError("上一张原图仍在传输")
            token = time.monotonic_ns()
            self._pending_sample = (token, callback, time.monotonic() + 15, self.topic)
        self._commands.put({"cmd": "sample", "id": token, "topic": self.topic})

    def _finish_sample(self, token=None, msg=None, error=None):
        with self._sample_lock:
            pending = self._pending_sample
            if pending is None or (token is not None and token != pending[0]):
                return
            self._pending_sample = None
        if pending[3] != self.topic:
            msg, error = None, "话题已切换，已丢弃旧话题样本"
        try:
            pending[1](msg, error)
        except Exception as exc:
            self._report("处理原图采集结果失败：%s" % exc)

    def set_param(self, node, name, value):
        """Queue a remote ROS parameter update; safe to call from the GUI thread."""
        if self._stop.is_set():
            return
        if hasattr(value, "item"):
            value = value.item()
        self._commands.put({"cmd": "set_param", "node": str(node), "name": str(name),
                            "value": value})

    def set_topic(self, topic):
        """Queue a subscription change; the remote service drops the previous topic's frame."""
        topic = str(topic).strip()
        if not topic:
            self._report("SSH 话题不能为空，未切换")
            return
        if not self._stop.is_set():
            self.topic = topic
            self._finish_sample(error="话题已切换，已取消原图采集")
            self._commands.put({"cmd": "set_topic", "topic": topic})

    def stop(self):
        """Request graceful remote cleanup. Safe to call from another thread."""
        self._stop.set()

    def _report(self, message):
        if self._log:
            try:
                self._log(str(message))
            except Exception:
                pass

    def _send_shutdown(self):
        with self._write_lock:
            if self._stdin_closed or not self._process or not self._process.stdin:
                return
            try:
                self._process.stdin.write(b'{"cmd":"shutdown"}\n')
                self._process.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                pass
            try:
                self._process.stdin.close()
            except (OSError, ValueError):
                pass
            self._stdin_closed = True

    def _queue_event(self, event):
        try:
            self._events.put_nowait(event)
        except queue.Full:
            if event[0] == "eof":
                self._protocol_ended.set()
            elif event[0] in ("identity", "error", "sample", "sample_error"):
                try:
                    dropped = self._events.get_nowait()
                    if dropped[0] == "eof":
                        self._protocol_ended.set()
                    self._events.put_nowait(event)
                except (queue.Empty, queue.Full):
                    pass

    def _queue_log(self, message):
        try:
            self._logs.put_nowait(str(message))
        except queue.Full:
            pass

    def _send_command(self, command):
        try:
            wire = (json.dumps(command, ensure_ascii=False, allow_nan=False,
                               separators=(",", ":")) + "\n").encode("utf-8")
        except (TypeError, ValueError) as exc:
            self._report("SSH 命令参数无效：%s" % exc)
            return
        if len(wire) > MAX_CONTROL_BYTES:
            self._report("SSH 命令超过协议大小限制，已丢弃")
            return
        with self._write_lock:
            if self._stdin_closed or not self._process or not self._process.stdin:
                return
            try:
                self._process.stdin.write(wire)
                self._process.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                self._queue_event(("eof", None))

    @staticmethod
    def _read_exact(stream, count):
        chunks = []
        remain = count
        while remain:
            chunk = stream.read(remain)
            if not chunk:
                return None
            chunks.append(chunk)
            remain -= len(chunk)
        return b"".join(chunks)

    def _stdout_reader(self):
        stream = self._process.stdout
        try:
            while not self._stop.is_set():
                line = stream.readline(MAX_HEADER_BYTES + 1)
                if not line:
                    self._queue_event(("eof", None))
                    return
                if len(line) > MAX_HEADER_BYTES or not line.endswith(b"\n"):
                    self._queue_log("远端协议头超过限制或未结束，连接已关闭")
                    self._queue_event(("eof", None))
                    return
                try:
                    header = json.loads(line.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    # Some remote shells print a startup banner to stdout. Ignore a small line,
                    # report it, then continue until the first valid protocol header.
                    self._queue_log("远端启动输出：" + line.decode("utf-8", "replace").rstrip())
                    continue
                if not isinstance(header, dict):
                    self._queue_log("忽略格式错误的远端协议头")
                    continue
                kind = header.get("type")
                if kind in ("image", "sample"):
                    length = header.get("length")
                    width, height = header.get("width"), header.get("height")
                    if (not isinstance(length, int) or length <= 0 or length > MAX_PACKET_BYTES or
                            not isinstance(width, int) or not isinstance(height, int) or
                            width <= 0 or height <= 0 or width * height > MAX_IMAGE_PIXELS):
                        self._queue_log("远端图像尺寸或数据长度超过协议限制")
                        self._queue_event(("eof", None))
                        return
                    payload = self._read_exact(stream, length)
                    if payload is None:
                        self._queue_event(("eof", None))
                        return
                    if kind == "sample":
                        self._queue_event(("sample", (header, payload)))
                        continue
                    if header.get("topic") != self.topic:
                        continue
                    # Bound pending memory to one full-resolution frame; newer frames replace old.
                    try:
                        self._frames.put_nowait((header, payload))
                    except queue.Full:
                        try:
                            self._frames.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            self._frames.put_nowait((header, payload))
                        except queue.Full:
                            pass
                elif kind == "heartbeat":
                    continue
                elif kind in ("identity", "error", "sample_error"):
                    try:
                        data = header.get("data", header)
                        if kind == "identity" and (not isinstance(data, dict) or
                                                    not isinstance(data.get("devices", []), list)):
                            self._queue_log("忽略格式错误的远端设备身份")
                        elif kind == "error" and not isinstance(data, dict):
                            self._queue_log("忽略格式错误的远端错误消息")
                        else:
                            self._queue_event((kind, data))
                    except Exception:
                        self._queue_log("忽略格式错误的远端控制消息")
                else:
                    self._queue_log("收到未知远端消息：%s" % kind)
        except (OSError, ValueError) as exc:
            self._queue_log("读取 SSH 图像流失败：%s" % exc)
            self._queue_event(("eof", None))

    def _stderr_reader(self):
        try:
            for raw in self._process.stderr:
                line = raw.decode("utf-8", "replace").rstrip()
                if line:
                    self._queue_log(line)
        except (OSError, ValueError):
            pass

    def _decode_image(self, header, payload):
        import cv2
        import numpy as np

        encoded = np.frombuffer(payload, dtype=np.uint8)
        if header.get("type") == "sample":
            if not payload.startswith(b"\x89PNG\r\n\x1a\n") or header.get("encoding") != "mono8":
                raise ValueError("标定样本必须是无损灰度 PNG")
            gray = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
            if (gray is None or gray.ndim != 2 or gray.dtype != np.uint8 or
                    gray.shape != (header["height"], header["width"])):
                raise ValueError("无损样本尺寸或编码错误")
            return SimpleNamespace(encoding="mono8", width=gray.shape[1], height=gray.shape[0],
                                   step=gray.shape[1], data=gray.tobytes())
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("远端 PNG 无法解码")
        height, width = bgr.shape[:2]
        if (width, height) != (header["width"], header["height"]):
            raise ValueError("远端 PNG 像素尺寸与协议头不一致")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return SimpleNamespace(encoding="rgb8", height=height, width=width,
                               step=width * 3, data=rgb.tobytes())

    def run(self, on_image, on_identity, log):
        """Connect and dispatch callbacks until SSH ends or ``stop`` is called."""
        self._log = log
        try:
            self._process = subprocess.Popen(
                self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                bufsize=0, start_new_session=True, env=self._ssh_env,
            )
        except OSError as exc:
            self._report("无法启动 SSH：%s" % exc)
            return
        readers = [
            threading.Thread(target=self._stdout_reader, name="remote-camera-stdout", daemon=True),
            threading.Thread(target=self._stderr_reader, name="remote-camera-stderr", daemon=True),
        ]
        for thread in readers:
            thread.start()
        eof = False
        try:
            while not self._stop.is_set() and not eof:
                try:
                    kind, data = self._events.get(timeout=0.05)
                except queue.Empty:
                    kind = None
                if kind == "identity":
                    try:
                        on_identity(data)
                    except Exception as exc:
                        self._report("处理远端相机身份失败：%s" % exc)
                elif kind == "sample":
                    header, payload = data
                    try:
                        self._finish_sample(header.get("id"), self._decode_image(header, payload))
                    except Exception as exc:
                        self._finish_sample(header.get("id"), error=str(exc))
                elif kind == "sample_error":
                    self._finish_sample(data.get("id"), error=data.get("message", "原图采集失败"))
                elif kind == "heartbeat":
                    pass
                elif kind == "error":
                    self._report(str(data.get("message", data)))
                elif kind == "log":
                    self._report(data)
                elif kind == "eof":
                    eof = True
                try:
                    frame = self._frames.get_nowait()
                except queue.Empty:
                    frame = None
                if frame is not None and frame[0].get("topic") == self.topic:
                    try:
                        on_image(self._decode_image(*frame))
                    except Exception as exc:
                        self._report("处理远端图像失败：%s" % exc)
                while not self._stop.is_set():
                    try:
                        command = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    self._send_command(command)
                try:
                    self._report(self._logs.get_nowait())
                except queue.Empty:
                    pass
                with self._sample_lock:
                    pending = self._pending_sample
                if pending is not None and time.monotonic() >= pending[2]:
                    self._finish_sample(pending[0], error="原图传输超时")
                if self._protocol_ended.is_set() or (self._process.poll() is not None and
                                                       self._events.empty()):
                    eof = True
        finally:
            self._finish_sample(error="SSH 连接已结束")
            self._send_shutdown()
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._terminate_process_group()
            for thread in readers:
                thread.join(timeout=0.2)
            for pipe in (self._process.stdin, self._process.stdout, self._process.stderr):
                try:
                    pipe.close()
                except (AttributeError, OSError, ValueError):
                    pass
            self._process = None

    def _terminate_process_group(self):
        proc = self._process
        if not proc:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.terminate()
            except OSError:
                pass
        try:
            proc.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                try:
                    proc.kill()
                except OSError:
                    pass
            try:
                proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                pass


def _write_stdout(data, stop_event=None, timeout=20.0):
    """Write a protocol chunk with a deadline while remaining responsive to shutdown."""
    fd = sys.stdout.fileno()
    was_blocking = os.get_blocking(fd)
    if was_blocking:
        os.set_blocking(fd, False)
    view = memoryview(data)
    deadline = time.monotonic() + timeout
    try:
        while view:
            if stop_event is not None and stop_event.is_set():
                raise InterruptedError("远端相机服务正在退出")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("SSH 图像流写入超时")
            try:
                _, ready, _ = select.select([], [fd], [], min(0.1, remaining))
            except InterruptedError:
                continue
            if not ready:
                continue
            try:
                written = os.write(fd, view)
            except BlockingIOError:
                continue
            if written <= 0:
                raise BrokenPipeError("SSH 图像流已断开")
            view = view[written:]
    finally:
        if was_blocking:
            try:
                os.set_blocking(fd, True)
            except OSError:
                pass


def _wire_send(kind, data=None, payload=None, extra=None, stop_event=None):
    header = {"type": kind}
    if data is not None:
        header["data"] = data
    if extra:
        header.update(extra)
    if payload is not None:
        if len(payload) > MAX_PACKET_BYTES:
            raise ValueError("PNG 超过协议大小限制")
        header["length"] = len(payload)
    line = (json.dumps(header, ensure_ascii=False,
                       separators=(",", ":")) + "\n").encode("utf-8")
    _write_stdout(line, stop_event=stop_event, timeout=5.0)
    if payload is not None:
        _write_stdout(payload, stop_event=stop_event,
                      timeout=max(5.0, min(30.0, len(payload) / (1024 * 1024))))


def _emit_remote_identity(driver, serial, transport, model):
    """Build an identity snapshot only on the remote camera host and match its driver log."""
    from calib_gui import read_camera_identity

    serial = "" if serial == "unknown" else serial
    if not serial:
        # A host-side USB scan cannot prove which device the driver opened without its serial.
        return {"status": "unavailable", "devices": []}
    detected = read_camera_identity(driver)
    device = next((dict(d) for d in detected.get("devices", [])
                   if d.get("serial") == serial), {})
    device.update(serial=serial, model=model.strip(), transport=transport)
    return {"status": "confirmed", "devices": [device]}


def camera_occupancy_hint(driver, sysfs_root="/sys/bus/usb/devices", proc_root="/proc"):
    """Only identify an owner when one matching USB camera can be located."""
    from pathlib import Path
    from calib_gui import read_camera_identity
    try:
        devices = read_camera_identity(driver, sysfs_root=sysfs_root).get("devices", [])
        if len(devices) != 1:
            return None
        base = Path(sysfs_root) / devices[0]["usb_port"]
        bus = int((base / "busnum").read_text().strip())
        number = int((base / "devnum").read_text().strip())
        result = subprocess.run(["fuser", "/dev/bus/usb/%03d/%03d" % (bus, number)],
                                capture_output=True, text=True, timeout=1)
        owners = []
        for value in result.stdout.split():
            if not value.isdigit():
                continue
            pid = int(value)
            try:
                name = (Path(proc_root) / str(pid) / "comm").read_text().strip()
            except OSError:
                name = "其他进程"
            owners.append("%s（PID %d）" % (name, pid))
        if owners:
            return "相机被占用：%s。请先停止占用程序，再点【启动驱动】。" % "、".join(owners)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError):
        pass
    return None


def _source_remote_environment(remote_dir, driver, camera_root):
    """Source the repository/ROS/driver overlays exactly as the local GUI does."""
    from driver_runtime import build_drivers, sdk_environment

    from driver_runtime import default_camera_root
    root = os.path.expanduser(camera_root or os.environ.get("CAMERA_ROOT") or
                              default_camera_root())
    d = build_drivers(root)[driver]
    base_setup = os.path.join(d["ws"], "install", "setup.bash")
    overlay = os.path.join(remote_dir, ".driver_runtime", driver, "install", "setup.bash")
    py = "import json,os;print(json.dumps(dict(os.environ),ensure_ascii=False))"
    commands = [
        "set -e",
        "cd -- {}".format(_quote_remote_path(remote_dir)),
        "source ./runtime.sh",
        "calib_base_env",
        "calib_ros_env",
        "if [[ -f {} ]]; then source {}; fi".format(
            shlex.quote(base_setup), shlex.quote(base_setup)),
        "if [[ -f {} ]]; then source {}; fi".format(
            shlex.quote(overlay), shlex.quote(overlay)),
        "exec /usr/bin/python3 -s -c {}".format(shlex.quote(py)),
    ]
    result = subprocess.run(["bash", "-c", "; ".join(commands)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=30)
    if result.returncode:
        raise RuntimeError("准备远端 ROS 环境失败：%s" % result.stderr.strip())
    try:
        # Keep the final JSON line robust to harmless output from sourced setup hooks.
        env = json.loads(result.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError) as exc:
        raise RuntimeError("远端 ROS 环境没有返回有效变量：%s" % result.stdout[-500:]) from exc
    return sdk_environment(driver, env), d


class _RemoteService:
    def __init__(self, args):
        self.args = args
        self.stop = threading.Event()
        self.commands = queue.Queue()
        self.parameter_commands = queue.Queue()
        self.events = queue.Queue(maxsize=16)
        self.frame_lock = threading.Lock()
        self.latest_rgb = None
        self.sample_requests = queue.Queue(maxsize=1)
        self.latest_seq = 0
        self.driver_proc = None
        self.stdin_thread = None
        self.driver_log_thread = None
        self.node = None
        self.executor = None
        self.sub = None
        self.topic = args.topic
        self.env = None
        self.driver = None

    @staticmethod
    def _log(message):
        try:
            print(message, file=sys.stderr, flush=True)
        except (BrokenPipeError, OSError, ValueError):
            # SSH may close stderr while the service is shutting down. Logging must
            # never prevent the driver cleanup path from running.
            pass

    def _driver_log(self):
        proc = self.driver_proc
        fd = proc.stdout.fileno()
        pending = bytearray()
        try:
            while True:
                try:
                    readable, _, _ = select.select([fd], [], [], 0.1)
                except InterruptedError:
                    continue
                if not readable:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except InterruptedError:
                    continue
                if not chunk:
                    break
                pending.extend(chunk)
                while True:
                    newline = pending.find(b"\n")
                    if newline < 0:
                        break
                    line = bytes(pending[:newline])
                    del pending[:newline + 1]
                    self._handle_driver_line(line)
                if len(pending) > MAX_HEADER_BYTES:
                    self._handle_driver_line(bytes(pending[-MAX_HEADER_BYTES:]))
                    pending.clear()
            if pending:
                self._handle_driver_line(bytes(pending))
        except (OSError, ValueError) as exc:
            self._log("读取驱动日志失败：%s" % exc)
        finally:
            self._post_event(("driver_exit", proc.poll()))

    def _handle_driver_line(self, raw_line):
        line = raw_line.decode("utf-8", "replace").rstrip()
        match = DRIVER_SELECTED_RE.search(line)
        if match:
            serial, transport, model = match.groups()
            try:
                identity = _emit_remote_identity(self.args.driver, serial, transport, model)
                self._post_event(("identity", identity))
            except Exception as exc:
                self._log("读取远端相机身份失败：%s" % exc)
        if re.search(r"(OpenDevice|GXOpenDevice|CameraInit).*?(failed|error|失败)", line, re.I):
            hint = camera_occupancy_hint(self.args.driver)
            if hint:
                self._log(hint)
                self._post_event(("fatal", hint))
        if line:
            self._log("驱动| " + line[-500:])

    def _post_event(self, event):
        try:
            self.events.put_nowait(event)
        except queue.Full:
            if event[0] in ("identity", "driver_exit", "fatal"):
                try:
                    self.events.get_nowait()
                    self.events.put_nowait(event)
                except (queue.Empty, queue.Full):
                    pass

    def _start_driver(self):
        from driver_runtime import runtime_setup

        self.env, self.driver = _source_remote_environment(
            self.args.remote_dir, self.args.driver, self.args.camera_root)
        if self.stop.is_set():
            return
        if os.environ.get("CAMERA_CALIB_REMOTE_ENV_READY") != "1":
            # ROS Python extensions and SDK libraries read the dynamic loader path at exec.
            # Re-enter once with the sourced environment before importing rclpy/cv2.
            self.env["CAMERA_CALIB_REMOTE_ENV_READY"] = "1"
            argv = [sys.executable, "-s", os.path.abspath(__file__), *sys.argv[1:]]
            os.execve(sys.executable, argv, self.env)
        if not os.path.isdir(self.driver["ws"]):
            raise RuntimeError("远端驱动工作区不存在：%s" % self.driver["ws"])
        if not runtime_setup(self.args.driver) and not os.path.isfile(
                os.path.join(self.driver["ws"], "install", "setup.bash")):
            raise RuntimeError("远端驱动尚未编译：%s/install/setup.bash 不存在" % self.driver["ws"])
        command = ["ros2", "run", self.driver["pkg"], self.driver["exe"]]
        if runtime_setup(self.args.driver):
            camera_info = os.path.join(self.driver["ws"], "config", "camera_info.yaml")
            command.extend(("--ros-args", "-p", "camera_info_url:=file://" + camera_info))
            serial = os.environ.get("CALIB_CAMERA_SERIAL", "").strip()
            if serial:
                command.extend(("-p", "camera_serial:=" + serial))
        self._log("启动远端驱动：%s" % self.args.driver)
        self.driver_proc = subprocess.Popen(
            command, cwd=self.args.remote_dir, env=self.env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0,
            start_new_session=True,
        )
        self.driver_log_thread = threading.Thread(
            target=self._driver_log, name="remote-camera-driver-log", daemon=True)
        self.driver_log_thread.start()

    def _subscribe(self, topic, generation):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        def on_image(msg):
            if generation != self._generation:
                return
            try:
                from ros_img import to_rgb
                rgb = to_rgb(msg)
            except Exception as exc:
                self._log("ROS 图像转换失败：%s" % exc)
                return
            with self.frame_lock:
                if generation != self._generation:
                    return
                self.latest_rgb = rgb
                self.latest_seq += 1

        self.topic = topic
        self.sub = self.node.create_subscription(
            Image, topic, on_image, qos_profile_sensor_data)

    def _ros_loop(self):
        from rclpy.executors import SingleThreadedExecutor

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        next_publish_check = time.monotonic() + 0.5
        try:
            while not self.stop.is_set():
                while True:
                    try:
                        command = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    if command.get("cmd") == "shutdown":
                        self.stop.set()
                        break
                    if command.get("cmd") == "set_topic":
                        topic = str(command.get("topic", "")).strip()
                        if topic:
                            with self.frame_lock:
                                self._generation += 1
                                generation = self._generation
                                self.latest_rgb = None
                            if self.sub is not None:
                                self.node.destroy_subscription(self.sub)
                            self._subscribe(topic, generation)
                            self._log("图像话题已切换：%s" % topic)
                    elif command.get("cmd") == "set_param":
                        self.parameter_commands.put(command)
                try:
                    self.executor.spin_once(timeout_sec=0.05)
                except Exception as exc:
                    if not self.stop.is_set():
                        self._log("ROS 图像订阅已退出：%s" % exc)
                    self.stop.set()
                now = time.monotonic()
                if now >= next_publish_check:
                    try:
                        count = self.node.count_publishers(self.topic)
                    except Exception:
                        count = 0
                    if count > 1:
                        self._post_event(("fatal", "话题 %s 有 %d 个发布者，无法确认画面来自同一相机" %
                                          (self.topic, count)))
                        return
                    next_publish_check = now + 1.0
        finally:
            try:
                self.executor.remove_node(self.node)
            except Exception:
                pass

    def _parameter_loop(self):
        while not self.stop.is_set():
            try:
                command = self.parameter_commands.get(timeout=0.1)
            except queue.Empty:
                continue
            if command.get("cmd") == "shutdown":
                return
            node = str(command.get("node", ""))
            name = str(command.get("name", ""))
            value = command.get("value")
            if not node or not name or value is None:
                self._log("忽略无效的 ROS 参数命令")
                continue
            try:
                result = subprocess.run(["ros2", "param", "set", node, name, str(value)],
                                        env=self.env, capture_output=True, text=True, timeout=10)
                output = (result.stdout + result.stderr).strip().splitlines()
                self._log("参数 %s/%s=%s：%s" % (
                    node, name, value, output[-1] if output else "完成"))
            except Exception as exc:
                self._log("设置远端参数 %s/%s 失败：%s" % (node, name, exc))

    def _stdin_loop(self):
        try:
            fd = sys.stdin.fileno()
            pending = bytearray()
            while not self.stop.is_set():
                try:
                    readable, _, _ = select.select([fd], [], [], 0.1)
                except InterruptedError:
                    continue
                if not readable:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except InterruptedError:
                    continue
                if not chunk:
                    if pending:
                        self._log("客户端控制消息未以换行结束")
                    self.commands.put({"cmd": "shutdown"})
                    self.stop.set()
                    return
                pending.extend(chunk)
                while True:
                    newline = pending.find(b"\n")
                    if newline < 0:
                        if len(pending) > MAX_CONTROL_BYTES:
                            self._log("客户端控制消息超过协议大小限制")
                            self.commands.put({"cmd": "shutdown"})
                            self.stop.set()
                            return
                        break
                    if newline + 1 > MAX_CONTROL_BYTES:
                        self._log("客户端控制消息超过协议大小限制")
                        self.commands.put({"cmd": "shutdown"})
                        self.stop.set()
                        return
                    line = bytes(pending[:newline])
                    del pending[:newline + 1]
                    try:
                        message = json.loads(line.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        self._log("忽略无效的客户端控制消息")
                        continue
                    if not isinstance(message, dict):
                        continue
                    if message.get("cmd") == "shutdown":
                        self.commands.put(message)
                        self.stop.set()
                        return
                    if message.get("cmd") == "set_topic":
                        topic = message.get("topic")
                        if isinstance(topic, str) and topic.strip():
                            self.commands.put({"cmd": "set_topic", "topic": topic.strip()})
                    elif message.get("cmd") == "sample":
                        try:
                            self.sample_requests.put_nowait(message)
                        except queue.Full:
                            pass
                    elif message.get("cmd") == "set_param":
                        if isinstance(message.get("node"), str) and isinstance(message.get("name"), str):
                            self.commands.put(message)
        except (OSError, ValueError) as exc:
            self._log("读取客户端控制消息失败：%s" % exc)
            self.commands.put({"cmd": "shutdown"})
            self.stop.set()

    def _start_ros(self):
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=None)
        self.node = Node("remote_camera_bridge")
        self._generation = 0
        self._subscribe(self.topic, self._generation)

    def _send_frames(self):
        import cv2

        delay = 1.0 / float(self.args.fps) if self.args.fps > 0 else 0.0
        next_frame = time.monotonic()
        last_seq = -1
        last_heartbeat = 0.0
        while not self.stop.is_set():
            try:
                event, data = self.events.get_nowait()
            except queue.Empty:
                event = None
                data = None
            if event == "identity":
                _wire_send("identity", data=data, stop_event=self.stop)
            elif event == "driver_exit":
                _wire_send("error", data={"message": "远端相机驱动已退出（退出码 %s）" % data},
                           stop_event=self.stop)
                return
            elif event == "fatal":
                _wire_send("error", data={"message": data}, stop_event=self.stop)
                return
            try:
                request = self.sample_requests.get_nowait()
            except queue.Empty:
                request = None
            if request is not None:
                with self.frame_lock:
                    raw, raw_topic = self.latest_rgb, self.topic
                if raw is None or request.get("topic") != raw_topic:
                    _wire_send("sample_error", data={"id": request.get("id"),
                        "message": "当前没有可采集的原图，或话题已切换"}, stop_event=self.stop)
                else:
                    # Grayscale conversion is exactly the one used for local calibration.
                    gray = cv2.cvtColor(raw, cv2.COLOR_RGB2GRAY)
                    ok, encoded = cv2.imencode(".png", gray, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                    if not ok:
                        raise RuntimeError("标定原图 PNG 编码失败")
                    _wire_send("sample", payload=encoded.tobytes(), stop_event=self.stop,
                        extra={"id": request.get("id"), "topic": raw_topic, "encoding": "mono8",
                               "width": gray.shape[1], "height": gray.shape[0]})
            now = time.monotonic()
            if now >= next_frame:
                with self.frame_lock:
                    rgb, seq, topic = self.latest_rgb, self.latest_seq, self.topic
                if rgb is not None and seq != last_seq:
                    # Only the preview is JPEG; calibration samples use the PNG path above.
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    ok, encoded = cv2.imencode(
                        ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok:
                        self._log("远端 JPEG 预览编码失败")
                    else:
                        payload = encoded.tobytes()
                        if len(payload) > MAX_PACKET_BYTES:
                            self._log("远端图像超过协议大小限制，已丢弃")
                            last_seq = seq
                        else:
                            _wire_send("image", payload=payload, stop_event=self.stop,
                                       extra={"width": int(rgb.shape[1]),
                                              "height": int(rgb.shape[0]), "seq": seq,
                                              "topic": topic, "format": "jpeg"})
                            last_seq = seq
                    last_heartbeat = now
                elif now - last_heartbeat >= 1.0:
                    _wire_send("heartbeat", data={"time": time.time(), "topic": self.topic},
                               stop_event=self.stop)
                    last_heartbeat = now
                next_frame = now + delay  # never catch up by emitting queued frames
            time.sleep(min(0.02, max(0.001, next_frame - time.monotonic())))

    def _cleanup(self):
        self.stop.set()
        try:
            self.commands.put_nowait({"cmd": "shutdown"})
            self.parameter_commands.put_nowait({"cmd": "shutdown"})
        except queue.Full:
            pass
        if self.executor:
            try:
                self.executor.shutdown(timeout_sec=0.2)
            except Exception:
                pass
        if self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        proc = self.driver_proc
        if proc and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError, OSError):
                try:
                    proc.terminate()
                except OSError:
                    pass
            try:
                proc.wait(timeout=0.6)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError, OSError):
                    try:
                        proc.kill()
                    except OSError:
                        pass
                try:
                    proc.wait(timeout=0.4)
                except subprocess.TimeoutExpired:
                    pass
        if self.stdin_thread and self.stdin_thread is not threading.current_thread():
            self.stdin_thread.join(timeout=0.25)
        if self.driver_log_thread and self.driver_log_thread is not threading.current_thread():
            self.driver_log_thread.join(timeout=1.0)
        if proc:
            # The log reader uses os.read on the file descriptor, so it owns no
            # BufferedReader lock. Close the stream only after its bounded join.
            if not self.driver_log_thread or not self.driver_log_thread.is_alive():
                try:
                    proc.stdout.close()
                except (AttributeError, OSError, ValueError):
                    pass

    def run(self):
        previous_handlers = {}

        def request_stop(signum, _frame):
            self.stop.set()
            self._log("收到信号 %s，正在清理远端相机" % signum)

        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            try:
                previous_handlers[sig] = signal.signal(sig, request_stop)
            except (ValueError, OSError):
                pass
        try:
            self.stdin_thread = threading.Thread(
                target=self._stdin_loop, name="remote-camera-stdin", daemon=True)
            self.stdin_thread.start()
            self._start_driver()
            if self.stop.is_set():
                return 0
            self._start_ros()
            # Explicitly state that identity is remote and currently unconfirmed. A later
            # CALIB_SELECTED_CAMERA log replaces this with the selected device snapshot.
            _wire_send("identity", data={"status": "unavailable", "devices": []},
                       stop_event=self.stop)
            threading.Thread(target=self._ros_loop, name="remote-camera-ros",
                             daemon=True).start()
            threading.Thread(target=self._parameter_loop, name="remote-camera-params",
                             daemon=True).start()
            self._send_frames()
            return 0
        except Exception as exc:
            self._log("远端相机服务启动失败：%s" % exc)
            try:
                _wire_send("error", data={"message": "远端相机服务启动失败：%s" % exc},
                           stop_event=self.stop)
            except Exception:
                pass
            return 1
        finally:
            self._cleanup()
            for sig, handler in previous_handlers.items():
                try:
                    signal.signal(sig, handler)
                except (ValueError, OSError):
                    pass


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serve", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--driver", choices=DRIVER_NAMES, default="hik")
    parser.add_argument("--topic", default="/image_raw")
    parser.add_argument("--camera-root", default="")
    parser.add_argument("--remote-dir", default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--fps", type=float, default=0, help="预览帧率上限，0 表示不限速（默认）")
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    if not args.serve:
        print("此脚本由 RemoteConnection 通过 SSH 启动；单独运行请加 --serve。", file=sys.stderr)
        return 2
    if not math.isfinite(args.fps) or args.fps < 0:
        print("fps 必须是非负有限数，0 表示不限速", file=sys.stderr)
        return 2
    if not args.topic.strip():
        print("topic 不能为空", file=sys.stderr)
        return 2
    return _RemoteService(args).run()


if __name__ == "__main__":
    sys.exit(main())
