"""Local, per-host SSH connection profiles for the camera calibration GUI."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from collections.abc import Mapping


HERE = Path(__file__).resolve().parent
DEFAULT_DIR = HERE / "ssh_configs"
_VERSION = 1
_DRIVERS = {"hik", "galaxy", "mv"}


def _text(value, name, *, default=None, allow_empty=False):
    if value is None and default is not None:
        value = default
    if not isinstance(value, str):
        raise ValueError("%s 必须是字符串" % name)
    value = value.strip()
    if not allow_empty and not value:
        raise ValueError("%s 不能为空" % name)
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise ValueError("%s 不能包含控制字符" % name)
    return value


def _normalize(profile, *, include_password):
    if not isinstance(profile, Mapping):
        raise ValueError("SSH 配置必须是对象")
    if "version" in profile and (type(profile["version"]) is not int or profile["version"] != _VERSION):
        raise ValueError("不支持的 SSH 配置版本")

    host = _text(profile.get("host"), "host")
    if host.startswith("-") or any(char.isspace() for char in host):
        raise ValueError("host 不能以 - 开头或包含空白字符")

    port = profile.get("port", 22)
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("port 必须是 1 到 65535 之间的整数")

    identity_file = _text(profile.get("identity_file", ""), "identity_file", allow_empty=True)
    remote_dir = _text(profile.get("remote_dir"), "remote_dir")
    camera_root = _text(profile.get("camera_root", ""), "camera_root", allow_empty=True)
    driver = _text(profile.get("driver", "hik"), "driver")
    if driver not in _DRIVERS:
        raise ValueError("driver 必须是 hik、galaxy 或 mv")
    topic = _text(profile.get("topic", "/image_raw"), "topic")
    if any(char.isspace() for char in topic):
        raise ValueError("topic 不能包含空白字符")

    result = {
        "version": _VERSION,
        "host": host,
        "port": port,
        "identity_file": identity_file,
        "remote_dir": remote_dir,
        "camera_root": camera_root,
        "driver": driver,
        "topic": topic,
    }
    if include_password and "password" in profile:
        password = profile["password"]
        if not isinstance(password, str):
            raise ValueError("password 必须是字符串")
        result["password"] = password  # 密码中的空格也是有效字符，不做 strip。
    return result


def _profile_path(host, port, directory):
    readable = re.sub(r"[^a-zA-Z0-9_.-]+", "_", host.casefold()).strip("._-")
    readable = (readable or "host")[:40]
    digest = hashlib.sha256((host + "\0" + str(port)).encode("utf-8")).hexdigest()[:10]
    return Path(directory) / ("%s-%d-%s.json" % (readable, port, digest))


def save_profile(profile, directory=DEFAULT_DIR, remember_password=False):
    """Validate and atomically save a profile; passwords are omitted by default."""
    if type(remember_password) is not bool:
        raise ValueError("remember_password 必须是布尔值")
    normalized = _normalize(profile, include_password=remember_password)
    path = _profile_path(normalized["host"], normalized["port"], directory)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)

    fd, tmp_name = tempfile.mkstemp(prefix=".%s." % path.stem, suffix=".tmp", dir=str(path.parent))
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(normalized, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_name, path)
    except BaseException:
        try:
            os.close(fd)
        except OSError:
            pass
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise
    return path


def load_profile(path):
    """Load and validate one versioned JSON profile."""
    try:
        with open(path, "r", encoding="utf-8") as stream:
            raw = json.load(stream)
    except json.JSONDecodeError as exc:
        raise ValueError("SSH 配置 JSON 格式无效：%s" % exc.msg) from exc
    except UnicodeDecodeError as exc:
        raise ValueError("SSH 配置不是有效的 UTF-8 JSON") from exc
    if not isinstance(raw, dict):
        raise ValueError("SSH 配置 JSON 顶层必须是对象")
    if "version" not in raw:
        raise ValueError("SSH 配置缺少版本号")
    return _normalize(raw, include_password=True)


def list_profiles(directory=DEFAULT_DIR):
    """List JSON profile files in stable filename order."""
    directory = Path(directory)
    if not directory.is_dir():
        return []
    return sorted((path for path in directory.glob("*.json") if path.is_file()),
                  key=lambda path: path.name)
