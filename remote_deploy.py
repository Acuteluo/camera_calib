#!/usr/bin/env python3
"""Upload local camera sources and SDKs, then build them in an isolated SSH release."""

from __future__ import annotations

import os
import posixpath
import shlex
import shutil
import subprocess
import tarfile
import tempfile
import threading
import uuid
from pathlib import Path

from driver_runtime import build_drivers
import remote_camera


DRIVERS = ("hik", "galaxy", "mv")
ALL_DRIVERS = DRIVERS
SDK_SPECS = {
    "hik": ("HIK_SDK_ROOT", "hikSDK", "MvCameraControl.h"),
    "galaxy": ("GALAXY_SDK_ROOT", "GalaxySDK", "GxIAPI.h"),
    "mv": ("MV_SDK_ROOT", "mvsdk", "CameraApi.h"),
}
PROJECT_FILES = {
    "CMakeLists.txt", "package.xml", "src", "include", "config", "launch",
    "hikSDK", "GalaxySDK", "mvsdk", "resource", "LICENSE",
}
GENERATED_NAMES = {
    ".git", ".driver_runtime", "build", "install", "log", "__pycache__",
    "MvSdkLog", "标定结果", "calibration_results", "camera_data", "captures",
}
CAMERA_DATA_SUFFIXES = {
    ".mcap", ".bag", ".db3", ".jpg", ".jpeg", ".png", ".bmp", ".tif",
    ".tiff", ".avi", ".mp4", ".mov",
}
SOURCE_PROJECT_FILES = PROJECT_FILES - {spec[1] for spec in SDK_SPECS.values()}
SSH_TIMEOUT_SECONDS = 15 * 60


def _selected_drivers(driver):
    if driver == "all":
        return list(ALL_DRIVERS)
    if driver not in DRIVERS:
        raise ValueError("driver 必须是 hik、galaxy、mv 或 all")
    return [driver]


def _emit(log, message):
    if log is None:
        return
    try:
        log(str(message))
    except Exception:
        # A closed GUI log view must not interrupt a remote build.
        pass


def _quote_remote_path(path):
    """Quote a path for SSH's remote shell, preserving a leading ~/ expansion."""
    path = str(path)
    if path == "~":
        return '"$HOME"'
    if path.startswith("~/"):
        return '"$HOME"/' + shlex.quote(path[2:])
    return shlex.quote(path)


def _join_remote_path(path, *parts):
    path = str(path).strip()
    if not path:
        raise ValueError("remote_dir 不能为空")
    if path != "/":
        path = path.rstrip("/")
    for part in parts:
        path = path.rstrip("/") + "/" + str(part).strip("/")
    return path


def _ignore_source(directory, names):
    excluded = {name for name in names if name in GENERATED_NAMES}
    excluded.update(
        name for name in names
        if Path(name).suffix.lower() in CAMERA_DATA_SUFFIXES
    )
    return excluded


def _copy_driver_source(source, destination):
    if not source.is_dir():
        raise FileNotFoundError("找不到本机 %s 驱动源码：%s" % (destination.name, source))

    required = ("package.xml", "CMakeLists.txt", "config/camera_info.yaml")
    missing = [name for name in required if not (source / name).is_file()]
    if missing:
        raise FileNotFoundError(
            "本机驱动源码 %s 缺少 %s" % (source, ", ".join(missing))
        )

    def ignore(directory, names):
        if Path(directory) == source:
            return set(names) - SOURCE_PROJECT_FILES
        return _ignore_source(directory, names)

    shutil.copytree(source, destination, symlinks=True, ignore=ignore)


def _sdk_root(driver, source, local_camera_root, environ):
    variable, sdk_name, header = SDK_SPECS[driver]
    configured = environ.get(variable)
    if configured:
        return Path(os.path.expandvars(os.path.expanduser(configured))).resolve(), sdk_name

    if driver == "hik":
        system_mvs = Path("/opt/MVS")
        if (system_mvs / "include" / header).is_file():
            return system_mvs.resolve(), sdk_name

    return (source / sdk_name).resolve(), sdk_name


def _validate_sdk_links(sdk_root):
    """Reject SDK symlinks that escape the archive's include/lib payload."""
    sdk_root = Path(sdk_root)
    for entry in (sdk_root / "include", sdk_root / "lib"):
        if not entry.exists() and not entry.is_symlink():
            continue
        candidates = [entry]
        if entry.is_dir() and not entry.is_symlink():
            for current, dirs, files in os.walk(entry, followlinks=False):
                for name in dirs + files:
                    candidate = Path(current) / name
                    if candidate.is_symlink():
                        candidates.append(candidate)
        for candidate in candidates:
            if not candidate.is_symlink():
                continue
            target = os.readlink(candidate)
            if os.path.isabs(target):
                raise ValueError("SDK 符号链接不能使用绝对路径：%s -> %s" % (candidate, target))
            relative = candidate.relative_to(sdk_root).as_posix()
            resolved = posixpath.normpath(posixpath.join(posixpath.dirname(relative), target))
            if (resolved == ".." or resolved.startswith("../") or
                    not (resolved.startswith("include/") or resolved == "include" or
                         resolved.startswith("lib/") or resolved == "lib")):
                raise ValueError("SDK 符号链接指向打包范围之外：%s -> %s" % (candidate, target))


def _copy_sdk(driver, source, destination, local_camera_root, environ):
    sdk_root, sdk_name = _sdk_root(driver, source, local_camera_root, environ)
    if not sdk_root.is_dir():
        return False

    _validate_sdk_links(sdk_root)
    sdk_destination = destination / sdk_name
    copied = False
    for name in ("include", "lib"):
        item = sdk_root / name
        if not item.exists() and not item.is_symlink():
            continue
        output = sdk_destination / name
        output.parent.mkdir(parents=True, exist_ok=True)
        if item.is_symlink():
            os.symlink(os.readlink(item), output)
        elif item.is_dir():
            shutil.copytree(item, output, symlinks=True)
        elif item.is_file():
            shutil.copy2(item, output)
        copied = True
    return copied


def _validate_archive_symlinks(project):
    """Do not put links escaping the uploaded project into the remote tarball."""
    root = Path(project)
    for current, dirs, files in os.walk(root, followlinks=False):
        for name in dirs + files:
            path = Path(current) / name
            if not path.is_symlink():
                continue
            target = os.readlink(path)
            if os.path.isabs(target):
                raise ValueError("发布包中不允许绝对符号链接：%s -> %s" % (path, target))
            member = path.relative_to(root).as_posix()
            resolved = posixpath.normpath(posixpath.join(posixpath.dirname(member), target))
            if resolved == ".." or resolved.startswith("../"):
                raise ValueError("发布包符号链接逃逸项目目录：%s -> %s" % (path, target))


def _create_archive(local_camera_root, drivers, environ=None):
    """Create a temporary gzip tar and return it with SDK variables bundled in it."""
    environ = os.environ if environ is None else environ
    local_camera_root = Path(os.path.expanduser(str(local_camera_root))).resolve()
    source_map = build_drivers(str(local_camera_root))
    archive = tempfile.TemporaryFile(prefix="camera-ssh-deploy-", suffix=".tar.gz")
    bundled_sdk_vars = {}
    try:
        with tempfile.TemporaryDirectory(prefix="camera-ssh-project-") as temp_dir:
            project = Path(temp_dir) / "project"
            project.mkdir()

            repository = Path(__file__).resolve().parent
            for candidate in sorted(repository.iterdir()):
                if candidate.is_file() and candidate.suffix in (".py", ".sh"):
                    if candidate.is_symlink():
                        raise ValueError("核心脚本不能是符号链接：%s" % candidate)
                    shutil.copy2(candidate, project / candidate.name)
            for directory in ("driver_patches", "reference_json"):
                source_dir = repository / directory
                if source_dir.is_dir():
                    shutil.copytree(source_dir, project / directory, symlinks=True,
                                    ignore=_ignore_source)

            drivers_root = project / "drivers"
            drivers_root.mkdir()
            for driver in drivers:
                source = Path(source_map[driver]["ws"]).resolve()
                destination = drivers_root / source.name
                _copy_driver_source(source, destination)
                if _copy_sdk(driver, source, destination, local_camera_root, environ):
                    variable = SDK_SPECS[driver][0]
                    bundled_sdk_vars[driver] = variable

            _validate_archive_symlinks(project)
            with tarfile.open(fileobj=archive, mode="w:gz", format=tarfile.PAX_FORMAT) as tar:
                tar.add(project, arcname="project", recursive=True)
        archive.seek(0)
        return archive, bundled_sdk_vars
    except Exception:
        archive.close()
        raise


def _remote_deploy_command(remote_dir, release_id, drivers, bundled_sdk_vars):
    release_base = _join_remote_path(remote_dir, ".ssh_deploy")
    release = _join_remote_path(release_base, release_id)
    stage = _join_remote_path(release_base, ".staging-" + release_id)
    words = [
        # ROS Humble's generated setup scripts reference optional unset variables.
        # Keep nounset disabled while still stopping on command/pipeline failures.
        "set -eo pipefail",
        "base=%s" % _quote_remote_path(release_base),
        "release=%s" % _quote_remote_path(release),
        "stage=%s" % _quote_remote_path(stage),
        "release_created=0",
        "mkdir -p -- \"$base\"",
        "if [[ -e \"$release\" || -e \"$stage\" ]]; then echo '发布目录已存在，拒绝覆盖' >&2; exit 1; fi",
        "mkdir -m 700 -- \"$stage\"",
        "cleanup() { if [[ -d \"${stage:-}\" ]]; then rm -rf -- \"$stage\"; fi; if [[ \"$release_created\" == 1 && -d \"$release\" ]]; then rm -rf -- \"$release\"; fi; }",
        "trap cleanup EXIT",
        "tar --extract --gzip --file=- --no-same-owner --no-same-permissions --directory=\"$stage\"",
        # Colcon embeds install paths in setup files and RPATHs. Build at the permanent
        # release path so moving a completed staging tree cannot invalidate them.
        "mv -T -- \"$stage\" \"$release\"",
        "stage=''",
        "release_created=1",
        "cd -- \"$release/project\"",
        "source ./runtime.sh",
        "calib_base_env",
        "if ! calib_ros_env; then :; fi",
        "export CAMERA_ROOT=\"$PWD/drivers\"",
    ]
    source_names = {
        "hik": "ros2-hik-camera-main",
        "galaxy": "rm_vision_ros2_galaxy_camera-master",
        "mv": "ros2_mindvision_camera",
    }
    for driver in drivers:
        variable = bundled_sdk_vars.get(driver)
        if variable:
            sdk_name = SDK_SPECS[driver][1]
            words.append(
                "export %s=\"$CAMERA_ROOT/%s/%s\"" %
                (variable, source_names[driver], sdk_name)
            )
    args = " ".join(shlex.quote(driver) for driver in drivers)
    words.extend((
        "/usr/bin/python3 -s build_camera_drivers.py " + args,
        "printf 'READY\\n' > \"$release/READY\"",
        "release_created=0",
        "printf 'DEPLOYED_PROJECT=%s\\n' \"$release/project\"",
    ))
    return "bash -c " + shlex.quote("; ".join(words))


def _run_ssh(argv, archive, log, timeout=SSH_TIMEOUT_SECONDS, env=None):
    """Upload the tar while forwarding remote build output to the GUI as it arrives."""
    process = subprocess.Popen(
        argv, stdin=archive, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0,
        env=env,
    )

    def forward_output():
        try:
            for raw in iter(process.stdout.readline, b""):
                line = raw.decode("utf-8", "replace").rstrip()
                if line:
                    _emit(log, line)
        except (OSError, ValueError) as error:
            _emit(log, "读取 SSH 部署日志失败：%s" % error)

    reader = threading.Thread(target=forward_output, name="remote-deploy-log", daemon=True)
    reader.start()
    try:
        returncode = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired as error:
        process.kill()
        process.wait()
        reader.join(timeout=5)
        raise RuntimeError("远端驱动部署超过 15 分钟，未确认发布成功") from error
    finally:
        reader.join(timeout=5)
        if process.stdout:
            process.stdout.close()
    return returncode


def deploy_remote(host, port, identity_file, remote_dir, driver, local_camera_root, log,
                  password=None) -> str:
    """Upload local driver sources/SDKs and build a new isolated remote release."""
    selected = _selected_drivers(driver)
    remote_dir = str(remote_dir).strip()
    if not remote_dir:
        raise ValueError("remote_dir 不能为空")

    release_id = uuid.uuid4().hex
    _emit(log, "准备打包本机驱动源码与 SDK：%s" % ", ".join(selected))
    archive, bundled_sdk_vars = _create_archive(local_camera_root, selected)
    try:
        # Reuse the camera bridge's host, port, identity and non-interactive SSH policy.
        argv = remote_camera.build_ssh_argv(
            host, port, identity_file, ".", selected[0], topic="/image_raw", fps=10,
            password=password,
        )
        argv[-1] = _remote_deploy_command(remote_dir, release_id, selected, bundled_sdk_vars)
        _emit(log, "正在通过 SSH 上传并在远端编译；最多等待 15 分钟。")
        archive.seek(0)
        try:
            returncode = _run_ssh(
                argv, archive, log, env=remote_camera.ssh_environment(password),
            )
        except OSError as error:
            raise RuntimeError("无法启动 SSH 部署：%s" % error) from error
        if returncode:
            raise RuntimeError("SSH 上传或远端构建失败（退出码 %s）；未发布新项目" % returncode)
        deployed = _join_remote_path(remote_dir, ".ssh_deploy", release_id, "project")
        _emit(log, "远端驱动已构建并发布：%s" % deployed)
        return deployed
    finally:
        archive.close()
