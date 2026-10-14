"""GUI 和命令行共用相机运行环境，避免同一相机走两个入口却加载不同 SDK。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex

HERE = Path(__file__).resolve().parent


def default_camera_root():
    """三个相机驱动的工作空间根目录（本工具的“驱动”功能需要它们）。

    优先级：环境变量 CAMERA_ROOT > 常见位置自动探测 > 家目录下的默认猜测。
    三个驱动各自的子目录名是上游仓库名（ros2-hik-camera-main 等），换位置时只要保证
    目录名不变、或直接设 CAMERA_ROOT 指到新的根目录即可。
    """
    env = os.environ.get("CAMERA_ROOT")
    if env:
        return os.path.abspath(os.path.expanduser(env))
    home = os.path.expanduser("~")
    for cand in (os.path.join(home, "下载", "camera"),
                 os.path.join(home, "Downloads", "camera"),
                 os.path.join(home, "camera"),
                 os.path.join(home, "camera_ws"),
                 os.path.join(home, "ws", "camera"),
                 "/opt/camera"):
        if os.path.isdir(cand):
            return cand
    return os.path.join(home, "下载", "camera")


def build_drivers(camera_root):
    """按驱动根目录生成三个驱动的元信息表。"""
    return {
        "hik": dict(
            label="海康 MVS 工业相机", pkg="hik_camera", exe="hik_camera_node",
            ws=os.path.join(camera_root, "ros2-hik-camera-main"),
            node="/hik_camera", exp="exposure_time", gain="gain", cam_name="narrow_stereo"),
        "galaxy": dict(
            label="大恒 Galaxy", pkg="galaxy_camera", exe="galaxy_camera_node",
            ws=os.path.join(camera_root, "rm_vision_ros2_galaxy_camera-master"),
            node="/galaxy_camera", exp="exposure_time", gain="gain", cam_name="narrow_stereo"),
        "mv": dict(
            label="迈德威视", pkg="mindvision_camera", exe="mindvision_camera_node",
            ws=os.path.join(camera_root, "ros2_mindvision_camera"),
            node="/mv_camera", exp="exposure_time", gain="analog_gain", cam_name="mv_camera"),
    }


def driver_patch_digest(driver):
    patches = sorted((HERE / 'driver_patches').glob(driver + '*.patch'))
    return hashlib.sha256(b''.join(p.read_bytes() for p in patches)).hexdigest()


def runtime_setup(driver):
    path = HERE / '.driver_runtime' / driver / 'install' / 'setup.bash'
    return str(path.resolve()) if path.is_file() else ''


def sdk_environment(driver, env):
    """已有修复驱动优先使用它安装的整套 SDK；旧海康驱动优先系统 MVS。"""
    result = dict(env)
    overlay = runtime_setup(driver)
    if overlay:
        installed_hash = Path(overlay).parent.parent / 'patch.sha256'
        patch = HERE / 'driver_patches' / (driver + '.patch')
        if (not installed_hash.is_file() or not patch.is_file() or
                installed_hash.read_text().strip() != driver_patch_digest(driver)):
            raise RuntimeError('兼容驱动补丁已更新，请运行 /usr/bin/python3 -s build_camera_drivers.py ' + driver)
        pkg = {'hik': 'hik_camera', 'galaxy': 'galaxy_camera', 'mv': 'mindvision_camera'}[driver]
        library = str(Path(overlay).parent / pkg / 'lib')
    elif driver == 'hik':
        root = Path(result.get('HIK_SDK_ROOT') or '/opt/MVS')
        import platform
        arch = 'arm64' if platform.machine() in ('aarch64', 'arm64') else 'amd64'
        library = str(root / 'lib' / arch if (root / 'lib' / arch).is_dir() else root / 'lib' / '64')
        if not (Path(library) / 'libMvCameraControl.so').is_file():
            if result.get('HIK_SDK_ROOT'):
                raise RuntimeError('HIK_SDK_ROOT 下缺少 lib/64/libMvCameraControl.so；请重建驱动')
            return result
    else:
        return result
    paths = [p for p in result.get('LD_LIBRARY_PATH', '').split(':') if p and p != library]
    result['LD_LIBRARY_PATH'] = ':'.join([library] + paths)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('driver', choices=['hik', 'galaxy', 'mv'])
    parser.add_argument('--shell', action='store_true')
    parser.add_argument('--setup', action='store_true')
    args = parser.parse_args()
    if args.setup:
        print(runtime_setup(args.driver))
    else:
        env = sdk_environment(args.driver, os.environ)
        if args.shell:
            print('export LD_LIBRARY_PATH=' + shlex.quote(env.get('LD_LIBRARY_PATH', '')))
        else:
            print(json.dumps(env))
