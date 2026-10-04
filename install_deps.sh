#!/usr/bin/env bash
# Ubuntu apt 部署；ROS 本体由用户安装，纯 OpenCV/离线自检不需要 ROS。
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-install}"
[[ $# -le 1 && ( "$MODE" == install || "$MODE" == --check ) ]] || {
  echo "用法: $0 [--check]" >&2; exit 2;
}
# shellcheck source=runtime.sh
source "$HERE/runtime.sh"
command -v dpkg-query >/dev/null && command -v apt-get >/dev/null || {
  echo "错误：本脚本需要 Ubuntu/Debian apt；其他环境请使用 requirements.txt。" >&2; exit 1;
}
SUDO=()
[[ $(id -u) -eq 0 ]] || SUDO=(sudo)
if [[ ! -x "$CALIB_PYTHON" ]]; then
  [[ "$MODE" != --check ]] || { echo "缺少系统 Python：$CALIB_PYTHON" >&2; exit 1; }
  "${SUDO[@]}" apt-get update && "${SUDO[@]}" apt-get install -y python3 || exit 1
fi
calib_base_env || exit 1
# dpkg-query -W 对已删除但残留配置的包也可能返回 0，必须核对安装状态。
have() { [[ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null)" == 'install ok installed' ]]; }
REQUIRED=(python3 python3-numpy python3-opencv python3-yaml python3-pil python3-pil.imagetk python3-tk)
RECOMMENDED=(fonts-noto-cjk)
ROS_ARGS=()
if calib_ros_env; then
  echo "ROS 2: $ROS_DISTRO"
  REQUIRED+=("ros-$ROS_DISTRO-rclpy" "ros-$ROS_DISTRO-sensor-msgs")
  ROS_ARGS=(--ros)
else
  echo "未找到 ROS 2：可运行离线标定/自检；启动图形界面的 ROS 订阅功能需另装 ROS。"
fi
MISSING=()
for package in "${REQUIRED[@]}"; do
  if have "$package"; then
    echo "✔ $package 已安装"
  else
    echo "✘ $package 未安装"
    MISSING+=("$package")
  fi
done
RUNTIME_OK=0
"$CALIB_PYTHON" -s "$HERE/check_runtime.py" "${ROS_ARGS[@]}" && RUNTIME_OK=1
if [[ "$MODE" == --check ]]; then
  [[ ${#MISSING[@]} -eq 0 && $RUNTIME_OK -eq 1 ]] || {
    echo "依赖未就绪，请运行 ./install_deps.sh。" >&2; exit 1;
  }
  echo "必需依赖包及实际 Python 导入全部通过。"
  exit 0
fi
OPTIONAL_MISSING=()
for package in "${RECOMMENDED[@]}"; do
  have "$package" || OPTIONAL_MISSING+=("$package")
done
if [[ ${#MISSING[@]} -gt 0 || $RUNTIME_OK -eq 0 || ${#OPTIONAL_MISSING[@]} -gt 0 ]]; then
  "${SUDO[@]}" apt-get update || { echo "apt 更新失败，请检查软件源/网络。" >&2; exit 1; }
  if [[ ${#MISSING[@]} -gt 0 || $RUNTIME_OK -eq 0 ]]; then
    "${SUDO[@]}" apt-get install -y "${REQUIRED[@]}" || exit 1
  fi
  if [[ ${#OPTIONAL_MISSING[@]} -gt 0 ]]; then
    "${SUDO[@]}" apt-get install -y "${OPTIONAL_MISSING[@]}" || echo "提示：中文字体未安装。" >&2
  fi
fi
# 安装成功必须以运行环境可用为准；损坏的系统包尝试修复一次。
if ! "$CALIB_PYTHON" -s "$HERE/check_runtime.py" "${ROS_ARGS[@]}"; then
  echo "安装后导入仍失败，尝试重新安装系统依赖。"
  "${SUDO[@]}" apt-get install --reinstall -y "${REQUIRED[@]}" || exit 1
  "$CALIB_PYTHON" -s "$HERE/check_runtime.py" "${ROS_ARGS[@]}" || {
    echo "依赖仍不可用：请检查上方模块路径及错误，尤其 /usr/local 下的 pip 包冲突。" >&2; exit 1;
  }
fi
for package in "${REQUIRED[@]}"; do
  have "$package" || { echo "安装状态异常：$package" >&2; exit 1; }
done
echo "安装和实际导入验证完成。"
echo "无相机自检：./run_gui.sh --self-test"
echo "桌面启动：  ./run_gui.sh"
echo "ROS 官方标定（可选）：sudo apt install ros-\${ROS_DISTRO}-camera-calibration ros-\${ROS_DISTRO}-image-pipeline"
