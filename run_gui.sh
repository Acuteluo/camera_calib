#!/usr/bin/env bash
# 图形界面默认需要 ROS 和桌面；--self-test 无需 ROS/显示。
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime.sh
source "$HERE/runtime.sh"
calib_base_env || exit 1
SELF_TEST=0
for arg in "$@"; do
  [[ "$arg" == "--self-test" ]] && SELF_TEST=1
done
if (( SELF_TEST )); then
  "$CALIB_PYTHON" -s "$HERE/check_runtime.py" || exit 1
else
  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "错误：没有图形显示。请在桌面终端运行；无相机自检用 ./run_gui.sh --self-test。" >&2
    exit 1
  fi
  if ! calib_ros_env; then
    echo "错误：找不到 /opt/ros/*/setup.bash。请先安装 ROS 2（Ubuntu 22.04 用 Humble）。" >&2
    exit 1
  fi
  "$CALIB_PYTHON" -s "$HERE/check_runtime.py" --ros || {
    echo "运行依赖检查失败，请执行 $HERE/install_deps.sh（详细错误见上方）。" >&2
    exit 1
  }
fi
exec "$CALIB_PYTHON" -s "$HERE/calib_gui.py" "$@"
