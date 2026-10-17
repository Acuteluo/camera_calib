#!/usr/bin/env bash
# 图形界面需要桌面；本地来源需要 ROS，SSH 来源仅需基础依赖。--self-test 无需显示。
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
  # SSH 图像来源无需本机安装 ROS；存在 ROS 时仍加载它以支持切回本地。
  calib_ros_env || true
  "$CALIB_PYTHON" -s "$HERE/check_runtime.py" || {
    echo "运行依赖检查失败，请执行 $HERE/install_deps.sh（详细错误见上方）。" >&2
    exit 1
  }
fi
exec "$CALIB_PYTHON" -s "$HERE/calib_gui.py" "$@"
