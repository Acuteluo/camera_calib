#!/usr/bin/env bash
# =============================================================================
# 图形化相机标定工具的启动脚本
#   ./run_gui.sh                        # 默认海康，11x8 内角点，曝光 5000us，方格 25mm
#   ./run_gui.sh --driver mv --size 11x8 --square 0.025
#   ./run_gui.sh --topic /image_raw
#
# 会自动 source ROS 2 环境；必须在桌面终端里运行（GUI 需要图形显示）。
# =============================================================================
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  echo "错误：没有图形显示（DISPLAY 为空）。" >&2
  echo "      请在桌面终端（Ctrl+Alt+T）里运行，不要通过 SSH 或后台服务运行。" >&2
  exit 1
fi

# ROS 2 环境：优先当前已 source 的 ROS_DISTRO，其次 humble，最后 /opt/ros 下任意一个
ROS_SETUP=""
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  ROS_SETUP="/opt/ros/humble/setup.bash"
else
  for _c in $(ls -1d /opt/ros/*/setup.bash 2>/dev/null | sort -r); do
    ROS_SETUP="$_c"; break
  done
fi

if [[ -z "$ROS_SETUP" ]]; then
  echo "错误：找不到任何 ROS 2 环境（/opt/ros/*/setup.bash）。" >&2
  echo "      请先安装 ROS 2（Humble 或更新版本），或确认 /opt/ros 存在。" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

# 依赖预检：缺包时给出明确指引，而不是抛一堆 ImportError
_missing=""
for _m in tkinter "PIL.ImageTk" cv2 numpy yaml rclpy; do
  python3 -c "import $_m" >/dev/null 2>&1 || _missing="$_missing $_m"
done
if [[ -n "$_missing" ]]; then
  echo "错误：缺少 Python 依赖:$_missing" >&2
  echo "      先跑 ./install_deps.sh 一键安装，或 ./install_deps.sh --check 看详情。" >&2
  exit 1
fi

exec python3 "$HERE/calib_gui.py" "$@"
