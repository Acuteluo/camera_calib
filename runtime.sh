#!/usr/bin/env bash
# apt/ROS 使用 Ubuntu 系统 Python；不受 Conda、venv 和用户 pip 包影响。
CALIB_PYTHON=/usr/bin/python3
calib_base_env() {
  if [[ ! -x "$CALIB_PYTHON" ]]; then
    echo "错误：找不到 $CALIB_PYTHON。请安装 Ubuntu python3。" >&2
    return 1
  fi
  unset PYTHONHOME PYTHONPATH
  export PYTHONNOUSERSITE=1
  export PATH="/usr/bin:/bin:$PATH"
}
calib_ros_env() {
  local setup="" candidate
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    setup="/opt/ros/$ROS_DISTRO/setup.bash"
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    setup=/opt/ros/humble/setup.bash
  else
    for candidate in /opt/ros/*/setup.bash; do
      [[ -f "$candidate" ]] && setup="$candidate" && break
    done
  fi
  [[ -n "$setup" ]] || return 1
  # shellcheck disable=SC1090
  source "$setup"
}
