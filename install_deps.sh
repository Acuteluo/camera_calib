#!/usr/bin/env bash
# =============================================================================
# install_deps.sh —— 安装本工具的系统依赖（Ubuntu 22.04 + ROS 2）
#
#   ./install_deps.sh            # 检查并安装缺失的依赖
#   ./install_deps.sh --check    # 只检查、只报告，不安装（不需要 sudo）
#
# 说明：
#   * 只安装系统包，不改动你的代码、配置和标定结果；
#   * ROS 2 本身（rclpy / sensor_msgs / image_transport / cv_bridge）需要你事先装好，
#     本脚本只做检查并给出官方安装指引，不会替你装 ROS。
# =============================================================================
set -o pipefail

MODE="${1:-install}"
if [[ "$MODE" != "--check" && "$MODE" != "install" ]]; then
  echo "用法: $0 [--check]" >&2
  exit 2
fi

C_OK=$'\033[32m'; C_ERR=$'\033[31m'; C_WARN=$'\033[33m'; C_DIM=$'\033[2m'; C_END=$'\033[0m'
ok()   { printf '%s✔%s %s\n' "$C_OK" "$C_END" "$*"; }
bad()  { printf '%s✘%s %s\n' "$C_ERR" "$C_END" "$*"; }
warn() { printf '%s!%s %s\n' "$C_WARN" "$C_END" "$*"; }
dim()  { printf '%s%s%s\n' "$C_DIM" "$*" "$C_END"; }

have() { dpkg-query -W -f='${Version}' "$1" >/dev/null 2>&1; }

# ROS 发行版：优先当前环境，否则 humble，否则 /opt/ros 下任意一个
ROS_DISTRO_DETECTED="${ROS_DISTRO:-}"
if [[ -z "$ROS_DISTRO_DETECTED" || ! -f "/opt/ros/$ROS_DISTRO_DETECTED/setup.bash" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    ROS_DISTRO_DETECTED=humble
  else
    for _d in $(ls -1d /opt/ros/*/setup.bash 2>/dev/null | sort -r); do
      ROS_DISTRO_DETECTED="$(basename "$(dirname "$_d")")"; break
    done
  fi
fi

# 必需：Python 运行库（GUI / 标定 / YAML）
REQUIRED=(python3-numpy python3-opencv python3-yaml python3-pil python3-pil.imagetk python3-tk)
# 推荐：中文字体（没有也能跑，界面中文会退化成方框/英文字体）
RECOMMENDED=(fonts-noto-cjk)
# 可选：只有 ros2_calibrate.sh（ROS 官方 cameracalibrator 编排）路线需要
OPTIONAL=()

echo "=============================================================="
echo " 相机标定工具 —— 依赖检查"
echo "   （操作系统 与 ROS 2 发行版 是两层：Humble 绑定 Ubuntu 22.04）"
echo "=============================================================="
echo
echo "[0/4] 操作系统"
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  echo "  ${PRETTY_NAME:-未知}"
  if [[ "${VERSION_ID:-}" == "22.04" ]]; then
    ok "Ubuntu 22.04 —— 本工具验证过的版本"
  else
    warn "非 Ubuntu 22.04（VERSION_ID=${VERSION_ID:-?}）——未验证；纯 OpenCV 路线一般仍可用"
  fi
else
  warn "读不到 /etc/os-release，无法判断发行版"
fi
echo
echo "[1/4] ROS 2 发行版"
if [[ -n "$ROS_DISTRO_DETECTED" && -f "/opt/ros/$ROS_DISTRO_DETECTED/setup.bash" ]]; then
  ok "找到 ROS 2 $ROS_DISTRO_DETECTED （/opt/ros/$ROS_DISTRO_DETECTED）"
  OPTIONAL=("ros-$ROS_DISTRO_DETECTED-camera-calibration" "ros-$ROS_DISTRO_DETECTED-image-pipeline")
else
  bad "没找到任何 ROS 2 环境（/opt/ros/*/setup.bash）"
  dim "    纯 OpenCV 路线（calibrate_mono.py / capture_chessboard.py / selftest_synthetic.py）不需要 ROS，"
  dim "    但图形界面和图像话题订阅需要。安装指引："
  dim "    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
  echo "    sudo apt install ros-humble-desktop"
fi
echo

echo "[2/4] 必需的系统包"
MISSING=()
for p in "${REQUIRED[@]}"; do
  if have "$p"; then
    ok "$p"
  else
    bad "$p"
    MISSING+=("$p")
  fi
done
echo

echo "[3/4] 推荐的系统包"
MISSING_OPT=()
for p in "${RECOMMENDED[@]}"; do
  if have "$p"; then
    ok "$p"
  else
    warn "$p（缺它界面中文可能显示成方框，程序仍可运行）"
    MISSING_OPT+=("$p")
  fi
done
echo

echo "[4/4] 可选：ROS 官方标定编排（ros2_calibrate.sh 用）"
for p in "${OPTIONAL[@]}"; do
  if have "$p"; then
    ok "$p"
  else
    warn "$p（只有 ./ros2_calibrate.sh run 这条路线需要，纯 GUI / 纯 OpenCV 不需要）"
    MISSING_OPT+=("$p")
  fi
done
echo

# --------------------------------------------------------------------------- #
if [[ "$MODE" == "--check" ]]; then
  echo "=============================================================="
  if [[ ${#MISSING[@]} -eq 0 ]]; then
    ok "必需依赖齐全，可以直接用。"
  else
    bad "缺少必需依赖：${MISSING[*]}"
    echo "    装它们跑：  ./install_deps.sh"
  fi
  [[ ${#MISSING_OPT[@]} -gt 0 ]] && dim "    可选未装：${MISSING_OPT[*]}"
  echo "=============================================================="
  [[ ${#MISSING[@]} -eq 0 ]]
  exit $?
fi

ALL=("${MISSING[@]}" "${MISSING_OPT[@]}")
if [[ ${#ALL[@]} -eq 0 ]]; then
  echo "=============================================================="
  ok "全部依赖已就绪，不需要安装。"
  echo "=============================================================="
  exit 0
fi

echo "准备安装：${ALL[*]}"
echo "（需要 sudo 权限；只装系统包，不动你的代码与标定结果）"
echo
if ! sudo apt update; then
  bad "apt update 失败，请检查网络或软件源"
  exit 1
fi
if sudo apt install -y "${ALL[@]}"; then
  echo
  echo "=============================================================="
  ok "安装完成。下一步："
  dim "    自检（不需要相机）：  python3 selftest_synthetic.py"
  dim "                         python3 calib_gui.py --self-test"
  dim "    图形界面：            ./run_gui.sh"
  echo "=============================================================="
else
  bad "安装失败，请把上面的报错发出来"
  exit 1
fi
