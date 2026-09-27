#!/usr/bin/env bash
# =============================================================================
# ros2_calibrate.sh —— 为本地三个相机驱动（海康 / 大恒 / 迈德威视）编排棋盘格标定
#
#   ./ros2_calibrate.sh list
#   ./ros2_calibrate.sh check   hik|galaxy|mv
#   ./ros2_calibrate.sh run     hik|galaxy|mv [--size 8x6] [--square 0.025] \
#                                              [--camera-name 名字] [--no-driver]
#   ./ros2_calibrate.sh install hik|galaxy|mv [calibrationdata.tar.gz]
#   ./ros2_calibrate.sh verify  hik|galaxy|mv [--size 8x6] [--square 0.025] [tar]
#
# run 会：起驱动 -> 自动探测图像话题 -> 起 cameracalibrator -> 结束后把
#        /tmp/calibrationdata.tar.gz 里的 ost.yaml 回写到驱动的 config/camera_info.yaml
# verify 会：从 tar 里的原始采集图独立复算 RMS/覆盖率，并与 ost.yaml 对比
#        （ROS 2 版 cameracalibrator 不打印 RMS，所以需要这一步）
# =============================================================================
set -o pipefail   # 注意：不能加 set -u，ROS 的 setup.bash 会引用未定义变量而中断

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 三个相机驱动的工作空间根目录：优先环境变量 CAMERA_ROOT，否则按常见位置自动探测
if [[ -z "${CAMERA_ROOT:-}" ]]; then
  for _c in "$HOME/下载/camera" "$HOME/Downloads/camera" "$HOME/camera" \
            "$HOME/camera_ws" "$HOME/ws/camera" "/opt/camera"; do
    [[ -d "$_c" ]] && { CAMERA_ROOT="$_c"; break; }
  done
fi
CAMERA_ROOT="${CAMERA_ROOT:-$HOME/下载/camera}"
# 所有结果集中落在这里，每次一个按时间命名的子目录（可用 CALIB_RESULTS_DIR 覆盖）
RESULTS_DIR="${CALIB_RESULTS_DIR:-$SCRIPT_DIR/标定结果}"
declare -A WS PKG EXE CFG NODE
WS[hik]="$CAMERA_ROOT/ros2-hik-camera-main";            PKG[hik]=hik_camera;        EXE[hik]=hik_camera_node;        CFG[hik]="config/camera_info.yaml"; NODE[hik]="/hik_camera"
WS[galaxy]="$CAMERA_ROOT/rm_vision_ros2_galaxy_camera-master"; PKG[galaxy]=galaxy_camera; EXE[galaxy]=galaxy_camera_node; CFG[galaxy]="config/camera_info.yaml"; NODE[galaxy]="/galaxy_camera"
WS[mv]="$CAMERA_ROOT/ros2_mindvision_camera";           PKG[mv]=mindvision_camera;  EXE[mv]=mindvision_camera_node;  CFG[mv]="config/camera_info.yaml";     NODE[mv]="/mv_camera"

C_OK=$'\033[32m'; C_ERR=$'\033[31m'; C_WARN=$'\033[33m'; C_DIM=$'\033[2m'; C_END=$'\033[0m'
say()  { printf '%s\n' "$*"; }
ok()   { printf '%s✔%s %s\n' "$C_OK" "$C_END" "$*"; }
err()  { printf '%s✘%s %s\n' "$C_ERR" "$C_END" "$*" >&2; }
warn() { printf '%s!%s %s\n' "$C_WARN" "$C_END" "$*"; }

# 新建一个按时间命名的会话目录：标定结果/<YYYYmmdd_HHMMSS>_<标签>/
# 同时更新 <标定结果>/latest 软链接，方便找最近一次
new_session() {
  local label=$1 dir
  mkdir -p "$RESULTS_DIR"
  dir="$RESULTS_DIR/$(date +%Y%m%d_%H%M%S)_${label}"
  mkdir -p "$dir"
  ln -sfn "$dir" "$RESULTS_DIR/latest"
  printf '%s' "$dir"
}

# 找最近一次会话目录（没有就返回空）
latest_session() {
  [[ -L "$RESULTS_DIR/latest" ]] && readlink -f "$RESULTS_DIR/latest"
}

# 写一份人可读的清单，记录这次用了什么参数
write_manifest() {
  local dir=$1; shift
  {
    echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "参数: $*"
  } > "$dir/说明.txt"
}

usage() {
  cat <<'EOF'
ros2_calibrate.sh —— 为本地三个相机驱动（海康 / 大恒 / 迈德威视）编排棋盘格标定

  ./ros2_calibrate.sh list
  ./ros2_calibrate.sh check   hik|galaxy|mv
  ./ros2_calibrate.sh run     hik|galaxy|mv [--size 8x6] [--square 0.025]
                                             [--camera-name 名字] [--no-driver]
  ./ros2_calibrate.sh install hik|galaxy|mv [calibrationdata.tar.gz]
  ./ros2_calibrate.sh verify  hik|galaxy|mv [--size 8x6] [--square 0.025] [tar]
  ./ros2_calibrate.sh results

所有产物按时间归档到: <脚本目录>/标定结果/<时间戳>_<驱动>_<尺寸>/
  内含 camera_info.yaml（最终内参）、ost.yaml、calibrationdata.tar.gz、
  frames/（采集原图）、复算报告.txt、旧内参备份、说明.txt
  <脚本目录>/标定结果/latest 是指向最近一次的软链接

run    : 起驱动 -> 探测图像话题 -> 起 cameracalibrator -> 结束后回写 camera_info.yaml
verify : 从 tar 里的原始采集图独立复算 RMS/覆盖率，并与 ost.yaml 对比
         （ROS 2 版 cameracalibrator 不打印 RMS，所以需要这一步）
EOF
  exit 1
}

driver_exists() { [[ -n "${WS[$1]:-}" ]] || { err "未知驱动 '$1'，可选：hik / galaxy / mv"; return 1; }; }

load_env() {   # 载入 ROS + 该驱动工作空间环境
  local d=$1
  # ROS 2 环境：优先已 source 的 ROS_DISTRO，其次 humble，最后 /opt/ros 下任意一个
  local ros_setup=""
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    ros_setup="/opt/ros/$ROS_DISTRO/setup.bash"
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ros_setup="/opt/ros/humble/setup.bash"
  else
    ros_setup="$(ls -1d /opt/ros/*/setup.bash 2>/dev/null | sort -r | head -1)"
  fi
  [[ -n "$ros_setup" ]] || { err "找不到任何 ROS 2 环境（/opt/ros/*/setup.bash）"; return 1; }
  # shellcheck disable=SC1090
  source "$ros_setup"
  if [[ -f "${WS[$d]}/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${WS[$d]}/install/setup.bash"
  else
    err "还没编译：${WS[$d]}/install 不存在。先在该目录执行 colcon build"
    return 1
  fi
}

exe_path() { echo "${WS[$1]}/install/${PKG[$1]}/lib/${PKG[$1]}/${EXE[$1]}"; }

cmd_list() {
  local d
  for d in hik galaxy mv; do
    local st="未编译"
    [[ -f "${WS[$d]}/install/setup.bash" ]] && st="已编译"
    printf '%-8s %-22s %-24s %s\n     %s\n' \
      "$d" "${PKG[$d]}" "$(basename "${WS[$d]}")" "$st" "${WS[$d]}"
  done
}

cmd_check() {
  local d=$1; driver_exists "$d" || return 1
  say "──────── 检查驱动 $d (${PKG[$d]}) ────────"
  local exe; exe="$(exe_path "$d")"
  [[ -f "$exe" ]] || { err "可执行文件不存在: $exe（需要在该工作空间 colcon build）"; return 1; }
  if [[ ! -x "$exe" ]]; then
    chmod +x "$exe" && ok "已修复缺失的可执行权限: $exe"
  else
    ok "可执行文件就绪"
  fi
  load_env "$d" || return 1
  ros2 pkg prefix "${PKG[$d]}" >/dev/null 2>&1 || { err "ament 找不到包 ${PKG[$d]}"; return 1; }
  ok "ament 已注册 ${PKG[$d]}"

  say "→ 启动节点 6 秒，检查 SDK 是否可用 ..."
  local log; log="$(mktemp)"
  # 让后台作业自成进程组（$! 即组长），退出时杀整组；
  # 只杀 ros2 run 外壳会留下真正的节点进程变野节点。
  set -m
  ros2 run "${PKG[$d]}" "${EXE[$d]}" >"$log" 2>&1 &
  local cpid=$!
  set +m
  sleep 6
  kill -TERM -- -"$cpid" 2>/dev/null
  sleep 1
  kill -KILL -- -"$cpid" 2>/dev/null
  wait "$cpid" 2>/dev/null
  local sdk_ok=0
  if grep -qE "camera count = [0-9]|device_count = [0-9]" "$log"; then
    sdk_ok=1
    ok "SDK 加载成功、枚举正常（没插相机也能确认 SDK 本身是好的）"
  fi
  if grep -qiE "GxIAPI failed|Init failed|初始化失败|MV_E_|dlopen|not found TL" "$log"; then
    err "SDK 初始化失败，诊断如下："
    grep -iE "GxIAPI failed|Init failed|MV_E_|not found TL|Unable to open file" "$log" | head -3 | sed 's/^/    /'
    sdk_ok=0
  fi
  if (( sdk_ok )); then
    if grep -qE "camera count = [1-9]" "$log"; then
      ok "检测到相机，可以直接标定：$0 run $d"
    else
      warn "没有检测到相机（插上相机、确认供电与 USB3 口）"
    fi
  elif ! grep -qiE "GxIAPI failed|Init failed" "$log"; then
    warn "输出不符合预期，原始日志："
    head -5 "$log" | sed 's/^/    /'
  fi
  rm -f "$log"
}

pick_image_topic() {   # 轮询等待相机节点发布图像话题，打印话题名
  local deadline=$((SECONDS + ${1:-20})) t
  while (( SECONDS < deadline )); do
    t="$(ros2 topic list 2>/dev/null | grep -E '/image_raw$' | head -1)"
    [[ -n "$t" ]] && { echo "$t"; return 0; }
    sleep 1
  done
  return 1
}

cmd_run() {
  local d=$1; shift; driver_exists "$d" || return 1
  local size=8x6 square=0.025 camname="" start_driver=1
  while (( $# )); do
    case $1 in
      --size)        size=$2; shift 2 ;;
      --square)      square=$2; shift 2 ;;
      --camera-name) camname=$2; shift 2 ;;
      --no-driver)   start_driver=0; shift ;;
      *) err "未知参数 $1"; return 1 ;;
    esac
  done
  load_env "$d" || return 1

  if ! ros2 pkg prefix camera_calibration >/dev/null 2>&1; then
    err "没装标定程序 camera_calibration，先执行："
    say "    sudo apt install -y ros-${ROS_DISTRO:-humble}-camera-calibration ros-${ROS_DISTRO:-humble}-image-pipeline ros-${ROS_DISTRO:-humble}-cv-bridge"
    return 1
  fi

  local drv_pid=""
  cleanup() {
    if [[ -n "$drv_pid" ]]; then
      kill -TERM -- -"$drv_pid" 2>/dev/null || kill "$drv_pid" 2>/dev/null
      sleep 0.5
      kill -KILL -- -"$drv_pid" 2>/dev/null
      say "已停止驱动节点（整个进程组）"
    fi
  }
  trap cleanup EXIT

  # 本次标定一个会话目录，所有产物都放进去
  local sess; sess="$(new_session "${d}_${size}")"
  write_manifest "$sess" "run $d --size $size --square $square"
  say "结果目录: $sess"

  if (( start_driver )); then
    say "→ 后台启动 ${PKG[$d]} ..."
    set -m
    ros2 run "${PKG[$d]}" "${EXE[$d]}" > "$sess/驱动日志.log" 2>&1 &
    drv_pid=$!
    set +m
    say "→ 等待图像话题（日志 $sess/驱动日志.log）..."
    local topic
    if ! topic="$(pick_image_topic 25)"; then
      err "25 秒内没等到图像话题，驱动日志尾部："
      tail -6 "$sess/驱动日志.log" | sed 's/^/    /'
      return 1
    fi
    ok "图像话题: $topic"
  else
    local topic
    topic="$(pick_image_topic 8)" || { err "没有找到 image_raw 话题，请先自己起驱动"; return 1; }
    ok "图像话题: $topic"
  fi

  # 驱动参数的 camera_name 决定 camera_info 的 set_camera_info 服务名前缀
  local cfg_cam
  cfg_cam="$(grep -E '^\s*camera_name:' "${WS[$d]}/${CFG[$d]}" 2>/dev/null | head -1 | awk '{print $2}')"
  local param_cam
  param_cam="$(grep -A5 -E "^${NODE[$d]}:" "${WS[$d]}/config/camera_params.yaml" 2>/dev/null | grep -E 'camera_name:' | head -1 | awk '{print $2}')"
  [[ -z "$camname" ]] && camname="${param_cam:-$cfg_cam}"

  say ""
  say "棋盘格：内角点 $size，方格 $square m，camera_name=$camname"
  say "标定窗口里把 X / Y / Size / Skew 四条都填绿，再点 CALIBRATE → SAVE"
  say ""
  ros2 run camera_calibration cameracalibrator \
      --size "$size" --square "$square" \
      -c "$camname" --no-service-check \
      --ros-args -r "image:=$topic" -r "camera:=/$camname"
  local rc=$?

  if [[ -f /tmp/calibrationdata.tar.gz ]]; then
    say ""
    ok "标定程序退出，开始归档并回写结果 ..."
    cp -f /tmp/calibrationdata.tar.gz "$sess/calibrationdata.tar.gz"
    cmd_install "$d" "$sess/calibrationdata.tar.gz" --session "$sess" --size "$size" --square "$square"
  else
    [[ $rc -ne 0 ]] && warn "标定程序退出码 $rc，且没有生成 /tmp/calibrationdata.tar.gz（没点 SAVE？）"
    say "  本次会话目录仍保留在: $sess"
  fi
}

cmd_install() {
  local d=$1; shift
  driver_exists "$d" || return 1
  local tarfile=/tmp/calibrationdata.tar.gz sess="" size=8x6 square=0.025
  while (( $# )); do
    case $1 in
      --session) sess=$2; shift 2 ;;
      --size)    size=$2; shift 2 ;;
      --square)  square=$2; shift 2 ;;
      -*)        err "未知参数 $1"; return 1 ;;
      *)         tarfile=$1; shift ;;
    esac
  done
  [[ -f "$tarfile" ]] || { err "找不到 $tarfile（先在标定窗口点 SAVE）"; return 1; }
  [[ -n "$sess" ]] || sess="$(new_session "${d}_${size}")"
  mkdir -p "$sess"
  local target="${WS[$d]}/${CFG[$d]}"

  # 1) 归档原始压缩包（如果它不在会话目录里）
  [[ "$(readlink -f "$tarfile")" == "$(readlink -f "$sess/calibrationdata.tar.gz" 2>/dev/null)" ]] \
    || cp -f "$tarfile" "$sess/calibrationdata.tar.gz"

  # 2) 解包：ost.yaml / ost.txt 放会话目录，采集图放 frames/
  local tmp; tmp="$(mktemp -d)"
  tar -xzf "$tarfile" -C "$tmp" 2>/dev/null
  local yaml; yaml="$(find "$tmp" -maxdepth 1 -name 'ost.yaml' | head -1)"
  if [[ -z "$yaml" ]]; then
    if [[ "$tarfile" == *.yaml ]]; then yaml="$tarfile"; else err "压缩包里没有 ost.yaml"; rm -rf "$tmp"; return 1; fi
  fi
  [[ -f "$tmp/ost.txt" ]] && cp -f "$tmp/ost.txt" "$sess/ost.txt"
  mkdir -p "$sess/frames"
  find "$tmp" -maxdepth 1 -name '*.png' -exec cp -f {} "$sess/frames/" \; 2>/dev/null
  local nframes; nframes="$(find "$sess/frames" -name '*.png' | wc -l)"

  local newfx oldfx
  newfx="$(grep -A4 'camera_matrix:' "$yaml" | grep -oE '\-?[0-9]+\.[0-9]+' | head -1)"
  oldfx="$(grep -A4 'camera_matrix:' "$target" 2>/dev/null | grep -oE '\-?[0-9]+\.[0-9]+' | head -1)"

  say "──────── 回写 ${PKG[$d]}/$(basename "${CFG[$d]}") ────────"
  say "  结果目录: $sess"
  say "  新标定: fx=$(printf '%.3f' "${newfx:-0}")  $(grep -E 'image_width|image_height' "$yaml" | tr '\n' ' ')"
  say "  原文件: fx=$(printf '%.3f' "${oldfx:-0}")  $(grep -E 'image_width|image_height' "$target" 2>/dev/null | tr '\n' ' ')"

  # 3) 旧内参备份：驱动旁边留一份 .bak，会话目录里也留一份好找的
  if [[ -f "$target" ]]; then
    cp -a "$target" "${target}.bak.$(date +%Y%m%d_%H%M%S)" 2>/dev/null \
      && say "  已备份原文件 -> $(basename "$target").bak.*（在驱动目录）"
    cp -a "$target" "$sess/旧内参_$(basename "${CFG[$d]}")"
  fi

  # 4) 写回驱动 + 会话目录归档 + ROS 约定目录
  local camname
  camname="$(grep -E '^\s*camera_name:' "$target" 2>/dev/null | head -1 | awk '{print $2}')"
  cp -f "$yaml" "$sess/ost.yaml"
  sed -E "s/^camera_name:.*/camera_name: ${camname:-camera}/" "$sess/ost.yaml" > "$target"
  cp -f "$target" "$sess/camera_info.yaml"
  ok "已写入 $target"
  say "  重启驱动后 camera_info 即生效（驱动启动时会读这个文件）"
  rm -rf "$tmp"
  mkdir -p "$HOME/.ros/camera_info"
  cp "$target" "$HOME/.ros/camera_info/${PKG[$d]}.yaml"
  say "  副本: ~/.ros/camera_info/${PKG[$d]}.yaml"

  write_manifest "$sess" "install $d --size $size --square $square"$'\n'"来源: $tarfile"$'\n'"采集图: $nframes 张"
  say ""
  say "  目录内容:"
  ls -1 "$sess" | sed 's/^/    /'
  say "  快速查看最近一次:  ls -l $RESULTS_DIR/latest"
}

# ---------------------------------------------------------------------------
# verify：ROS 2 版的 cameracalibrator 不打印 RMS（源码里 reproj_err 拿到就丢了），
#         所以这里从 SAVE 出来的图片独立复算 RMS / 覆盖率，并和 ost.yaml 对比。
# ---------------------------------------------------------------------------
cmd_verify() {
  local d=$1; shift; driver_exists "$d" || return 1
  local tarfile=/tmp/calibrationdata.tar.gz size=8x6 square=0.025 sess=""
  while (( $# )); do
    case $1 in
      --size)    size=$2; shift 2 ;;
      --square)  square=$2; shift 2 ;;
      --session) sess=$2; shift 2 ;;
      -*)        err "未知参数 $1"; return 1 ;;
      *)         tarfile=$1; shift ;;
    esac
  done
  # 没指定会话就跟着最近一次；连最近一次都没有就新建一个
  [[ -n "$sess" ]] || sess="$(latest_session)"
  [[ -n "$sess" && -d "$sess" ]] || sess="$(new_session "${d}_${size}_verify")"
  # 优先用会话目录里归档好的那个包
  [[ -f "$tarfile" ]] || tarfile="$sess/calibrationdata.tar.gz"
  [[ -f "$tarfile" ]] || { err "找不到 $tarfile（先在标定窗口点 SAVE）"; return 1; }

  mkdir -p "$sess"
  local report="$sess/复算报告.txt"
  say "  报告会存到: $report"
  _verify_run "$d" "$tarfile" "$size" "$square" | tee "$report"
}

# 真正干活的函数；输出由 cmd_verify 用 tee 存成报告
_verify_run() {
  local d=$1 tarfile=$2 size=$3 square=$4

  local tmp; tmp="$(mktemp -d)"
  tar -xzf "$tarfile" -C "$tmp" 2>/dev/null
  local n; n="$(find "$tmp" -maxdepth 1 -name '*.png' | wc -l)"
  if (( n == 0 )); then err "压缩包里没有图片，无法复算"; rm -rf "$tmp"; return 1; fi
  say "──────── 独立复算 ${PKG[$d]} 的标定质量 ────────"
  say "  素材: $tarfile 里的 $n 张图；棋盘格 $size，方格 $square m"
  say ""
  python3 "$SCRIPT_DIR/calibrate_mono.py" --images "$tmp" --size "$size" --square "$square" \
        --camera-name verify --output "$tmp/recomputed.yaml" --min-views 5 --k 2 2>&1 \
        | grep -vE "^\s*\[(成功|跳过)\]" | sed 's/^/  /'
  say ""

  local ost="$tmp/ost.yaml"
  if [[ -f "$ost" ]]; then
    python3 - "$ost" "$tmp/recomputed.yaml" <<'PY'
import sys, yaml
def load(p):
    d = yaml.safe_load(open(p))
    return (d,
            [float(x) for x in d["camera_matrix"]["data"]],
            [float(x) for x in d["distortion_coefficients"]["data"]])
try:
    d1, K1, D1 = load(sys.argv[1]); d2, K2, D2 = load(sys.argv[2])
except Exception as e:
    print("  对比跳过：%s" % e); sys.exit(0)
print("  cameracalibrator 与独立复算的对比（差值为独立复算 - cameracalibrator）")
print("  %-5s %16s %16s %12s" % ("参数", "cameracalibrator", "独立复算", "差值"))
for name, i in (("fx", 0), ("fy", 4), ("cx", 2), ("cy", 5)):
    a, b = K1[i], K2[i]
    print("  %-5s %16.3f %16.3f %12.3f" % (name, a, b, b - a))
print("  %-5s %16.6f %16.6f %12.6f" % ("k1", D1[0], D2[0], D2[0] - D1[0]))
if len(D1) > 1 and len(D2) > 1:
    print("  %-5s %16.6f %16.6f %12.6f" % ("k2", D1[1], D2[1], D2[1] - D1[1]))
print("  分辨率: cameracalibrator %dx%d | 独立复算 %dx%d"
      % (d1["image_width"], d1["image_height"], d2["image_width"], d2["image_height"]))
rel = max(abs(K2[i] - K1[i]) / abs(K1[i]) for i in (0, 4, 2, 5))
if rel < 0.005:
    print("  ✔ 两套结果一致（相对差 %.3f%% < 0.5%%），ost.yaml 可信" % (rel * 100))
elif rel < 0.02:
    print("  ! 两套结果略有差异（相对差 %.3f%%），属正常范围" % (rel * 100))
else:
    print("  ✘ 两套结果差异较大（相对差 %.3f%%），建议补拍更多姿态重标" % (rel * 100))
PY
  else
    warn "压缩包里没有 ost.yaml，跳过对比"
  fi
  say ""
  say "  判据：RMS < 0.5 px 合格（< 0.3 优秀）；角点覆盖 > 40%；fx/fy 比值在 0.99~1.01"
  say "  （RMS 由 OpenCV 重投影误差给出，正是 ROS 2 界面不显示的那个数）"
  rm -rf "$tmp"
}

# 列出所有标定会话（按时间倒序），并抽取每个会话的 fx / RMS 方便对比
cmd_results() {
  [[ -d "$RESULTS_DIR" ]] || { say "还没有任何结果目录（$RESULTS_DIR）"; return 0; }
  say "结果根目录: $RESULTS_DIR"
  say ""
  local d fx rms n shown=0
  for d in $(ls -1dt "$RESULTS_DIR"/*/ 2>/dev/null); do
    [[ "$(basename "$d")" == latest* ]] && continue      # 跳过软链接
    fx="-"; rms="-"; n="-"
    if [[ -f "$d/camera_info.yaml" ]]; then
      fx="$(grep -A4 'camera_matrix:' "$d/camera_info.yaml" | grep -oE '\-?[0-9]+\.[0-9]+' | head -1)"
    fi
    if [[ -f "$d/复算报告.txt" ]]; then
      rms="$(grep -oE '整体 RMS 重投影误差: [0-9.]+' "$d/复算报告.txt" | head -1 | awk '{print $NF}')"
      [[ -z "$rms" ]] && rms="-"
    fi
    [[ -d "$d/frames" ]] && n="$(find "$d/frames" -name '*.png' 2>/dev/null | wc -l)"
    printf '  %-40s fx=%-12s RMS=%-8s 采集图=%s\n' "$(basename "$d")" "$fx" "$rms" "$n"
    shown=$(( shown + 1 ))
  done
  if (( shown == 0 )); then
    say "  （还没有标定会话）"
    say ""
    say "  跑一次标定即可:  ./ros2_calibrate.sh run hik --size 11x8 --square 0.025"
    return 0
  fi
  say ""
  say "  看最近一次:  ls -l $RESULTS_DIR/latest"
  say "  看某次报告:  cat $RESULTS_DIR/latest/复算报告.txt"
}

case "${1:-}" in
  results) cmd_results ;;
  list)    cmd_list ;;
  check)   shift; [[ $# -ge 1 ]] || usage; cmd_check "$1" ;;
  run)     shift; [[ $# -ge 1 ]] || usage; cmd_run "$@" ;;
  install) shift; [[ $# -ge 1 ]] || usage; cmd_install "$@" ;;
  verify)  shift; [[ $# -ge 1 ]] || usage; cmd_verify "$@" ;;
  *)       usage ;;
esac
