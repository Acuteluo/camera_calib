#!/usr/bin/env bash
# 在真实桌面或 Xvfb 上验证 ROS 订阅→采集→标定→检验→保存。
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/runtime.sh"
calib_base_env
calib_ros_env || { echo '请先安装 ROS 2。' >&2; exit 1; }
OUT="${1:-$(mktemp -d /tmp/calib_e2e_XXXXXX)}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
# 使用独立 ROS domain 和话题，避免真实相机的数据干扰。
export ROS_DOMAIN_ID="${CALIB_TEST_DOMAIN:-87}"
export ROS_LOCALHOST_ONLY=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
READY="$(mktemp)"
rm -f "$READY"
cleanup() {
  kill "${PUB_PID:-}" 2>/dev/null || true
  wait "${PUB_PID:-}" 2>/dev/null || true
  rm -f "$READY"
}
trap cleanup EXIT
"$CALIB_PYTHON" -s "$HERE/demo_camera.py" --ready-file "$READY" --duration 100 >"$OUT/publisher.log" 2>&1 &
PUB_PID=$!
for (( i=0; i<60; i++ )); do
  [[ -s "$READY" ]] && break
  kill -0 "$PUB_PID" 2>/dev/null || { cat "$OUT/publisher.log"; exit 1; }
  sleep 1
done
[[ -s "$READY" ]] || { echo '合成相机启动超时'; exit 1; }
timeout 85 "$HERE/run_gui.sh" --demo --topic /calib_demo/image_raw --size 8x6 --square 0.025 \
  --demo-calibrate-after 35 --demo-exit-after 55 --results-dir "$OUT/results" \
  --ace-template "$HERE/reference_json/uav.json" >"$OUT/gui.log" 2>&1 || {
    cat "$OUT/gui.log"; exit 1;
  }
"$CALIB_PYTHON" -s "$HERE/tests/verify_saved.py" "$OUT/results/latest" "$HERE/reference_json/uav.json"
echo "桌面全链路验证通过，日志和产物：$OUT"
