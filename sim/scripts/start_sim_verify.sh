#!/usr/bin/env bash
set -e

export DAEDALUS_MAX_FPS="${DAEDALUS_MAX_FPS:-60}"
export DAEDALUS_WEB_PORT="${DAEDALUS_WEB_PORT:-9001}"
export DAEDALUS_AUTO_AIM_DEFAULT="${DAEDALUS_AUTO_AIM_DEFAULT:-1}"

PIDS=()
cleanup() {
    echo ""
    echo "[sim_verify] Stopping all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

"$HOME/bevy_robomaster_simulator/run_sim.sh" > /tmp/daedalus_sim.log 2>&1 &
PIDS+=($!)

for i in $(seq 1 40); do
    if [ -f /tmp/talos_ipc_meta ] && [ -f /tmp/talos_ipc_image_pool ]; then
        break
    fi
    sleep 0.25
done
sleep 1

echo "============================================================"
echo " Daedalus ?????: http://192.168.1.50:${DAEDALUS_WEB_PORT}"
echo " ????           : ${DAEDALUS_MAX_FPS} FPS"
echo "============================================================"

WORK_DIR="$HOME/awakening_sentry"
export VISION_ROOT="$WORK_DIR"
export MVCAM_SDK_PATH=/home/nuc/pb2025_sentry_ws-main/src/dependencies/hik_camera_ros2_driver/hikSDK
export LD_LIBRARY_PATH="$MVCAM_SDK_PATH/lib/amd64:$WORK_DIR/lib:$WORK_DIR/build:${LD_LIBRARY_PATH:-}"
cd "$WORK_DIR"
"$WORK_DIR/bin/standard" config/sentry_sim.yaml true
