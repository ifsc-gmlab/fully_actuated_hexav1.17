#!/usr/bin/env bash
# Start Gazebo and spawn a model for HITL — replaces `make px4_sitl gz_x500`.
# Does NOT start any PX4 SITL process; the real flight controller is the brain.
#
# Usage:
#   ./start_gz.sh                       # default: world=default, model=x500
#   ./start_gz.sh forest x500           # custom world
#   WORLD=baylands MODEL=x500_depth ./start_gz.sh
#
# Press Ctrl-C to stop Gazebo.

set -e

WORLD="${1:-${WORLD:-default}}"
MODEL="${2:-${MODEL:-x500}}"
POS_X="${POS_X:-0}"
POS_Y="${POS_Y:-0}"
POS_Z="${POS_Z:-0.3}"

# PX4-Autopilot root (resolves regardless of where script is called from)
PX4_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
export PX4_GZ_MODELS="${PX4_ROOT}/Tools/simulation/gz/models"
export PX4_GZ_WORLDS="${PX4_ROOT}/Tools/simulation/gz/worlds"
export PX4_GZ_PLUGINS="${PX4_ROOT}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:${PX4_GZ_PLUGINS}"

# Server config: loads IMU/mag/baro/navsat sensor systems. Without this, those
# topics exist in the namespace but never publish data (because the sensor
# system plugins are not loaded).
SERVER_CONFIG="${PX4_ROOT}/Tools/simulation/gz/server.config"
if [ -f "${SERVER_CONFIG}" ]; then
    export GZ_SIM_SERVER_CONFIG_PATH="${SERVER_CONFIG}"
fi

WORLD_SDF="${PX4_GZ_WORLDS}/${WORLD}.sdf"
if [ ! -f "${WORLD_SDF}" ]; then
    echo "ERROR: world file not found: ${WORLD_SDF}"
    exit 1
fi
MODEL_SDF="${PX4_GZ_MODELS}/${MODEL}/model.sdf"
if [ ! -f "${MODEL_SDF}" ]; then
    echo "ERROR: model file not found: ${MODEL_SDF}"
    exit 1
fi

echo "[start_gz] PX4 root:  ${PX4_ROOT}"
echo "[start_gz] world:     ${WORLD}"
echo "[start_gz] model:     ${MODEL} (spawn name = ${MODEL})"
echo "[start_gz] pose:      x=${POS_X} y=${POS_Y} z=${POS_Z}"
echo

# Start Gazebo server + GUI in background
gz sim -r -v 1 "${WORLD_SDF}" &
GZ_PID=$!

cleanup() {
    echo
    echo "[start_gz] stopping gazebo (pid=${GZ_PID})..."
    kill ${GZ_PID} 2>/dev/null || true
    wait ${GZ_PID} 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# Wait for world to be ready (the /world/<name>/scene/info service appears)
echo "[start_gz] waiting for Gazebo world to come up..."
for i in {1..30}; do
    if gz service -i --service "/world/${WORLD}/scene/info" 2>&1 | grep -q "Service providers"; then
        echo "[start_gz] world ready after ${i}s"
        break
    fi
    sleep 1
    if [ ${i} -eq 30 ]; then
        echo "ERROR: timed out waiting for world '${WORLD}'"
        cleanup
    fi
done

# Spawn the model
echo "[start_gz] spawning ${MODEL}..."
gz service -s "/world/${WORLD}/create" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"${MODEL_SDF}\", name: \"${MODEL}\", pose: {position: {x: ${POS_X}, y: ${POS_Y}, z: ${POS_Z}}}"

echo
echo "[start_gz] ✅ ready. Now in another terminal run:"
echo "    ${PX4_ROOT}/Tools/simulation/gz_hitl_bridge/build/gz_hitl_bridge \\"
echo "        --device /dev/ttyACM0 --baud 921600 \\"
echo "        --world ${WORLD} --model ${MODEL}"
echo
echo "[start_gz] Ctrl-C here to stop Gazebo."

# Wait for Gazebo to exit (or get killed by Ctrl-C)
wait ${GZ_PID}
