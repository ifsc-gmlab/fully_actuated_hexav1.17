#!/usr/bin/env python3
"""
One-command launcher for gz_hitl_bridge.

Reads a single YAML config file and:
  1. Starts Gazebo with the chosen world
  2. Spawns each vehicle's SDF model at its configured pose
  3. Execs the C++ bridge with the same YAML

Usage:
  ./launch.py configs/swarm_3x_quad.yaml
  ./launch.py configs/single_x500.yaml
  ./launch.py configs/vtol.yaml

YAML extensions over the base bridge schema:
  vehicles:
    - name: drone_0
      model: x500_0           # name to spawn AS (must be unique per vehicle)
      sdf_model: x500         # which SDF folder under models/ (default: same as `model`)
      pose: {x: 0, y: 0, z: 0.3, yaw: 0}    # all optional, default z=0.3
      ... other bridge fields ...

  world: default              # top-level, default "default"

Press Ctrl-C once to stop everything cleanly.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.stderr.write("ERROR: PyYAML required.  sudo apt install python3-yaml\n")
    sys.exit(2)

THIS_DIR = Path(__file__).resolve().parent
PX4_ROOT = THIS_DIR.parent.parent.parent
GZ_MODELS = PX4_ROOT / "Tools/simulation/gz/models"
GZ_WORLDS = PX4_ROOT / "Tools/simulation/gz/worlds"
GZ_PLUGINS = PX4_ROOT / "build/px4_sitl_default/src/modules/simulation/gz_plugins"
SERVER_CONFIG = PX4_ROOT / "Tools/simulation/gz/server.config"
BRIDGE_BIN = THIS_DIR / "build/gz_hitl_bridge"


def fail(msg):
    sys.stderr.write(f"ERROR: {msg}\n")
    sys.exit(1)


def setup_gz_env():
    """Mirror what start_gz.sh sets — gz needs to find models, plugins, server.config."""
    env = os.environ.copy()
    env["PX4_GZ_MODELS"] = str(GZ_MODELS)
    env["PX4_GZ_WORLDS"] = str(GZ_WORLDS)
    env["PX4_GZ_PLUGINS"] = str(GZ_PLUGINS)
    sep = ":"
    env["GZ_SIM_RESOURCE_PATH"] = sep.join(
        filter(None, [env.get("GZ_SIM_RESOURCE_PATH", ""), str(GZ_MODELS), str(GZ_WORLDS)])
    )
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = sep.join(
        filter(None, [env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""), str(GZ_PLUGINS)])
    )
    if SERVER_CONFIG.is_file():
        env["GZ_SIM_SERVER_CONFIG_PATH"] = str(SERVER_CONFIG)
    return env


def wait_for_world(world_name, timeout_s=30):
    """Poll for the gz `/world/<name>/scene/info` service to become available."""
    for i in range(timeout_s):
        r = subprocess.run(
            ["gz", "service", "-i", "--service", f"/world/{world_name}/scene/info"],
            capture_output=True, text=True
        )
        if "Service providers" in (r.stdout + r.stderr):
            print(f"[launch] gz world '{world_name}' ready after {i+1}s")
            return True
        time.sleep(1)
    return False


def spawn_model(world, sdf_model, instance_name, pose, env):
    """Spawn one SDF model into the running Gz world at the given pose."""
    sdf_path = GZ_MODELS / sdf_model / "model.sdf"
    if not sdf_path.is_file():
        fail(f"SDF not found for sdf_model='{sdf_model}' at {sdf_path}")

    x = pose.get("x", 0.0)
    y = pose.get("y", 0.0)
    z = pose.get("z", 0.3)
    yaw = pose.get("yaw", 0.0)
    # gz EntityFactory: pose is gz::msgs::Pose, position+orientation (quaternion)
    # For yaw only: quat = (0, 0, sin(yaw/2), cos(yaw/2))
    import math
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    req = (
        f'sdf_filename: "{sdf_path}", '
        f'name: "{instance_name}", '
        f'allow_renaming: false, '
        f'pose: {{'
        f' position: {{ x: {x}, y: {y}, z: {z} }}, '
        f' orientation: {{ x: 0, y: 0, z: {qz}, w: {qw} }} '
        f'}}'
    )

    print(f"[launch] spawning '{instance_name}' (sdf={sdf_model}) at x={x} y={y} z={z} yaw={yaw}")
    r = subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/create",
         "--reqtype", "gz.msgs.EntityFactory",
         "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000",
         "--req", req],
        capture_output=True, text=True, env=env
    )
    out = r.stdout + r.stderr
    if "data: true" not in out:
        sys.stderr.write(f"[launch] WARN spawn '{instance_name}' may have failed:\n{out}\n")
        return False
    return True


def main():
    ap = argparse.ArgumentParser(description="One-command launcher for gz_hitl_bridge")
    ap.add_argument("config", help="path to YAML config file")
    ap.add_argument("--headless", action="store_true",
                    help="run Gazebo without the GUI")
    ap.add_argument("--skip-gz", action="store_true",
                    help="assume Gazebo is already running with models spawned, just start bridge")
    ap.add_argument("--gz-only", action="store_true",
                    help="start Gazebo and spawn models, but do NOT start the bridge")
    args = ap.parse_args()

    cfg_path = Path(args.config).resolve()
    if not cfg_path.is_file():
        fail(f"config file not found: {cfg_path}")
    if not BRIDGE_BIN.is_file():
        fail(f"bridge not built — run: cmake --build {THIS_DIR/'build'} -j")

    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    world = cfg.get("world") or cfg.get("defaults", {}).get("world", "default")
    vehicles = cfg.get("vehicles", [])
    if not vehicles:
        fail("config has no 'vehicles' list")

    print(f"[launch] config: {cfg_path}")
    print(f"[launch] world:  {world}")
    print(f"[launch] vehicles: {len(vehicles)}")

    gz_proc = None
    env = setup_gz_env()

    if not args.skip_gz:
        # ---- 1. Start Gazebo ----
        world_sdf = GZ_WORLDS / f"{world}.sdf"
        if not world_sdf.is_file():
            fail(f"world file not found: {world_sdf}")

        gz_cmd = ["gz", "sim", "-r", "-v", "1", str(world_sdf)]
        if args.headless:
            gz_cmd.insert(2, "-s")  # server-only mode
        print(f"[launch] starting gz: {' '.join(gz_cmd)}")
        gz_proc = subprocess.Popen(gz_cmd, env=env)

        # ---- 2. Wait for world ----
        if not wait_for_world(world):
            gz_proc.terminate()
            fail(f"timed out waiting for world '{world}'")

        # ---- 3. Spawn each model ----
        for v in vehicles:
            name = v.get("name", "drone")
            instance_name = v.get("model", name)
            sdf_model = v.get("sdf_model", instance_name)
            # If `sdf_model` is not set but `model` looks like "x500_0", strip the
            # _N suffix to find the SDF folder.
            if not v.get("sdf_model"):
                if "_" in instance_name and instance_name.rsplit("_", 1)[1].isdigit():
                    sdf_model = instance_name.rsplit("_", 1)[0]
            pose = v.get("pose", {})
            spawn_model(world, sdf_model, instance_name, pose, env)

    if args.gz_only:
        print("[launch] --gz-only: Gz running. Press Ctrl-C to stop.")
        try:
            if gz_proc:
                gz_proc.wait()
        except KeyboardInterrupt:
            if gz_proc:
                gz_proc.terminate()
        return

    # ---- 4. Start bridge ----
    print(f"[launch] starting bridge: {BRIDGE_BIN} --config {cfg_path}")
    bridge_cmd = [str(BRIDGE_BIN), "--config", str(cfg_path)]
    bridge_proc = subprocess.Popen(bridge_cmd)

    # ---- 5. Wait — propagate signals to both subprocesses ----
    def shutdown(signum, frame):
        print(f"\n[launch] received signal {signum}, stopping...")
        if bridge_proc.poll() is None:
            bridge_proc.terminate()
        if gz_proc and gz_proc.poll() is None:
            gz_proc.terminate()

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    bridge_proc.wait()
    if gz_proc:
        gz_proc.wait()


if __name__ == "__main__":
    main()
