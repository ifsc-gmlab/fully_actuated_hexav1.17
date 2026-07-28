#!/bin/bash
# PX4 Custom HITL & Multi-Drone Simulation changes packing script (Enhanced with Gazebo Submodule Assets)
# Run this script at the root of the PX4-Autopilot repository to generate a single portable bundle.

set -euo pipefail

# Directories
WORKSPACE="$(pwd)"
MIG_DIR="${WORKSPACE}/px4_custom_migration"
PATCHES_DIR="${MIG_DIR}/patches"
FILES_DIR="${MIG_DIR}/files"

echo "[1/6] Cleaning up any previous migration directories..."
rm -rf "${MIG_DIR}"
rm -f px4_custom_sim_migration.tar.gz

mkdir -p "${PATCHES_DIR}"
mkdir -p "${FILES_DIR}/submodule_gz/params"

echo "[2/6] Generating Git patch for the main PX4-Autopilot repository..."
# Export the latest custom commit (b8730bbcebc9aa0cad227278c48af0a4e612b62e)
git format-patch -1 HEAD -o "${PATCHES_DIR}"
# Find the generated patch file and rename it to a friendly name
PATCH_NAME=$(find "${PATCHES_DIR}" -name "*.patch" | head -n 1)
mv "${PATCH_NAME}" "${PATCHES_DIR}/0001-custom-hitl-and-multidrone-simulation.patch"

echo "[3/6] Packaging submodule (Tools/simulation/gz) modifications & assets..."
SUBMODULE_GZ_DIR="${WORKSPACE}/Tools/simulation/gz"
if [ -d "${SUBMODULE_GZ_DIR}" ]; then
    cd "${SUBMODULE_GZ_DIR}"
    # Use --binary to capture binary assets (e.g. models/arucotag/*.png) and complete custom worlds/models
    # We diff against the official base commit (e05f4312d3f28aa621157610584a4870406cb6d3) which is tag v1.16.2's original pointer
    git diff --binary e05f4312d3f28aa621157610584a4870406cb6d3 > "${PATCHES_DIR}/submodule_gz_changes.patch"
    echo "  Generated binary patch for Gazebo assets (worlds, models, config files)."
    
    # Copy any untracked camera_info bridge yaml files if they exist
    UNTRACKED_YAML="${SUBMODULE_GZ_DIR}/params/x500_mono_cam_aruco_camera_info_bridge.yaml"
    if [ -f "${UNTRACKED_YAML}" ]; then
        cp "${UNTRACKED_YAML}" "${FILES_DIR}/submodule_gz/params/"
        echo "  Copied untracked file: ${UNTRACKED_YAML}"
    fi
    cd "${WORKSPACE}"
else
    echo "  [Warning] Submodule Tools/simulation/gz not found or not checked out."
fi

echo "[4/6] Creating the automatic installer script (install_migration.sh)..."
cat << 'EOF' > "${MIG_DIR}/install_migration.sh"
#!/bin/bash
# Auto-installer for Custom HITL & Multi-Drone Simulation Setup
# Place this script in the root of the target PX4-Autopilot repository and run it.

set -euo pipefail

WORKSPACE="$(pwd)"

if [ ! -f "CMakeLists.txt" ] || [ ! -d "src/modules" ]; then
    echo "[Error] You are not in the root directory of a PX4-Autopilot repository!"
    exit 1
fi

echo "[1/4] Checking current branch and git cleanliness..."
if ! git diff-index --quiet HEAD --; then
    echo "[Warning] You have unstaged changes in your repository. We suggest stashing them first."
    read -p "Do you want to continue applying the migration anyway? [y/N]: " choice
    if [[ ! "$choice" =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
fi

echo "[2/4] Initializing and checking out Tools/simulation/gz submodule..."
if [ -d "Tools/simulation/gz" ]; then
    # Make sure submodule is initialized and checked out to the official pointer
    git submodule update --init --recursive Tools/simulation/gz
else
    echo "[Error] Tools/simulation/gz submodule directory not found!"
    exit 1
fi

echo "[3/4] Applying main PX4-Autopilot git patch..."
if [ -f "patches/0001-custom-hitl-and-multidrone-simulation.patch" ]; then
    # We apply the main patch but exclude the submodule commit pointer change,
    # because we will manage the submodule contents locally via the submodule patch.
    # This prevents git apply from complaining about unrecognized submodule commits.
    if git apply --check --exclude="Tools/simulation/gz" "patches/0001-custom-hitl-and-multidrone-simulation.patch" 2>/dev/null; then
        git apply --exclude="Tools/simulation/gz" "patches/0001-custom-hitl-and-multidrone-simulation.patch"
        echo "  [Success] Applied main repository patch successfully!"
    else
        echo "  [Notice] Standard patch application failed validation. Attempting a 3-way merge apply..."
        # If standard apply fails, we commit it or use git am, but exclude submodule
        if git am --exclude="Tools/simulation/gz" -3 "patches/0001-custom-hitl-and-multidrone-simulation.patch"; then
            echo "  [Success] Applied main patch via 3-way merge!"
        else
            echo "  [Error] Failed to apply main patch. Please merge manually."
            exit 1
        fi
    fi
else
    echo "[Error] Main patch file not found!"
    exit 1
fi

echo "[4/4] Applying Gazebo Harmonic simulation assets and configurations..."
# Apply the binary patch to the submodule
if [ -f "patches/submodule_gz_changes.patch" ] && [ -s "patches/submodule_gz_changes.patch" ]; then
    cd Tools/simulation/gz
    if git apply --check "../../patches/submodule_gz_changes.patch" 2>/dev/null; then
        git apply "../../patches/submodule_gz_changes.patch"
        echo "  [Success] Applied Gazebo simulation assets patch successfully (including worlds & models)!"
    else
        echo "  [Warning] Gazebo submodule patch check failed. Attempting force application..."
        if git apply --reject "../../patches/submodule_gz_changes.patch" 2>/dev/null || git apply "../../patches/submodule_gz_changes.patch"; then
            echo "  [Success] Applied Gazebo assets patch!"
        else
            echo "  [Warning] Failed to apply Gazebo submodule patch automatically. You might need to manually apply it."
        fi
    fi
    cd "${WORKSPACE}"
fi

# Copy any untracked parameter files
if [ -f "files/submodule_gz/params/x500_mono_cam_aruco_camera_info_bridge.yaml" ]; then
    mkdir -p Tools/simulation/gz/params
    cp -v files/submodule_gz/params/x500_mono_cam_aruco_camera_info_bridge.yaml Tools/simulation/gz/params/
    echo "  [Success] Copied camera_info yaml bridge config!"
fi

echo ""
echo "=========================================================="
echo "🎉 Custom HITL & Multi-Drone Simulation successfully ported!"
echo "=========================================================="
echo "Next Steps to Compile:"
echo "1. Build the PX4 main simulator target (SITL):"
echo "   make px4_sitl_default"
echo ""
echo "2. Build the hardware-in-the-loop (HITL) bridge:"
echo "   cd Tools/simulation/gz_hitl_bridge"
echo "   cmake -B build -S ."
echo "   cmake --build build -j\$(nproc)"
echo ""
echo "3. Update your flight controller default.px4board config and flash (if doing HITL):"
echo "   Add 'CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y' to boards/cuav/7-nano/default.px4board"
echo "   make cuav_7-nano_default upload"
echo "=========================================================="
EOF
chmod +x "${MIG_DIR}/install_migration.sh"

echo "[5/6] Creating a comprehensive Markdown README inside the bundle..."
cat << 'EOF' > "${MIG_DIR}/README.md"
# PX4 Custom HITL & Multi-Drone Simulation Setup - Porting Bundle

This directory contains everything you need to completely replicate and port our custom Gazebo Harmonic HITL (Hardware-In-The-Loop) and Multi-Drone simulation setups (including custom worlds, models, and assets).

## 📂 Bundle Structure
- `install_migration.sh`: Automatically runs in a target `PX4-Autopilot` repository to apply the patches and setup files.
- `patches/0001-custom-hitl-and-multidrone-simulation.patch`: Git patch for the main repository including:
  - `gz_hitl_bridge` tool source code, CMake, Python launcher, and YAML presets.
  - Multi-drone SITL launcher scripts (`launch_mono_sim.sh`, `launch_dual_sim.sh`, `dual_x500_dual_sim.sh`, `kill_dual_sim.sh`).
  - Airframes and configuration adjustments for DDS / uXRCE throttling.
- `patches/submodule_gz_changes.patch`: **Enhanced binary patch** for the `Tools/simulation/gz` submodule. Contains:
  - Custom Worlds: `x500_rigid_pair.sdf`, `aruco.sdf`, `default.sdf`.
  - Custom Models: `models/x500_dual/model.sdf` (dual quad rigid connected configuration), `models/x500_arucotag/model.sdf`, and Aruco tag image assets (`.png`).
  - Bridge mappings for ROS 2 and Gazebo.
- `files/`: Contains untracked camera_info bridge parameters.

## 🚀 Quick Install (for the recipient)
1. Copy the compressed `.tar.gz` bundle to the target `PX4-Autopilot` directory.
2. Extract the archive:
   ```bash
   tar -xzvf px4_custom_sim_migration.tar.gz
   cd px4_custom_migration
   ```
3. Move/copy all files inside `px4_custom_migration` into the parent `PX4-Autopilot` folder:
   ```bash
   mv * ..
   cd ..
   ```
4. Run the installer:
   ```bash
   ./install_migration.sh
   ```
5. Clean up installer files once complete:
   ```bash
   rm -rf patches files install_migration.sh README.md
   ```
EOF

echo "[6/6] Archiving everything into 'px4_custom_sim_migration.tar.gz'..."
tar -czf px4_custom_sim_migration.tar.gz -C "${WORKSPACE}" px4_custom_migration

rm -rf "${MIG_DIR}"

echo ""
echo "=========================================================="
echo "🎉 Success! The migration bundle is ready!"
echo "Package File: /home/ubuntu22/PX4-Autopilot/px4_custom_sim_migration.tar.gz"
echo "=========================================================="
echo "You can now send this single tar.gz package to anyone."
echo "They only need to extract it in their PX4-Autopilot root folder"
echo "and run './install_migration.sh' to get all the features."
echo "=========================================================="
