#!/usr/bin/env python3
"""Structural contract for the isolated six-actuator flying-car GZ model."""

from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET


MODEL_PATH = Path(__file__).with_name("model.sdf")
REPO_ROOT = MODEL_PATH.parents[4]
AIRFRAME_PATH = REPO_ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes/80003_gz_flying_car"
GZ_STARTUP_PATH = REPO_ROOT / "ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim"
GZ_ENV_PATH = REPO_ROOT / "src/modules/simulation/gz_bridge/gz_env.sh.in"


class FlyingCarModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(MODEL_PATH).getroot()
        cls.model = cls.root.find("model")
        if cls.model is None:
            raise AssertionError("model.sdf must contain one model")

    def test_has_four_rotors_and_two_wheel_joints(self):
        joint_names = {joint.get("name") for joint in self.model.findall("joint")}
        self.assertTrue({f"rotor_{index}_joint" for index in range(4)} <= joint_names)
        self.assertTrue({"left_wheel_joint", "right_wheel_joint"} <= joint_names)

    def test_wheels_have_collision_and_friction(self):
        for side in ("left", "right"):
            link = self.model.find(f"link[@name='{side}_wheel']")
            self.assertIsNotNone(link, f"missing {side} wheel link")
            collision = link.find("collision")
            self.assertIsNotNone(collision, f"missing {side} wheel collision")
            self.assertIsNotNone(collision.find("surface/friction/ode"),
                                 f"missing {side} wheel friction")

    def test_has_imu_and_navsat(self):
        sensor_types = {sensor.get("type") for sensor in self.model.findall(".//sensor")}
        self.assertTrue({"imu", "navsat"} <= sensor_types)

    def test_rotor_plugins_use_four_ordered_actuator_channels(self):
        plugins = [plugin for plugin in self.model.findall("plugin")
                   if plugin.get("name") == "gz::sim::systems::MulticopterMotorModel"]
        self.assertEqual(4, len(plugins))
        self.assertEqual([f"rotor_{index}_joint" for index in range(4)],
                         [plugin.findtext("jointName") for plugin in plugins])
        self.assertEqual([0, 1, 2, 3],
                         [int(plugin.findtext("motorNumber")) for plugin in plugins])
        self.assertEqual(["command/motor_speed"] * 4,
                         [plugin.findtext("commandSubTopic") for plugin in plugins])

    def test_wheel_plugins_use_two_ordered_actuator_channels(self):
        plugins = [plugin for plugin in self.model.findall("plugin")
                   if plugin.get("name") == "gz::sim::systems::JointController"
                   and plugin.findtext("joint_name") in
                   {"left_wheel_joint", "right_wheel_joint"}]
        self.assertEqual(2, len(plugins))
        self.assertEqual(["left_wheel_joint", "right_wheel_joint"],
                         [plugin.findtext("joint_name") for plugin in plugins])
        self.assertEqual([0, 1],
                         [int(plugin.findtext("actuator_number")) for plugin in plugins])
        self.assertEqual(["command/motor_speed"] * 2,
                         [plugin.findtext("sub_topic") for plugin in plugins])
        self.assertEqual(["true"] * 2,
                         [plugin.findtext("use_actuator_msg") for plugin in plugins])

    def test_logical_motor_indices_are_unique_and_ordered(self):
        rotor_indices = [int(plugin.findtext("motorNumber")) for plugin in
                         self.model.findall("plugin")
                         if plugin.get("name") == "gz::sim::systems::MulticopterMotorModel"]
        wheel_indices = [4 + int(plugin.findtext("actuator_number")) for plugin in
                         self.model.findall("plugin")
                         if plugin.get("name") == "gz::sim::systems::JointController"
                         and plugin.findtext("joint_name") in
                         {"left_wheel_joint", "right_wheel_joint"}]
        self.assertEqual(list(range(6)), rotor_indices + wheel_indices)
        self.assertEqual(6, len(set(rotor_indices + wheel_indices)))

    def test_posix_airframe_routes_rotors_and_wheels_to_separate_interfaces(self):
        airframe = AIRFRAME_PATH.read_text(encoding="utf-8")
        for channel, function in enumerate(range(101, 105), start=1):
            self.assertIn(f"SIM_GZ_EC_FUNC{channel} {function}", airframe)
        for channel, function in enumerate(range(105, 107), start=1):
            self.assertIn(f"SIM_GZ_WH_FUNC{channel} {function}", airframe)
        self.assertNotIn("SIM_GZ_EC_FUNC5", airframe)
        self.assertNotIn("SIM_GZ_EC_FUNC6", airframe)

    def test_gz_startup_normalizes_only_flying_car_model_name(self):
        startup = GZ_STARTUP_PATH.read_text(encoding="utf-8")
        self.assertIn('if [ "${PX4_SIM_MODEL}" = "flying_car" ]; then', startup)
        self.assertIn('PX4_SIM_MODEL="gz_flying_car"', startup)

    def test_custom_resource_root_does_not_occupy_gz_submodule_path(self):
        gitmodules = (REPO_ROOT / ".gitmodules").read_text(encoding="utf-8")
        self.assertIn("path = Tools/simulation/gz", gitmodules)
        self.assertFalse((REPO_ROOT / "Tools/simulation/gz/models/flying_car").exists())
        self.assertEqual(REPO_ROOT / "Tools/simulation/gz_custom_models/flying_car/model.sdf",
                         MODEL_PATH)

        tracked_submodule_path = subprocess.run(
            ["git", "ls-files", "--stage", "Tools/simulation/gz"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        gitlink_fields = tracked_submodule_path.split()
        self.assertEqual(4, len(gitlink_fields))
        self.assertEqual("160000", gitlink_fields[0])
        self.assertRegex(gitlink_fields[1], r"^[0-9a-f]{40}$")
        self.assertEqual("0", gitlink_fields[2])
        self.assertEqual("Tools/simulation/gz", gitlink_fields[3])

    def test_environment_and_startup_resolve_the_custom_model_root(self):
        environment = GZ_ENV_PATH.read_text(encoding="utf-8")
        startup = GZ_STARTUP_PATH.read_text(encoding="utf-8")
        self.assertIn("PX4_GZ_CUSTOM_MODELS=@PX4_SOURCE_DIR@/Tools/simulation/gz_custom_models", environment)
        self.assertIn("$PX4_GZ_CUSTOM_MODELS", environment)
        self.assertIn('PX4_GZ_MODEL_ROOT="${PX4_GZ_CUSTOM_MODELS}"', startup)
        self.assertIn('PX4_GZ_MODEL_ROOT="${PX4_GZ_MODELS}"', startup)
        self.assertIn('file://${PX4_GZ_MODEL_ROOT}/${MODEL_NAME}/model.sdf', startup)


if __name__ == "__main__":
    unittest.main()
