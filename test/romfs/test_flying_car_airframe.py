#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
INIT_DIR = REPO_ROOT / "ROMFS" / "px4fmu_common" / "init.d"
HW_AIRFRAME = INIT_DIR / "airframes" / "80003_flying_car"
SITL_AIRFRAME = REPO_ROOT / "ROMFS" / "px4fmu_common" / "init.d-posix" / "airframes" / "80003_flying_car"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


class FlyingCarAirframeTest(unittest.TestCase):

    def test_80003_is_registered_once_per_airframe_registry(self):
        hardware_registry = read(INIT_DIR / "airframes" / "CMakeLists.txt")
        posix_registry = read(REPO_ROOT / "ROMFS" / "px4fmu_common" / "init.d-posix" / "airframes" / "CMakeLists.txt")

        self.assertEqual(hardware_registry.count("80003_flying_car"), 1)
        self.assertEqual(posix_registry.count("80003_flying_car"), 1)

    def test_sitl_build_selects_the_module_required_by_posix_80003(self):
        sitl_board = read(REPO_ROOT / "boards" / "px4" / "sitl" / "default.px4board")

        self.assertEqual(sitl_board.count("CONFIG_MODULES_FLYING_CAR=y"), 1)

    def test_hardware_airframe_has_isolated_six_actuator_contract(self):
        script = read(HW_AIRFRAME)
        expected_defaults = {
            "CA_AIRFRAME": "0",
            "CA_ROTOR_COUNT": "4",
            "SYS_FC_TYPE": "1",
            "CA_R_REV": "48",
            "PWM_AUX_FUNC1": "101",
            "PWM_AUX_FUNC2": "102",
            "PWM_AUX_FUNC3": "103",
            "PWM_AUX_FUNC4": "104",
            "PWM_AUX_FUNC5": "105",
            "PWM_AUX_FUNC6": "106",
            "PWM_AUX_DIS5": "1500",
            "PWM_AUX_DIS6": "1500",
            "PWM_AUX_MIN5": "1100",
            "PWM_AUX_MIN6": "1100",
            "PWM_AUX_MAX5": "1900",
            "PWM_AUX_MAX6": "1900",
        }

        for name, value in expected_defaults.items():
            self.assertRegex(script, rf"(?m)^param set-default {name}\s+{value}(?:\s|$)")

        for rotor in range(4):
            self.assertRegex(script, rf"(?m)^param set-default CA_ROTOR{rotor}_PX\s+-?[0-9.]+")
            self.assertRegex(script, rf"(?m)^param set-default CA_ROTOR{rotor}_PY\s+-?[0-9.]+")
            self.assertRegex(script, rf"(?m)^param set-default CA_ROTOR{rotor}_KM\s+-?[0-9.]+")

    def test_flying_car_startup_reuses_mc_apps_then_starts_only_its_module(self):
        setup = read(INIT_DIR / "rc.vehicle_setup")
        apps = read(INIT_DIR / "rc.flying_car_apps")

        branch = re.search(
            r"if \[ \$VEHICLE_TYPE = flying_car \].*?fi",
            setup,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(branch)
        self.assertIn("rc.flying_car_apps", branch.group(0))
        self.assertIn("rc.mc_apps", apps)
        self.assertEqual(len(re.findall(r"(?m)^flying_car start\s*$", apps)), 1)
        self.assertNotRegex(apps, r"(?m)^(rover_differential|rover_ackermann|rover_mecanum) start")

    def test_defaults_select_flying_car_without_changing_native_mc_defaults(self):
        defaults = read(INIT_DIR / "rc.flying_car_defaults")
        sitl_defaults = read(INIT_DIR / "rc.flying_car_sitl_defaults")

        self.assertIn("rc.mc_defaults", defaults)
        self.assertRegex(defaults, r"(?m)^set VEHICLE_TYPE flying_car$")
        self.assertRegex(defaults, r"(?m)^param set-default SYS_FC_TYPE\s+1$")
        self.assertIn("rc.flying_car_defaults", sitl_defaults)

    def test_no_ordinary_startup_script_starts_flying_car(self):
        allowed = INIT_DIR / "rc.flying_car_apps"
        offenders = []

        for script in INIT_DIR.glob("rc.*_apps"):
            if script != allowed and re.search(r"(?m)^flying_car start\s*$", read(script)):
                offenders.append(script.name)

        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
