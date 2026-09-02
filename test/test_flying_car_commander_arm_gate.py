import unittest
from pathlib import Path

from commander_arm_gate_structure import verify_gate_order


class CommanderArmGateStructureTest(unittest.TestCase):
    def test_gate_precedes_every_optional_preflight_bypass(self):
        source = (Path(__file__).parents[1] / "src/modules/commander/Commander.cpp").read_text(encoding="utf-8")
        self.assertEqual([], verify_gate_order(source))

    def test_checker_rejects_arm_false_bypass(self):
        unsafe = """
        transition_result_t Commander::arm(arm_disarm_reason_t reason, bool run_preflight_checks)
        {
            if (reason == arm_disarm_reason_t::rc_switch) { run_preflight_checks = false; }
            if (run_preflight_checks) { run_checks(); }
            if (!FlyingCarSafety::armingEntryAllowed(enabled, locked)) { return TRANSITION_DENIED; }
            return TRANSITION_CHANGED;
        }
        """
        self.assertTrue(verify_gate_order(unsafe))


if __name__ == "__main__":
    unittest.main()
