"""Structural regression checks for Commander's non-optional flying-car arming gate."""


def verify_gate_order(source: str) -> list[str]:
    errors: list[str] = []
    arm_start = source.find("transition_result_t Commander::arm(")
    arm_end = source.find("transition_result_t Commander::disarm(", arm_start + 1)

    if arm_start < 0 or arm_end < 0:
        return ["Commander::arm function boundaries not found"]

    arm = source[arm_start:arm_end]
    gate = arm.find("FlyingCarSafety::armingEntryAllowed")
    grace = arm.find("run_preflight_checks = false")
    optional_checks = arm.find("if (run_preflight_checks)")

    if gate < 0:
        errors.append("flying-car arming entry gate missing")

    for label, position in (("RC grace rewrite", grace), ("optional preflight branch", optional_checks)):
        if position < 0:
            errors.append(f"{label} missing")
        elif gate < 0 or gate > position:
            errors.append(f"flying-car gate must precede {label}")

    if gate >= 0 and grace >= 0 and "return TRANSITION_DENIED" not in arm[gate:grace]:
        errors.append("flying-car gate does not deny arming before bypass branches")

    return errors
