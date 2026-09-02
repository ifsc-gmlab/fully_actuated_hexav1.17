/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include "FlyingCarActuatorGate.hpp"

#include <mathlib/math/Limits.hpp>

#include <cmath>

namespace
{

constexpr uint16_t kWheelReversibleFlags{(1u << 4) | (1u << 5)};

float finiteOrDisabled(float value)
{
	return std::isfinite(value) ? value : NAN;
}

float finiteConstrainedWheelOrNeutral(float value)
{
	return std::isfinite(value) ? math::constrain(value, -1.f, 1.f) : 0.f;
}

FlyingCarActuatorGateOutput safeOutput(bool bypass)
{
	return {bypass, {NAN, NAN, NAN, NAN, 0.f, 0.f},
		static_cast<uint16_t>(bypass ? 0 : kWheelReversibleFlags)};
}

} // namespace

FlyingCarActuatorGateOutput FlyingCarActuatorGate::apply(bool configuration_enabled, FlyingCarMode mode,
		const std::array<float, 4> &rotor_values, const matrix::Vector2f &wheel_values)
{
	if (!configuration_enabled) {
		return safeOutput(true);
	}

	if (mode == FlyingCarMode::Flight) {
		auto output = safeOutput(false);

		for (size_t index = 0; index < rotor_values.size(); ++index) {
			output.controls[index] = finiteOrDisabled(rotor_values[index]);
		}

		return output;
	}

	if (mode == FlyingCarMode::Ground) {
		auto output = safeOutput(false);
		output.controls[4] = finiteConstrainedWheelOrNeutral(wheel_values(0));
		output.controls[5] = finiteConstrainedWheelOrNeutral(wheel_values(1));
		output.reversible_flags = kWheelReversibleFlags;
		return output;
	}

	return safeOutput(false);
}
