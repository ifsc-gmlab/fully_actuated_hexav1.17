/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include "FlyingCarDifferentialControl.hpp"

#include <mathlib/math/Limits.hpp>

#include <cmath>

matrix::Vector2f FlyingCarDifferentialControl::mix(float throttle, float yaw, float limit, uint8_t reversal_mask)
{
	if (!std::isfinite(throttle) || !std::isfinite(yaw) || !std::isfinite(limit) || limit <= 0.f) {
		return {0.f, 0.f};
	}

	const float normalized_limit = math::constrain(limit, 0.f, 1.f);
	float left = math::constrain(throttle - yaw, -normalized_limit, normalized_limit);
	float right = math::constrain(throttle + yaw, -normalized_limit, normalized_limit);

	if ((reversal_mask & 1u) != 0u) {
		left = -left;
	}

	if ((reversal_mask & 2u) != 0u) {
		right = -right;
	}

	return {left, right};
}
