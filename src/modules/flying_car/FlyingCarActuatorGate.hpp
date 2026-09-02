/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include "FlyingCarTypes.hpp"

#include <matrix/matrix/math.hpp>

#include <array>
#include <cstdint>

struct FlyingCarActuatorGateOutput {
	bool bypass;
	std::array<float, 6> controls;
	uint16_t reversible_flags;
};

class FlyingCarActuatorGate
{
public:
	/**
	 * Select mutually exclusive rotor or wheel ownership.
	 *
	 * A bypass result contains no replacement publication; callers retain their original sample.
	 */
	static FlyingCarActuatorGateOutput apply(bool configuration_enabled, FlyingCarMode mode,
			const std::array<float, 4> &rotor_values, const matrix::Vector2f &wheel_values);
};
