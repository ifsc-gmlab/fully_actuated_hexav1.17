/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

#include <cstdint>

class FlyingCarDifferentialControl
{
public:
	/** Mix normalized forward throttle and yaw into left and right wheel commands. */
	static matrix::Vector2f mix(float throttle, float yaw, float limit, uint8_t reversal_mask);
};
