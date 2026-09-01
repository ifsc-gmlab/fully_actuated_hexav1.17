/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

enum class FlyingCarMode : uint8_t {
	Flight = 0,
	TransitionToGround = 1,
	Ground = 2,
	TransitionToFlight = 3,
	Fault = 4,
};

enum class FlyingCarRejectionReason : uint8_t {
	None = 0,
	NotFlyingCar = 1,
	Armed = 2,
	NotLanded = 3,
	Moving = 4,
	ChainUnhealthy = 5,
	Timeout = 6,
};
