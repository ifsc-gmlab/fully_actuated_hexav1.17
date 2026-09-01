/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "FlyingCarTypes.hpp"

TEST(FlyingCarTypes, ModeAndRejectionValuesMatchFlyingCarStatusContract)
{
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::Flight), 0);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::TransitionToGround), 1);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::Ground), 2);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::TransitionToFlight), 3);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::Fault), 4);

	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::None), 0);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::NotFlyingCar), 1);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::Armed), 2);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::NotLanded), 3);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::Moving), 4);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::ChainUnhealthy), 5);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::Timeout), 6);
}
