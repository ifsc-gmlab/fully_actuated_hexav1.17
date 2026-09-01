/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <uORB/topics/flying_car_status.h>

#include "FlyingCarTypes.hpp"

TEST(FlyingCarTypes, EnumsMatchFlyingCarStatusContract)
{
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::Flight), flying_car_status_s::MODE_FLIGHT);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::TransitionToGround), flying_car_status_s::MODE_TRANSITION_TO_GROUND);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::Ground), flying_car_status_s::MODE_GROUND);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::TransitionToFlight), flying_car_status_s::MODE_TRANSITION_TO_FLIGHT);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarMode::Fault), flying_car_status_s::MODE_FAULT);

	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::None), flying_car_status_s::REJECTION_NONE);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::NotFlyingCar), flying_car_status_s::REJECTION_NOT_FLYING_CAR);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::Armed), flying_car_status_s::REJECTION_ARMED);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::NotLanded), flying_car_status_s::REJECTION_NOT_LANDED);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::Moving), flying_car_status_s::REJECTION_MOVING);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::ChainUnhealthy), flying_car_status_s::REJECTION_CHAIN_UNHEALTHY);
	EXPECT_EQ(static_cast<uint8_t>(FlyingCarRejectionReason::Timeout), flying_car_status_s::REJECTION_TIMEOUT);
}
