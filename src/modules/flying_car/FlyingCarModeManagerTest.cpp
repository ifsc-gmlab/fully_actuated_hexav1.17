/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <initializer_list>
#include <limits>

#include <uORB/topics/flying_car_status.h>

#include "FlyingCarModeManager.hpp"
#include "FlyingCarTypes.hpp"

namespace
{

constexpr float kMaximumSpeedMps{0.2f};
constexpr uint64_t kTransitionDelayUs{500'000};
constexpr uint64_t kTransitionTimeoutUs{2'000'000};

FlyingCarTransitionInput validInput(FlyingCarMode requested_mode, uint64_t now_us)
{
	return {true, false, true, true, true, 0.f, requested_mode, now_us};
}

FlyingCarTransitionResult update(FlyingCarModeManager &manager, const FlyingCarTransitionInput &input)
{
	return manager.update(input, kMaximumSpeedMps, kTransitionDelayUs, kTransitionTimeoutUs);
}

void enterTransitionToGround(FlyingCarModeManager &manager, uint64_t request_started_us = 0)
{
	update(manager, validInput(FlyingCarMode::Ground, request_started_us));
	const auto result = update(manager, validInput(FlyingCarMode::Ground, request_started_us + kTransitionDelayUs));
	ASSERT_EQ(result.mode, FlyingCarMode::TransitionToGround);
}

void enterGround(FlyingCarModeManager &manager, uint64_t request_started_us = 0)
{
	enterTransitionToGround(manager, request_started_us);
	const auto result = update(manager, validInput(FlyingCarMode::Ground, request_started_us + 2 * kTransitionDelayUs));
	ASSERT_EQ(result.mode, FlyingCarMode::Ground);
}

void enterFault(FlyingCarModeManager &manager)
{
	enterTransitionToGround(manager);
	const auto result = update(manager,
				   validInput(FlyingCarMode::Ground, kTransitionDelayUs + kTransitionTimeoutUs + 1));
	ASSERT_EQ(result.mode, FlyingCarMode::Fault);
}

} // namespace

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

TEST(FlyingCarModeManager, DefaultsToFlightWithExclusiveFlightOutput)
{
	FlyingCarModeManager manager;
	const auto result = update(manager, validInput(FlyingCarMode::Flight, 0));

	EXPECT_EQ(result.mode, FlyingCarMode::Flight);
	EXPECT_TRUE(result.flight_output_enabled);
	EXPECT_FALSE(result.ground_output_enabled);
	EXPECT_FALSE(result.arming_locked);
}

TEST(FlyingCarModeManager, DisabledConfigurationRejectsAndNeverGrantsGroundOutput)
{
	FlyingCarModeManager manager;
	enterGround(manager);
	auto input = validInput(FlyingCarMode::Ground, 2 * kTransitionDelayUs + 1);
	input.configuration_enabled = false;
	const auto result = update(manager, input);

	EXPECT_EQ(result.rejection, FlyingCarRejection::NotFlyingCar);
	EXPECT_FALSE(result.transition_allowed);
	EXPECT_FALSE(result.ground_output_enabled);
}

TEST(FlyingCarModeManager, ArmedVehicleRejectsRequestedModeChange)
{
	FlyingCarModeManager manager;
	auto input = validInput(FlyingCarMode::Ground, 0);
	input.armed = true;
	const auto result = update(manager, input);

	EXPECT_EQ(result.mode, FlyingCarMode::Flight);
	EXPECT_EQ(result.rejection, FlyingCarRejection::Armed);
	EXPECT_FALSE(result.transition_allowed);
}

TEST(FlyingCarModeManager, AirborneVehicleRejectsRequestedModeChange)
{
	FlyingCarModeManager manager;
	auto input = validInput(FlyingCarMode::Ground, 0);
	input.landed = false;
	const auto result = update(manager, input);

	EXPECT_EQ(result.mode, FlyingCarMode::Flight);
	EXPECT_EQ(result.rejection, FlyingCarRejection::NotLanded);
}

TEST(FlyingCarModeManager, NonFiniteHorizontalSpeedRejectsRequestedModeChange)
{
	for (const float speed : {std::numeric_limits<float>::infinity(),
					 -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
		FlyingCarModeManager manager;
		auto input = validInput(FlyingCarMode::Ground, 0);
		input.horizontal_speed_m_s = speed;
		const auto result = update(manager, input);

		EXPECT_EQ(result.mode, FlyingCarMode::Flight);
		EXPECT_EQ(result.rejection, FlyingCarRejection::Moving);
	}
}

TEST(FlyingCarModeManager, ExcessiveAbsoluteHorizontalSpeedRejectsRequestedModeChange)
{
	for (const float speed : {0.21f, -0.21f}) {
		FlyingCarModeManager manager;
		auto input = validInput(FlyingCarMode::Ground, 0);
		input.horizontal_speed_m_s = speed;
		const auto result = update(manager, input);

		EXPECT_EQ(result.rejection, FlyingCarRejection::Moving);
	}
}

TEST(FlyingCarModeManager, UnhealthyGroundChainRejectsGroundRequest)
{
	FlyingCarModeManager manager;
	auto input = validInput(FlyingCarMode::Ground, 0);
	input.ground_chain_ready = false;
	const auto result = update(manager, input);

	EXPECT_EQ(result.rejection, FlyingCarRejection::ChainUnhealthy);
}

TEST(FlyingCarModeManager, UnhealthyFlightChainRejectsFlightRequest)
{
	FlyingCarModeManager manager;
	enterGround(manager);
	auto input = validInput(FlyingCarMode::Flight, 2 * kTransitionDelayUs + 1);
	input.flight_chain_ready = false;
	const auto result = update(manager, input);

	EXPECT_EQ(result.mode, FlyingCarMode::Ground);
	EXPECT_EQ(result.rejection, FlyingCarRejection::ChainUnhealthy);
}

TEST(FlyingCarModeManager, RequestMustRemainStableForConfiguredDelay)
{
	FlyingCarModeManager manager;

	EXPECT_EQ(update(manager, validInput(FlyingCarMode::Ground, 100)).mode, FlyingCarMode::Flight);
	EXPECT_EQ(update(manager, validInput(FlyingCarMode::Flight, 200)).mode, FlyingCarMode::Flight);
	EXPECT_EQ(update(manager, validInput(FlyingCarMode::Ground, 300)).mode, FlyingCarMode::Flight);
	EXPECT_EQ(update(manager, validInput(FlyingCarMode::Ground, 300 + kTransitionDelayUs - 1)).mode,
		  FlyingCarMode::Flight);
	EXPECT_EQ(update(manager, validInput(FlyingCarMode::Ground, 300 + kTransitionDelayUs)).mode,
		  FlyingCarMode::TransitionToGround);
}

TEST(FlyingCarModeManager, FlightToGroundPassesThroughSafeTransitionDwell)
{
	FlyingCarModeManager manager;
	enterTransitionToGround(manager);

	const auto during_transition = update(manager,
				       validInput(FlyingCarMode::Ground, 2 * kTransitionDelayUs - 1));
	EXPECT_EQ(during_transition.mode, FlyingCarMode::TransitionToGround);
	EXPECT_TRUE(during_transition.arming_locked);
	EXPECT_FALSE(during_transition.flight_output_enabled);
	EXPECT_FALSE(during_transition.ground_output_enabled);

	const auto complete = update(manager, validInput(FlyingCarMode::Ground, 2 * kTransitionDelayUs));
	EXPECT_EQ(complete.mode, FlyingCarMode::Ground);
}

TEST(FlyingCarModeManager, GroundToFlightPassesThroughSafeTransitionDwell)
{
	FlyingCarModeManager manager;
	enterGround(manager);

	update(manager, validInput(FlyingCarMode::Flight, 2 * kTransitionDelayUs + 100));
	const auto transition = update(manager,
				       validInput(FlyingCarMode::Flight, 3 * kTransitionDelayUs + 100));
	EXPECT_EQ(transition.mode, FlyingCarMode::TransitionToFlight);
	EXPECT_FALSE(transition.flight_output_enabled);
	EXPECT_FALSE(transition.ground_output_enabled);

	const auto complete = update(manager,
				     validInput(FlyingCarMode::Flight, 4 * kTransitionDelayUs + 100));
	EXPECT_EQ(complete.mode, FlyingCarMode::Flight);
}

TEST(FlyingCarModeManager, TransitionBeyondTimeoutEntersFault)
{
	FlyingCarModeManager manager;
	enterTransitionToGround(manager);
	const auto result = update(manager,
				   validInput(FlyingCarMode::Ground, kTransitionDelayUs + kTransitionTimeoutUs + 1));

	EXPECT_EQ(result.mode, FlyingCarMode::Fault);
	EXPECT_EQ(result.rejection, FlyingCarRejection::Timeout);
	EXPECT_TRUE(result.arming_locked);
}

TEST(FlyingCarModeManager, StableModesOwnOnlyTheirCorrespondingOutput)
{
	FlyingCarModeManager manager;
	const auto flight = update(manager, validInput(FlyingCarMode::Flight, 0));
	EXPECT_TRUE(flight.flight_output_enabled);
	EXPECT_FALSE(flight.ground_output_enabled);

	enterGround(manager);
	const auto ground = update(manager, validInput(FlyingCarMode::Ground, 2 * kTransitionDelayUs + 1));
	EXPECT_FALSE(ground.flight_output_enabled);
	EXPECT_TRUE(ground.ground_output_enabled);
}

TEST(FlyingCarModeManager, FaultIsLatchedWithNoOutputOwnership)
{
	FlyingCarModeManager manager;
	enterFault(manager);
	const auto result = update(manager, validInput(FlyingCarMode::Flight, 10'000'000));

	EXPECT_EQ(result.mode, FlyingCarMode::Fault);
	EXPECT_EQ(result.rejection, FlyingCarRejection::Timeout);
	EXPECT_FALSE(result.flight_output_enabled);
	EXPECT_FALSE(result.ground_output_enabled);
}

TEST(FlyingCarModeManager, FaultResetRequiresDisarmedHealthyFlightChain)
{
	FlyingCarModeManager manager;
	enterFault(manager);

	EXPECT_FALSE(manager.resetFault(true, true));
	EXPECT_FALSE(manager.resetFault(false, false));
	EXPECT_EQ(update(manager, validInput(FlyingCarMode::Flight, 10'000'000)).mode, FlyingCarMode::Fault);
}

TEST(FlyingCarModeManager, ExplicitValidFaultResetReturnsToFlight)
{
	FlyingCarModeManager manager;
	enterFault(manager);

	EXPECT_TRUE(manager.resetFault(false, true));
	const auto result = update(manager, validInput(FlyingCarMode::Flight, 10'000'000));
	EXPECT_EQ(result.mode, FlyingCarMode::Flight);
	EXPECT_EQ(result.rejection, FlyingCarRejection::None);
	EXPECT_TRUE(result.flight_output_enabled);
}
