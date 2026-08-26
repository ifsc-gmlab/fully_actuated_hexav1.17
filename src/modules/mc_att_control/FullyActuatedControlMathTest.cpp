/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <lib/systemlib/fully_actuated_airframe.h>

#include "FullyActuatedControlMath.hpp"

using FullyActuatedControlMath::groundTaxiSetpoint;
using FullyActuatedControlMath::selectThreePositionMode;
using FullyActuatedControlMath::updateAttitudeModeState;
using matrix::Vector2f;

TEST(FullyActuatedAirframe, OnlyConfiguredAutostartIdsAreEligible)
{
	EXPECT_TRUE(px4::isFullyActuatedAirframe(6003));
	EXPECT_TRUE(px4::isFullyActuatedAirframe(4026));
	EXPECT_TRUE(px4::isFullyActuatedAirframe(22000));
	EXPECT_FALSE(px4::isFullyActuatedAirframe(0));
	EXPECT_FALSE(px4::isFullyActuatedAirframe(1003));
	EXPECT_FALSE(px4::isFullyActuatedAirframe(4001));
}

TEST(FullyActuatedControlMath, AuxPositionsMatchModeNumbers)
{
	const auto low = selectThreePositionMode(-1.f, 1, false);
	const auto middle = selectThreePositionMode(0.f, 0, false);
	const auto high = selectThreePositionMode(1.f, 0, false);

	EXPECT_EQ(low.mode, 0);
	EXPECT_EQ(middle.mode, 1);
	EXPECT_EQ(high.mode, 2);
	EXPECT_TRUE(low.selection_valid);
	EXPECT_TRUE(middle.selection_valid);
	EXPECT_TRUE(high.selection_valid);
}

TEST(FullyActuatedControlMath, AuxHysteresisRetainsPreviousSelection)
{
	const auto selection = selectThreePositionMode(0.4f, 1, true);

	EXPECT_EQ(selection.mode, 1);
	EXPECT_TRUE(selection.selection_valid);
	EXPECT_TRUE(selection.aux_valid);
}

TEST(FullyActuatedControlMath, AuxGapInitiallyUsesOff)
{
	const auto selection = selectThreePositionMode(-0.4f, 2, false);

	EXPECT_EQ(selection.mode, 0);
	EXPECT_FALSE(selection.selection_valid);
	EXPECT_TRUE(selection.aux_valid);
}

TEST(FullyActuatedControlMath, InvalidAuxUsesOff)
{
	const auto selection = selectThreePositionMode(NAN, 2, true);

	EXPECT_EQ(selection.mode, 0);
	EXPECT_FALSE(selection.selection_valid);
	EXPECT_FALSE(selection.aux_valid);
}

TEST(FullyActuatedControlMath, GroundTaxiEntersOnlyWhileLanded)
{
	const auto landed = updateAttitudeModeState(2, true, true, false);
	EXPECT_EQ(landed.active_mode, 2);
	EXPECT_TRUE(landed.ground_taxi_active);

	const auto airborne = updateAttitudeModeState(2, true, false, false);
	EXPECT_EQ(airborne.active_mode, 1);
	EXPECT_FALSE(airborne.ground_taxi_active);
}

TEST(FullyActuatedControlMath, GroundTaxiStaysLatchedUntilControlIsLost)
{
	const auto retained = updateAttitudeModeState(2, true, false, true);
	EXPECT_EQ(retained.active_mode, 2);
	EXPECT_TRUE(retained.ground_taxi_active);

	const auto unavailable = updateAttitudeModeState(2, false, true, true);
	EXPECT_EQ(unavailable.active_mode, 0);
	EXPECT_FALSE(unavailable.ground_taxi_active);

	const auto mode_changed = updateAttitudeModeState(1, true, true, true);
	EXPECT_EQ(mode_changed.active_mode, 1);
	EXPECT_FALSE(mode_changed.ground_taxi_active);
}

TEST(FullyActuatedControlMath, NeutralStickProducesNoThrust)
{
	const auto setpoint = groundTaxiSetpoint(Vector2f{}, 0.15f, 0.75f, 0.2f);

	EXPECT_FLOAT_EQ(setpoint.thrust_body(0), 0.f);
	EXPECT_FLOAT_EQ(setpoint.thrust_body(1), 0.f);
	EXPECT_FLOAT_EQ(setpoint.thrust_body(2), 0.f);
}

TEST(FullyActuatedControlMath, CouplesHorizontalAndVerticalThrust)
{
	const auto setpoint =
		groundTaxiSetpoint(Vector2f{0.5f, 0.f}, 0.15f, 0.75f, 0.2f);

	EXPECT_NEAR(setpoint.thrust_body(0), 0.075f, 1e-6f);
	EXPECT_FLOAT_EQ(setpoint.thrust_body(1), 0.f);
	EXPECT_NEAR(setpoint.thrust_body(2), -0.1f, 1e-6f);
	EXPECT_LE(setpoint.thrust_body.xy().norm(),
		  0.75f * fabsf(setpoint.thrust_body(2)) + 1e-6f);
}

TEST(FullyActuatedControlMath, LimitsHorizontalDemandByVerticalBudget)
{
	const auto setpoint =
		groundTaxiSetpoint(Vector2f{1.f, 0.f}, 0.4f, 0.75f, 0.2f);

	EXPECT_NEAR(setpoint.xy_limit, 0.15f, 1e-6f);
	EXPECT_NEAR(setpoint.thrust_body(0), 0.15f, 1e-6f);
	EXPECT_NEAR(setpoint.thrust_body(2), -0.2f, 1e-6f);
}

TEST(FullyActuatedControlMath, LimitsDiagonalStickMagnitude)
{
	const auto setpoint =
		groundTaxiSetpoint(Vector2f{1.f, 1.f}, 0.15f, 0.75f, 0.2f);

	EXPECT_NEAR(setpoint.thrust_body.xy().norm(), 0.15f, 1e-6f);
	EXPECT_NEAR(setpoint.thrust_body(2), -0.2f, 1e-6f);
}

TEST(FullyActuatedControlMath, InvalidGeometryProducesNoThrust)
{
	const auto setpoint =
		groundTaxiSetpoint(Vector2f{1.f, 0.f}, 0.15f, 0.f, 0.2f);

	EXPECT_FLOAT_EQ(setpoint.thrust_body.norm(), 0.f);
}
