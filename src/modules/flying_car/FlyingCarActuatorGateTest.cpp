/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "FlyingCarActuatorGate.hpp"

namespace
{

const std::array<float, 4> kRotors{0.1f, 0.2f, 0.3f, 0.4f};
const matrix::Vector2f kWheels{0.5f, -0.6f};

void expectRotorsDisabled(const FlyingCarActuatorGateOutput &output)
{
	for (size_t index = 0; index < 4; ++index) {
		EXPECT_TRUE(std::isnan(output.controls[index]));
	}
}

void expectWheelsNeutral(const FlyingCarActuatorGateOutput &output)
{
	EXPECT_FLOAT_EQ(output.controls[4], 0.f);
	EXPECT_FLOAT_EQ(output.controls[5], 0.f);
}

} // namespace

TEST(FlyingCarActuatorGate, DisabledConfigurationBypassesPublication)
{
	const auto output = FlyingCarActuatorGate::apply(false, FlyingCarMode::Ground, kRotors, kWheels);

	EXPECT_TRUE(output.bypass);
}

TEST(FlyingCarActuatorGate, FlightPassesFiniteRotorsAndNeutralizesWheels)
{
	const auto output = FlyingCarActuatorGate::apply(true, FlyingCarMode::Flight, kRotors, kWheels);

	EXPECT_FALSE(output.bypass);
	EXPECT_FLOAT_EQ(output.controls[0], 0.1f);
	EXPECT_FLOAT_EQ(output.controls[1], 0.2f);
	EXPECT_FLOAT_EQ(output.controls[2], 0.3f);
	EXPECT_FLOAT_EQ(output.controls[3], 0.4f);
	expectWheelsNeutral(output);
	EXPECT_EQ(output.reversible_flags, 0);
}

TEST(FlyingCarActuatorGate, FlightDisablesEachNonFiniteRotor)
{
	const float infinity = std::numeric_limits<float>::infinity();
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const std::array<float, 4> rotors{nan, infinity, -infinity, 0.4f};
	const auto output = FlyingCarActuatorGate::apply(true, FlyingCarMode::Flight, rotors, kWheels);

	EXPECT_TRUE(std::isnan(output.controls[0]));
	EXPECT_TRUE(std::isnan(output.controls[1]));
	EXPECT_TRUE(std::isnan(output.controls[2]));
	EXPECT_FLOAT_EQ(output.controls[3], 0.4f);
	expectWheelsNeutral(output);
	EXPECT_EQ(output.reversible_flags, 0);
}

TEST(FlyingCarActuatorGate, GroundDisablesRotorsAndPassesConstrainedWheels)
{
	const matrix::Vector2f wheels{1.5f, -2.f};
	const auto output = FlyingCarActuatorGate::apply(true, FlyingCarMode::Ground, kRotors, wheels);

	EXPECT_FALSE(output.bypass);
	expectRotorsDisabled(output);
	EXPECT_FLOAT_EQ(output.controls[4], 1.f);
	EXPECT_FLOAT_EQ(output.controls[5], -1.f);
	EXPECT_EQ(output.reversible_flags, static_cast<uint16_t>(48));
}

TEST(FlyingCarActuatorGate, GroundNeutralizesEachNonFiniteWheel)
{
	const float infinity = std::numeric_limits<float>::infinity();
	const matrix::Vector2f wheels{infinity, std::numeric_limits<float>::quiet_NaN()};
	const auto output = FlyingCarActuatorGate::apply(true, FlyingCarMode::Ground, kRotors, wheels);

	expectRotorsDisabled(output);
	expectWheelsNeutral(output);
	EXPECT_EQ(output.reversible_flags, static_cast<uint16_t>(48));
}

TEST(FlyingCarActuatorGate, TransitionFaultAndInvalidModeUseSafeOutputs)
{
	for (const FlyingCarMode mode : {FlyingCarMode::TransitionToGround, FlyingCarMode::TransitionToFlight,
					 FlyingCarMode::Fault, static_cast<FlyingCarMode>(255)}) {
		const auto output = FlyingCarActuatorGate::apply(true, mode, kRotors, kWheels);

		EXPECT_FALSE(output.bypass);
		expectRotorsDisabled(output);
		expectWheelsNeutral(output);
		EXPECT_EQ(output.reversible_flags, 0);
	}
}
