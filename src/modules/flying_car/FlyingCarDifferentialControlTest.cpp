/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "FlyingCarDifferentialControl.hpp"

namespace
{

void expectMix(float throttle, float yaw, float limit, uint8_t reversal_mask, float expected_left,
	       float expected_right)
{
	const auto output = FlyingCarDifferentialControl::mix(throttle, yaw, limit, reversal_mask);
	EXPECT_FLOAT_EQ(output(0), expected_left);
	EXPECT_FLOAT_EQ(output(1), expected_right);
}

} // namespace

TEST(FlyingCarDifferentialControl, MixesStraightForward)
{
	expectMix(0.4f, 0.f, 0.8f, 0, 0.4f, 0.4f);
}

TEST(FlyingCarDifferentialControl, MixesStraightReverse)
{
	expectMix(-0.4f, 0.f, 0.8f, 0, -0.4f, -0.4f);
}

TEST(FlyingCarDifferentialControl, MixesBothTurnDirections)
{
	expectMix(0.5f, 0.2f, 1.f, 0, 0.3f, 0.7f);
	expectMix(0.5f, -0.2f, 1.f, 0, 0.7f, 0.3f);
}

TEST(FlyingCarDifferentialControl, MixesPivotTurn)
{
	expectMix(0.f, 0.5f, 1.f, 0, -0.5f, 0.5f);
}

TEST(FlyingCarDifferentialControl, ReversesWheelsIndependentlyAndTogether)
{
	expectMix(0.25f, 0.1f, 1.f, 1, -0.15f, 0.35f);
	expectMix(0.25f, 0.1f, 1.f, 2, 0.15f, -0.35f);
	expectMix(0.25f, 0.1f, 1.f, 3, -0.15f, -0.35f);
}

TEST(FlyingCarDifferentialControl, SaturatesPositiveAndNegativeOutputs)
{
	expectMix(2.f, 0.f, 0.7f, 0, 0.7f, 0.7f);
	expectMix(-2.f, 0.f, 0.7f, 0, -0.7f, -0.7f);
}

TEST(FlyingCarDifferentialControl, RejectsNonFiniteInput)
{
	const float infinity = std::numeric_limits<float>::infinity();
	const float nan = std::numeric_limits<float>::quiet_NaN();

	expectMix(infinity, 0.f, 1.f, 0, 0.f, 0.f);
	expectMix(0.f, -infinity, 1.f, 0, 0.f, 0.f);
	expectMix(0.f, 0.f, nan, 0, 0.f, 0.f);
}

TEST(FlyingCarDifferentialControl, RejectsZeroAndNegativeLimit)
{
	expectMix(0.5f, 0.1f, 0.f, 0, 0.f, 0.f);
	expectMix(0.5f, 0.1f, -0.5f, 0, 0.f, 0.f);
}

TEST(FlyingCarDifferentialControl, ConstrainsOversizeLimitToOne)
{
	expectMix(0.8f, 0.5f, 2.f, 0, 0.3f, 1.f);
}
