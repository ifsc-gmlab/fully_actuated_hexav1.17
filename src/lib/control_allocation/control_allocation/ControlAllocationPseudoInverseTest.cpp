/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file ControlAllocationPseudoInverseTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include <gtest/gtest.h>
#include <ControlAllocationPseudoInverse.hpp>
#include <ControlAllocationPseudoInverseFullyActuated.hpp>

using namespace matrix;

TEST(ControlAllocationTest, AllZeroCase)
{
	ControlAllocationPseudoInverse method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	method.clipActuatorSetpoint();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}

TEST(ControlAllocationMetricTest, AllZeroCase)
{
	ControlAllocationPseudoInverse method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setMetricAllocation(true);
	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}

TEST(ControlAllocationTest, ThrustVectorNormalizationPreservesDirection)
{
	ControlAllocationPseudoInverseFullyActuated method;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 6> control_sp;

	// The physical X/Y/Z force authority is deliberately different. Shared
	// Fx/Fy/Fz normalization must keep the requested components in the same
	// units so the realized force direction is preserved.
	for (int axis = 0; axis < 3; ++axis) {
		effectiveness(axis, axis) = 1.f;
	}

	effectiveness(3, 3) = 0.5f;
	effectiveness(4, 4) = 0.25f;
	effectiveness(5, 5) = 1.f;
	control_sp(3) = 0.01f;
	control_sp(4) = 0.01f;
	control_sp(5) = 0.01f;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 6, true);
	method.setControlSetpoint(control_sp);
	method.allocate();

	const matrix::Vector<float, 6> allocated = method.getAllocatedControl();
	EXPECT_NEAR(allocated(3), control_sp(3), 1e-6f);
	EXPECT_NEAR(allocated(4), control_sp(4), 1e-6f);
	EXPECT_NEAR(allocated(5), control_sp(5), 1e-6f);
	EXPECT_FLOAT_EQ(method.getControlAllocationScale()(3), method.getControlAllocationScale()(5));
	EXPECT_FLOAT_EQ(method.getControlAllocationScale()(4), method.getControlAllocationScale()(5));

	// Check the raw force before ControlAllocation's bookkeeping scale is
	// applied. Per-axis normalization would make these components unequal.
	const matrix::Vector<float, 6> realized_force = effectiveness * method.getActuatorSetpoint();
	EXPECT_NEAR(realized_force(3), realized_force(5), 1e-6f);
	EXPECT_NEAR(realized_force(4), realized_force(5), 1e-6f);
}

TEST(ControlAllocationTest, ThrustVectorNormalizationUsesAvailableAxis)
{
	ControlAllocationPseudoInverseFullyActuated method;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 6> control_sp;

	// Forward-thrust-only airframes have no Z column. Preserve their existing
	// X-axis normalization while still assigning one common Fx/Fy/Fz scale.
	effectiveness(3, 0) = 0.5f;
	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 1, true);
	method.setControlSetpoint(control_sp);
	method.allocate();

	EXPECT_NEAR(method.getControlAllocationScale()(3), 2.f, 1e-6f);
	EXPECT_FLOAT_EQ(method.getControlAllocationScale()(3), method.getControlAllocationScale()(4));
	EXPECT_FLOAT_EQ(method.getControlAllocationScale()(3), method.getControlAllocationScale()(5));
}

TEST(ControlAllocationTest, ConventionalAirframeKeepsPerAxisThrustNormalization)
{
	ControlAllocationPseudoInverse method;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 6> control_sp;

	effectiveness(3, 3) = 0.5f;
	effectiveness(4, 4) = 0.25f;
	effectiveness(5, 5) = 1.f;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 6, true);
	method.setControlSetpoint(control_sp);
	method.allocate();

	EXPECT_NEAR(method.getControlAllocationScale()(3), 2.f, 1e-6f);
	EXPECT_NEAR(method.getControlAllocationScale()(4), 4.f, 1e-6f);
	EXPECT_NEAR(method.getControlAllocationScale()(5), 1.f, 1e-6f);
}
