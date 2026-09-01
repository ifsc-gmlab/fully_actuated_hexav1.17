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

#pragma once

#include <float.h>

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

namespace FullyActuatedControlMath
{

struct GroundTaxiSetpoint {
	matrix::Vector3f thrust_body{};
	float xy_limit{0.f};
};

struct AuxModeSelection {
	int32_t mode{0};
	bool aux_valid{false};
	bool selection_valid{false};
};

struct AttitudeModeState {
	int32_t active_mode{0};
	bool ground_taxi_active{false};
};

/**
 * Resolve a three-position AUX input into the matching numeric mode.
 *
 * Low/middle/high map monotonically to modes 0/1/2. Values between the
 * position bands retain the previous AUX-selected mode (Schmitt hysteresis).
 * Until AUX has selected a definite position, mode 0 is used. Parameter-mode
 * selection is intentionally handled by the caller only when AUX is disabled.
 */
inline AuxModeSelection selectThreePositionMode(float aux_value,
		int32_t previous_mode,
		bool previous_selection_valid)
{
	AuxModeSelection selection{};

	if (!PX4_ISFINITE(aux_value)) {
		return selection;
	}

	selection.aux_valid = true;

	if (aux_value <= -0.6f) {
		selection.mode = 0;
		selection.selection_valid = true;

	} else if ((aux_value >= -0.2f) && (aux_value <= 0.2f)) {
		selection.mode = 1;
		selection.selection_valid = true;

	} else if (aux_value >= 0.6f) {
		selection.mode = 2;
		selection.selection_valid = true;

	} else if (previous_selection_valid) {
		selection.mode = math::constrain(previous_mode, (int32_t)0, (int32_t)2);
		selection.selection_valid = true;
	}

	return selection;
}

/**
 * Apply the fully actuated Stabilized-mode safety gates.
 *
 * Ground taxi may only be entered while landed. Once entered, it stays active
 * until the request or controller availability is lost; land detection is not
 * used as an exit condition because horizontal taxi motion can unset it.
 */
inline AttitudeModeState updateAttitudeModeState(int32_t requested_mode,
		bool controller_available,
		bool landed,
		bool was_ground_taxi_active)
{
	AttitudeModeState state{};
	requested_mode = math::constrain(requested_mode, (int32_t)0, (int32_t)2);

	if (!controller_available) {
		return state;
	}

	state.ground_taxi_active = (requested_mode == 2) && (was_ground_taxi_active || landed);

	if (state.ground_taxi_active) {
		state.active_mode = 2;

	} else if (requested_mode == 2) {
		// An in-flight ground-taxi request uses the safe air mapping.
		state.active_mode = 1;

	} else {
		state.active_mode = requested_mode;
	}

	return state;
}

/**
 * Generate a body-frame ground-taxi force setpoint that stays inside the
 * unilateral-rotor force cone.
 *
 * Every rotor has an upward component and cannot reverse, so non-zero Fx/Fy
 * requires non-zero upward force. xy_to_z_ratio must be no larger than the
 * geometry's feasible |Fxy| / |Fz| ratio (including an allocation margin).
 */
inline GroundTaxiSetpoint groundTaxiSetpoint(matrix::Vector2f normalized_stick,
		float xy_limit_absolute,
		float xy_to_z_ratio,
		float z_limit)
{
	GroundTaxiSetpoint setpoint{};

	if (!normalized_stick.isAllFinite() || !PX4_ISFINITE(xy_limit_absolute) ||
	    !PX4_ISFINITE(xy_to_z_ratio) || !PX4_ISFINITE(z_limit)) {
		return setpoint;
	}

	const float ratio = math::constrain(xy_to_z_ratio, 0.f, 1.f);
	const float z_max = math::constrain(z_limit, 0.f, 1.f);
	const float xy_absolute = math::constrain(xy_limit_absolute, 0.f, 1.f);

	if ((ratio <= FLT_EPSILON) || (z_max <= FLT_EPSILON) ||
	    (xy_absolute <= FLT_EPSILON)) {
		return setpoint;
	}

	const float stick_norm = normalized_stick.norm();

	if (stick_norm > 1.f) {
		normalized_stick /= stick_norm;
	}

	setpoint.xy_limit = math::min(xy_absolute, ratio * z_max);
	const matrix::Vector2f thrust_xy = normalized_stick * setpoint.xy_limit;
	const float thrust_xy_norm = thrust_xy.norm();

	setpoint.thrust_body(0) = thrust_xy(0);
	setpoint.thrust_body(1) = thrust_xy(1);

	if (thrust_xy_norm > FLT_EPSILON) {
		setpoint.thrust_body(2) = -math::min(z_max, thrust_xy_norm / ratio);
	}

	return setpoint;
}

} // namespace FullyActuatedControlMath
