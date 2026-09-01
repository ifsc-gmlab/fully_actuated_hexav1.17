/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "FullyActuatedAttitudeControl.hpp"

#include "FullyActuatedControlMath.hpp"

#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

FullyActuatedAttitudeControl::FullyActuatedAttitudeControl(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FullyActuatedAttitudeControl::updateMode(const manual_control_setpoint_s &manual_control,
		bool stabilized_manual_control, bool landed, bool vtol)
{
	const bool was_airframe_enabled = _airframe_enabled;
	const int32_t previous_active_mode = _active_mode;
	const bool was_ground_taxi_active = _ground_taxi_active;

	_airframe_enabled = px4::isFullyActuatedAirframe(_param_sys_autostart.get());

	if (!_airframe_enabled) {
		if (was_airframe_enabled) {
			clearState();
			publishStatus();
		}

		return previous_active_mode != _active_mode;
	}

	const bool manual_control_valid = manualControlValid(manual_control);
	_requested_mode = resolveRequestedMode(manual_control, manual_control_valid);
	const bool controller_available = stabilized_manual_control && manual_control_valid && !vtol;
	const auto mode_state = FullyActuatedControlMath::updateAttitudeModeState(_requested_mode,
				controller_available, landed, was_ground_taxi_active);
	_active_mode = mode_state.active_mode;
	_ground_taxi_active = mode_state.ground_taxi_active;

	const bool active_mode_changed = previous_active_mode != _active_mode;

	if (active_mode_changed) {
		_roll_input_filter.reset(0.f);
		_pitch_input_filter.reset(0.f);
	}

	publishStatus();
	return active_mode_changed;
}

bool FullyActuatedAttitudeControl::generateAttitudeSetpoint(const SetpointInput &input,
		vehicle_attitude_setpoint_s &attitude_setpoint)
{
	if (!active()) {
		return false;
	}

	_roll_input_filter.setParameters(input.dt, math::max(input.tilt_time_constant, 0.f));
	_pitch_input_filter.setParameters(input.dt, math::max(input.tilt_time_constant, 0.f));
	attitude_setpoint.yaw_sp_move_rate = input.yaw_sp_move_rate;

	if (_active_mode == MODE_GROUND_TAXI) {
		const float pitch_input = math::expo_deadzone(input.pitch, 0.f, input.manual_deadzone);
		const float roll_input = math::expo_deadzone(input.roll, 0.f, input.manual_deadzone);
		const Vector2f normalized_stick{
			_pitch_input_filter.update(pitch_input),
			_roll_input_filter.update(roll_input)
		};

		// Stay below hover thrust and the normal spool-up output limit so taxi
		// cannot intentionally command lift-off or bypass ramp protection.
		const float hover_thrust_limit = 0.8f * math::constrain(input.hover_thrust, 0.f, 1.f);
		const float z_limit = math::min(math::min(_param_mpc_fa_gnd_zmax.get(), hover_thrust_limit),
						input.maximum_thrust);
		const auto ground_setpoint = FullyActuatedControlMath::groundTaxiSetpoint(normalized_stick,
					     _param_mpc_fa_xy_thr.get(), _param_mpc_fa_gnd_xy_z.get(), z_limit);
		ground_setpoint.thrust_body.copyTo(attitude_setpoint.thrust_body);

		// Follow the ground-constrained roll/pitch while yaw remains controlled.
		const Eulerf current_attitude{input.attitude};
		const Quatf q_sp{Eulerf{current_attitude.phi(), current_attitude.theta(), input.yaw_setpoint}};
		q_sp.copyTo(attitude_setpoint.q_d);
		return true;
	}

	const float xy_thr_abs = math::constrain(_param_mpc_fa_xy_thr.get(), 0.f, 1.f);
	const float xy_thr_ratio = math::constrain(_param_mpc_fa_xy_ratio.get(), 0.f, 1.f);
	const float xy_thr_max = math::min(xy_thr_abs, xy_thr_ratio * fabsf(input.vertical_thrust));

	Vector2f thrust_xy_sp(_pitch_input_filter.update(input.pitch * xy_thr_max),
			      _roll_input_filter.update(input.roll * xy_thr_max));
	const float thrust_xy_norm = thrust_xy_sp.norm();

	if ((xy_thr_max > FLT_EPSILON) && (thrust_xy_norm > xy_thr_max)) {
		thrust_xy_sp *= xy_thr_max / thrust_xy_norm;
	}

	// Heading-frame FRD force: x forward, y right, z down.
	const Vector3f thrust_heading{thrust_xy_sp(0), thrust_xy_sp(1), input.vertical_thrust};
	const Quatf q_yaw{Eulerf{0.f, 0.f, input.yaw_setpoint}};
	const Vector3f thrust_ned = q_yaw.rotateVector(thrust_heading);
	Quatf q_current{input.attitude};

	if (!q_current.isAllFinite() || (q_current.norm_squared() < FLT_EPSILON)) {
		q_current = q_yaw;

	} else {
		q_current.normalize();
	}

	const Vector3f thrust_body = q_current.rotateVectorInverse(thrust_ned);
	q_yaw.copyTo(attitude_setpoint.q_d);

	if (thrust_body.isAllFinite()) {
		thrust_body.copyTo(attitude_setpoint.thrust_body);

	} else {
		attitude_setpoint.thrust_body[0] = 0.f;
		attitude_setpoint.thrust_body[1] = 0.f;
		attitude_setpoint.thrust_body[2] = input.vertical_thrust;
	}

	return true;
}

int32_t FullyActuatedAttitudeControl::resolveRequestedMode(const manual_control_setpoint_s &manual_control,
		bool manual_control_valid)
{
	const int32_t param_mode = math::constrain(_param_mpc_fa_stab.get(), (int32_t)0, (int32_t)2);
	const int32_t aux_channel = _param_mpc_fa_stab_aux.get();
	const bool previous_selection_valid = (_aux_index == aux_channel) && _aux_selection_valid;

	_aux_valid = false;
	_aux_selection_valid = false;
	_aux_value = NAN;

	if (aux_channel == 0) {
		_mode_source = SOURCE_PARAMETER;
		_aux_index = 0;
		return param_mode;
	}

	// A configured AUX selector is authoritative. Never fall back to the
	// parameter if the input is unavailable or between defined positions.
	_mode_source = SOURCE_AUX;

	if ((aux_channel < 1) || (aux_channel > 6)) {
		_aux_index = 0;
		return MODE_OFF;
	}

	_aux_index = static_cast<uint8_t>(aux_channel);

	if (!manual_control_valid) {
		return MODE_OFF;
	}

	_aux_value = selectedAuxValue(manual_control, aux_channel);
	const auto selection = FullyActuatedControlMath::selectThreePositionMode(_aux_value, _requested_mode,
			       previous_selection_valid);
	_aux_valid = selection.aux_valid;
	_aux_selection_valid = selection.selection_valid;
	return selection.mode;
}

bool FullyActuatedAttitudeControl::manualControlValid(const manual_control_setpoint_s &manual_control) const
{
	return manual_control.valid
	       && (manual_control.timestamp != 0)
	       && (hrt_elapsed_time(&manual_control.timestamp) <= 500_ms)
	       && PX4_ISFINITE(manual_control.roll)
	       && PX4_ISFINITE(manual_control.pitch)
	       && PX4_ISFINITE(manual_control.yaw)
	       && PX4_ISFINITE(manual_control.throttle);
}

float FullyActuatedAttitudeControl::selectedAuxValue(const manual_control_setpoint_s &manual_control,
		int32_t aux_channel) const
{
	switch (aux_channel) {
	case 1:
		return manual_control.aux1;

	case 2:
		return manual_control.aux2;

	case 3:
		return manual_control.aux3;

	case 4:
		return manual_control.aux4;

	case 5:
		return manual_control.aux5;

	case 6:
		return manual_control.aux6;

	default:
		return NAN;
	}
}

void FullyActuatedAttitudeControl::clearState()
{
	_requested_mode = MODE_OFF;
	_active_mode = MODE_OFF;
	_mode_source = SOURCE_PARAMETER;
	_aux_index = 0;
	_aux_valid = false;
	_aux_selection_valid = false;
	_aux_value = NAN;
	_ground_taxi_active = false;
	_roll_input_filter.reset(0.f);
	_pitch_input_filter.reset(0.f);
}

void FullyActuatedAttitudeControl::publishStatus()
{
	fully_actuated_control_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.ground_taxi_active = _ground_taxi_active;
	_status_pub.publish(status);
}

void FullyActuatedAttitudeControl::printStatus() const
{
	if (!_airframe_enabled) {
		PX4_INFO("FA Stabilized: disabled for this airframe");
		return;
	}

	const char *mode_name = "Off (tilt)";

	switch (_active_mode) {
	case MODE_AIR:
		mode_name = "Air (level + Fx/Fy)";
		break;

	case MODE_GROUND_TAXI:
		mode_name = "Ground taxi (body Fx/Fy)";
		break;

	default:
		break;
	}

	const char *mode_source = (_mode_source == SOURCE_AUX) ? "AUX" : "parameter";
	PX4_INFO("FA Stabilized: %s (requested=%d, active=%d, source=%s)",
		 mode_name, (int)_requested_mode, (int)_active_mode, mode_source);
	PX4_INFO("MPC_FA_STAB=%d, AUX selector=%d, AUX value=%.3f (%s)",
		 (int)_param_mpc_fa_stab.get(), (int)_aux_index,
		 (double)_aux_value, _aux_valid ? "valid" : "invalid");
}
