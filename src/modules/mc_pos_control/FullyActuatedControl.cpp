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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#include "FullyActuatedControl.hpp"

#include "PositionControl/ControlMath.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>

using namespace time_literals;

FullyActuatedControl::FullyActuatedControl(ModuleParams *parent, bool vtol) :
	ModuleParams(parent),
	_vtol(vtol)
{
}

void FullyActuatedControl::updateSubscriptions()
{
	const bool enabled = px4::isFullyActuatedAirframe(_param_sys_autostart.get()) && !_vtol;

	if (enabled != _fullyactuated_enabled) {
		_fullyactuated_enabled = enabled;
		reset();
	}

	if (!_fullyactuated_enabled) {
		return;
	}

	vehicle_attitude_s vehicle_attitude{};

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		if ((_vehicle_attitude.timestamp != 0)
		    && (vehicle_attitude.quat_reset_counter != _vehicle_attitude.quat_reset_counter)) {
			_tilt_filter_initialized = false;
		}

		_vehicle_attitude = vehicle_attitude;
	}

	_manual_control_setpoint_sub.update(&_manual_control_setpoint);
}

void FullyActuatedControl::updateControlMode(const vehicle_control_mode_s &vehicle_control_mode)
{
	ControlContext control_context = ControlContext::Other;

	if (_fullyactuated_enabled
	    && vehicle_control_mode.flag_control_manual_enabled
	    && vehicle_control_mode.flag_control_altitude_enabled) {
		control_context = vehicle_control_mode.flag_control_position_enabled
				  ? ControlContext::Position
				  : ControlContext::Altitude;
	}

	if (control_context != _control_context) {
		_control_context = control_context;
		reset();
	}

	// MPC_FA_MODE and MPC_FA_RC_AUX select Position behavior only. Altitude
	// always uses mode 0, but the requested Position mode remains current.
	const int32_t requested_mode = _fullyactuated_enabled ? resolveMode(_mode) : 0;

	if (requested_mode != _mode) {
		_mode = requested_mode;

		if (control_context != ControlContext::Altitude) {
			reset();
		}
	}
}

void FullyActuatedControl::reset()
{
	_position_hold_valid = false;
	_tilt_filter_initialized = false;
	_output_requested = false;
}

FullyActuatedControl::DirectThrustConfiguration FullyActuatedControl::updateSetpoint(const matrix::Vector3f &position,
		bool flying, bool flying_but_ground_contact, trajectory_setpoint_s &setpoint)
{
	if (!_fullyactuated_enabled) {
		return {};
	}

	const bool manual_altitude_mode_active = (_control_context == ControlContext::Altitude);
	const bool manual_position_mode_active = (_control_context == ControlContext::Position);

	// Altitude always uses mode-0 semantics: roll/pitch remain level while the
	// normal XY stick acceleration is realized through direct Fx/Fy.
	_output_mode = manual_altitude_mode_active ? 0 : _mode;
	const bool manual_pose_mode_active = (_output_mode == 1)
					     && manual_position_mode_active
					     && flying && !flying_but_ground_contact;

	if (manual_pose_mode_active && position.isAllFinite()) {
		if (!_position_hold_valid) {
			_position_hold = position;
			_position_hold_valid = true;
		}

		_position_hold.copyTo(setpoint.position);
		matrix::Vector3f{}.copyTo(setpoint.velocity);
		matrix::Vector3f acceleration_setpoint{};
		acceleration_setpoint.setNaN();
		acceleration_setpoint.copyTo(setpoint.acceleration);

	} else {
		_position_hold_valid = false;
		_tilt_filter_initialized = false;
	}

	_output_requested = (_output_mode == 0) || (manual_pose_mode_active && _position_hold_valid);

	matrix::Quatf current_attitude{_vehicle_attitude.q};
	const bool current_attitude_valid = (_vehicle_attitude.timestamp != 0)
					    && (hrt_elapsed_time(&_vehicle_attitude.timestamp) <= 100_ms)
					    && current_attitude.isAllFinite()
					    && (current_attitude.norm_squared() > FLT_EPSILON);
	const bool direct_thrust_control_enabled = _output_requested && current_attitude_valid
			&& ((_output_mode != 1) || manualAttitudeInputValid());

	return DirectThrustConfiguration{
		direct_thrust_control_enabled,
		direct_thrust_control_enabled &&(_output_mode == 0),
		_param_mpc_fa_xy_thr.get(),
		_param_mpc_fa_xy_ratio.get()
	};
}

bool FullyActuatedControl::generateAttitudeSetpoint(const vehicle_local_position_setpoint_s &local_pos_sp, float dt,
		vehicle_attitude_setpoint_s &attitude_setpoint)
{
	if (!_output_requested) {
		return false;
	}

	if (generateAttitudeSetpointInternal(local_pos_sp, dt, attitude_setpoint)) {
		return true;
	}

	if (hrt_elapsed_time(&_last_invalid_output_warning) > 2_s) {
		PX4_WARN("full-actuated output invalid, using standard mapping");
		_last_invalid_output_warning = hrt_absolute_time();
	}

	return false;
}

bool FullyActuatedControl::generateAttitudeSetpointInternal(
	const vehicle_local_position_setpoint_s &local_pos_sp, float dt,
	vehicle_attitude_setpoint_s &attitude_setpoint)
{
	if ((_vehicle_attitude.timestamp == 0) || (hrt_elapsed_time(&_vehicle_attitude.timestamp) > 100_ms)) {
		return false;
	}

	matrix::Quatf q_current{_vehicle_attitude.q};

	if (!q_current.isAllFinite() || q_current.norm_squared() < FLT_EPSILON) {
		return false;
	}

	q_current.normalize();
	matrix::Quatf q_desired{};

	if (_output_mode == 0) {
		if (!PX4_ISFINITE(local_pos_sp.yaw)) {
			return false;
		}

		q_desired = matrix::Quatf{matrix::Eulerf{0.f, 0.f, local_pos_sp.yaw}};

	} else if (_output_mode == 1) {
		if (!manualAttitudeInputValid() || !PX4_ISFINITE(local_pos_sp.yaw)) {
			return false;
		}

		const float roll_input = math::expo_deadzone(_manual_control_setpoint.roll,
					 _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
		const float pitch_input = math::expo_deadzone(_manual_control_setpoint.pitch,
					  _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
		matrix::Vector2f tilt_target{roll_input, -pitch_input};

		if (tilt_target.norm() > 1.f) {
			tilt_target.normalize();
		}

		const float maximum_tilt = math::radians(math::constrain(_param_mpc_fa_tilt_max.get(), 1.f, 45.f));
		tilt_target *= maximum_tilt;

		if (!_tilt_filter_initialized) {
			const matrix::Eulerf current_euler{q_current};
			matrix::Vector2f current_tilt{current_euler.phi(), current_euler.theta()};

			if (current_tilt.norm() > maximum_tilt) {
				current_tilt = current_tilt.normalized() * maximum_tilt;
			}

			_tilt_filter.reset(current_tilt);
			_tilt_filter_initialized = true;
		}

		_tilt_filter.setParameters(dt, math::max(_param_mc_man_tilt_tau.get(), 0.f));
		const matrix::Vector2f tilt_setpoint = _tilt_filter.update(tilt_target);
		const matrix::Quatf q_roll_pitch{matrix::AxisAnglef{tilt_setpoint(0), tilt_setpoint(1), 0.f}};
		const matrix::Quatf q_yaw{cosf(local_pos_sp.yaw * 0.5f), 0.f, 0.f, sinf(local_pos_sp.yaw * 0.5f)};
		q_desired = q_yaw * q_roll_pitch;

	} else {
		return false;
	}

	if (!ControlMath::thrustNedToBody(matrix::Vector3f{local_pos_sp.thrust}, q_current, q_desired,
					  attitude_setpoint)) {
		return false;
	}

	attitude_setpoint.yaw_sp_move_rate = PX4_ISFINITE(local_pos_sp.yawspeed) ? local_pos_sp.yawspeed : 0.f;
	return true;
}

bool FullyActuatedControl::manualAttitudeInputValid() const
{
	return _manual_control_setpoint.valid
	       && (_manual_control_setpoint.timestamp != 0)
	       && (hrt_elapsed_time(&_manual_control_setpoint.timestamp) <= 500_ms)
	       && PX4_ISFINITE(_manual_control_setpoint.roll)
	       && PX4_ISFINITE(_manual_control_setpoint.pitch);
}

int32_t FullyActuatedControl::resolveMode(int32_t previous_mode) const
{
	const int32_t param_mode = math::constrain(_param_mpc_fa_mode.get(), (int32_t)0, (int32_t)1);
	const int32_t aux_channel = _param_mpc_fa_rc_aux.get();

	if ((aux_channel < 1) || (aux_channel > 6)) {
		return param_mode;
	}

	if (!_manual_control_setpoint.valid || (_manual_control_setpoint.timestamp == 0)
	    || (hrt_elapsed_time(&_manual_control_setpoint.timestamp) > 500_ms)) {
		return param_mode;
	}

	float aux = NAN;

	switch (aux_channel) {
	case 1:
		aux = _manual_control_setpoint.aux1;
		break;

	case 2:
		aux = _manual_control_setpoint.aux2;
		break;

	case 3:
		aux = _manual_control_setpoint.aux3;
		break;

	case 4:
		aux = _manual_control_setpoint.aux4;
		break;

	case 5:
		aux = _manual_control_setpoint.aux5;
		break;

	case 6:
		aux = _manual_control_setpoint.aux6;
		break;

	default:
		return param_mode;
	}

	if (!PX4_ISFINITE(aux)) {
		return param_mode;
	}

	if (aux < -0.2f) {
		return 0;

	} else if (aux > 0.2f) {
		return 1;
	}

	return math::constrain(previous_mode, (int32_t)0, (int32_t)1);
}

void FullyActuatedControl::adjustPositionHoldForEKFReset(const vehicle_local_position_s &vehicle_local_position,
		uint8_t xy_reset_counter, uint8_t z_reset_counter)
{
	if (!_position_hold_valid) {
		return;
	}

	if (vehicle_local_position.xy_reset_counter != xy_reset_counter) {
		_position_hold(0) += vehicle_local_position.delta_xy[0];
		_position_hold(1) += vehicle_local_position.delta_xy[1];
	}

	if (vehicle_local_position.z_reset_counter != z_reset_counter) {
		_position_hold(2) += vehicle_local_position.delta_z;
	}
}
