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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/systemlib/fully_actuated_airframe.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

class FullyActuatedControl : public ModuleParams
{
public:
	struct DirectThrustConfiguration {
		bool enabled{false};
		bool limits_enabled{false};
		float horizontal_limit{0.f};
		float horizontal_to_vertical_ratio{0.f};
	};

	FullyActuatedControl(ModuleParams *parent, bool vtol);

	/** Update the attitude and manual-control inputs owned by this controller. */
	void updateSubscriptions();

	/** Resolve the active fully actuated context and MPC_FA mode. */
	void updateControlMode(const vehicle_control_mode_s &vehicle_control_mode);

	/** Reset attitude- and position-dependent state across controller transitions. */
	void reset();

	/**
	 * Apply Pose position hold when requested and return direct-thrust settings.
	 */
	DirectThrustConfiguration updateSetpoint(const matrix::Vector3f &position, bool flying,
			bool flying_but_ground_contact, trajectory_setpoint_s &setpoint);

	/**
	 * Generate an independent attitude and body-thrust setpoint when FA output is active.
	 *
	 * @return true if a fully actuated setpoint was generated, false when the caller
	 *         must use the conventional position-controller attitude mapping.
	 */
	bool generateAttitudeSetpoint(const vehicle_local_position_setpoint_s &local_pos_sp, float dt,
				      vehicle_attitude_setpoint_s &attitude_setpoint);

	/** Keep the internal Pose position hold aligned with EKF local-frame resets. */
	void adjustPositionHoldForEKFReset(const vehicle_local_position_s &vehicle_local_position,
					   uint8_t xy_reset_counter, uint8_t z_reset_counter);

private:
	enum class ControlContext : uint8_t {
		Other = 0,
		Altitude,
		Position
	};

	int32_t resolveMode(int32_t previous_mode) const;
	bool generateAttitudeSetpointInternal(const vehicle_local_position_setpoint_s &local_pos_sp, float dt,
					      vehicle_attitude_setpoint_s &attitude_setpoint);
	bool manualAttitudeInputValid() const;

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_attitude_s _vehicle_attitude{};

	matrix::Vector3f _position_hold{};
	AlphaFilter<matrix::Vector2f> _tilt_filter{};

	hrt_abstime _last_invalid_output_warning{0};
	int32_t _mode{0};
	int32_t _output_mode{0};
	ControlContext _control_context{ControlContext::Other};
	bool _position_hold_valid{false};
	bool _tilt_filter_initialized{false};
	bool _output_requested{false};
	bool _fullyactuated_enabled{false};
	const bool _vtol;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>)       _param_sys_autostart,
		(ParamInt<px4::params::MPC_FA_MODE>)         _param_mpc_fa_mode,
		(ParamInt<px4::params::MPC_FA_RC_AUX>)       _param_mpc_fa_rc_aux,
		(ParamFloat<px4::params::MPC_FA_TILT_MAX>)   _param_mpc_fa_tilt_max,
		(ParamFloat<px4::params::MPC_FA_XY_THR>)     _param_mpc_fa_xy_thr,
		(ParamFloat<px4::params::MPC_FA_XY_RATIO>)   _param_mpc_fa_xy_ratio,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>)   _param_mc_man_tilt_tau,
		(ParamFloat<px4::params::MPC_HOLD_DZ>)       _param_mpc_hold_dz,
		(ParamFloat<px4::params::MPC_XY_MAN_EXPO>)   _param_mpc_xy_man_expo
	)
};
