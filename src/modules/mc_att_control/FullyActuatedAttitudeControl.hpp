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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/systemlib/fully_actuated_airframe.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/fully_actuated_control_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

/**
 * Fully actuated manual-attitude modes for airframes 6003, 6004, 4026, and 22000.
 *
 * This class owns all MPC_FA_* parameters, mode state and force-setpoint
 * generation used by mc_att_control. The conventional attitude controller only
 * supplies common inputs and uses the generated setpoint when this controller
 * reports that a fully actuated mode is active.
 */
class FullyActuatedAttitudeControl : public ModuleParams
{
public:
	struct SetpointInput {
		matrix::Quatf attitude{};
		float roll{0.f};
		float pitch{0.f};
		float yaw_setpoint{0.f};
		float yaw_sp_move_rate{0.f};
		float vertical_thrust{0.f};
		float hover_thrust{0.f};
		float maximum_thrust{0.f};
		float manual_deadzone{0.f};
		float tilt_time_constant{0.f};
		float dt{0.f};
	};

	explicit FullyActuatedAttitudeControl(ModuleParams *parent);

	/**
	 * Resolve the requested mode and apply the manual/landed safety gates.
	 *
	 * @return true when the active output mode changed. The caller uses this to
	 *         reset its conventional stick filters at mapping transitions.
	 */
	bool updateMode(const manual_control_setpoint_s &manual_control,
			bool stabilized_manual_control, bool landed, bool vtol);

	/** Generate the attitude/body-force setpoint for the active FA mode. */
	bool generateAttitudeSetpoint(const SetpointInput &input,
				      vehicle_attitude_setpoint_s &attitude_setpoint);

	enum Mode : int32_t {
		MODE_OFF = 0,
		MODE_AIR = 1,
		MODE_GROUND_TAXI = 2
	};

	enum ModeSource : uint8_t {
		SOURCE_PARAMETER = 0,
		SOURCE_AUX = 1
	};

	bool active() const { return _active_mode != MODE_OFF; }

	void printStatus() const;

private:
	int32_t resolveRequestedMode(const manual_control_setpoint_s &manual_control,
				     bool manual_control_valid);
	bool manualControlValid(const manual_control_setpoint_s &manual_control) const;
	float selectedAuxValue(const manual_control_setpoint_s &manual_control, int32_t aux_channel) const;
	void clearState();
	void publishStatus();

	uORB::Publication<fully_actuated_control_status_s> _status_pub{ORB_ID(fully_actuated_control_status)};

	AlphaFilter<float> _roll_input_filter;
	AlphaFilter<float> _pitch_input_filter;

	int32_t _requested_mode{MODE_OFF};
	int32_t _active_mode{MODE_OFF};
	uint8_t _mode_source{SOURCE_PARAMETER};
	uint8_t _aux_index{0};
	bool _aux_valid{false};
	bool _aux_selection_valid{false};
	bool _ground_taxi_active{false};
	bool _airframe_enabled{false};
	float _aux_value{NAN};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
		(ParamInt<px4::params::MPC_FA_STAB>) _param_mpc_fa_stab,
		(ParamInt<px4::params::MPC_FA_STAB_AUX>) _param_mpc_fa_stab_aux,
		(ParamFloat<px4::params::MPC_FA_XY_THR>) _param_mpc_fa_xy_thr,
		(ParamFloat<px4::params::MPC_FA_XY_RATIO>) _param_mpc_fa_xy_ratio,
		(ParamFloat<px4::params::MPC_FA_GND_XY_Z>) _param_mpc_fa_gnd_xy_z,
		(ParamFloat<px4::params::MPC_FA_GND_ZMAX>) _param_mpc_fa_gnd_zmax
	)
};
