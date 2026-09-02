/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include "FlyingCar.hpp"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>

#include <algorithm>
FlyingCar::FlyingCar() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_local_position.vx = NAN;
	_local_position.vy = NAN;
	updateParams();
}

FlyingCar::~FlyingCar()
{
	_actuator_motors_sub.unregisterCallback();
	ScheduleClear();
}

bool FlyingCar::init()
{
	const uint64_t now_us = hrt_absolute_time();
	publishInitialSafeOutput(now_us);

	const FlyingCarTransitionResult initial{
		FlyingCarMode::Flight,
		FlyingCarRejection::None,
		false,
		false,
		false,
		false,
	};
	publishStatus(initial, false, false, now_us, true);

	if (!_actuator_motors_sub.registerCallback()) {
		PX4_WARN("actuator callback unavailable; retaining 100 Hz schedule");
	}

	ScheduleOnInterval(kRunIntervalUs);
	return true;
}

void FlyingCar::updateSubscriptions()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_land_detected_sub.update(&_land_detected);
	_vehicle_local_position_sub.update(&_local_position);

	if (_manual_control_setpoint_sub.update(&_manual_control)) {
		const std::array<float, 6> aux{
			_manual_control.aux1, _manual_control.aux2, _manual_control.aux3,
			_manual_control.aux4, _manual_control.aux5, _manual_control.aux6
		};
		_requested_mode = FlyingCarRuntimeHelpers::selectRcRequest(
			_requested_mode, _param_fc_mode_ch.get(), _manual_control.valid,
			_manual_control.data_source == manual_control_setpoint_s::SOURCE_RC, aux);
	}

	_actuator_motors_sub.update(&_actuator_motors);
	_rover_throttle_setpoint_sub.update(&_throttle_setpoint);
	_rover_steering_setpoint_sub.update(&_steering_setpoint);
}

void FlyingCar::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	updateSubscriptions();
	const uint64_t now_us = hrt_absolute_time();
	std::array<float, 4> rotor_controls{};
	std::copy_n(_actuator_motors.control, rotor_controls.size(), rotor_controls.begin());

	const bool flight_chain_ready = FlyingCarRuntimeHelpers::flightChainReady(
		now_us, _actuator_motors.timestamp, _actuator_motors.timestamp_sample, rotor_controls);
	const bool ground_chain_ready = FlyingCarRuntimeHelpers::isFresh(
			now_us, _throttle_setpoint.timestamp, FlyingCarRuntimeHelpers::kGroundInputTimeoutUs)
		&& FlyingCarRuntimeHelpers::isFresh(
			now_us, _steering_setpoint.timestamp, FlyingCarRuntimeHelpers::kGroundInputTimeoutUs);
	const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const uint64_t transition_delay_us = static_cast<uint64_t>(std::max(_param_fc_sw_delay.get(), 0.f) * 1e6f);

	const FlyingCarTransitionInput input{
		_param_sys_fc_type.get() == 1,
		armed,
		_land_detected.landed,
		flight_chain_ready,
		ground_chain_ready,
		FlyingCarRuntimeHelpers::horizontalSpeed(_local_position.vx, _local_position.vy),
		_requested_mode,
		now_us,
	};
	const FlyingCarTransitionResult transition = _mode_manager.update(
		input, _param_fc_sw_vel_max.get(), transition_delay_us, kTransitionTimeoutUs);

	publishActuators(transition, flight_chain_ready, ground_chain_ready, now_us);
	publishStatus(transition, flight_chain_ready, ground_chain_ready, now_us);
}

void FlyingCar::publishInitialSafeOutput(uint64_t now_us)
{
	flying_car_actuator_motors_s output{};
	output.timestamp = now_us;
	output.timestamp_sample = now_us;
	output.reversible_flags = kWheelReversibleFlags;

	for (float &control : output.control) {
		control = NAN;
	}

	output.control[4] = 0.f;
	output.control[5] = 0.f;
	_actuator_pub.publish(output);
}

void FlyingCar::publishActuators(const FlyingCarTransitionResult &transition, bool flight_chain_ready,
		bool ground_chain_ready, uint64_t now_us)
{
	std::array<float, 4> rotor_controls{NAN, NAN, NAN, NAN};

	if (flight_chain_ready) {
		std::copy_n(_actuator_motors.control, rotor_controls.size(), rotor_controls.begin());
	}

	matrix::Vector2f wheel_controls{0.f, 0.f};
	const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (transition.mode == FlyingCarMode::Ground && armed && ground_chain_ready) {
		wheel_controls = FlyingCarDifferentialControl::mix(
			_throttle_setpoint.throttle_body_x,
			_steering_setpoint.normalized_steering_setpoint,
			_param_fc_wheel_thr_max.get(),
			static_cast<uint8_t>(_param_fc_wheel_rev.get()));
	}

	// The runtime was allowed to start only for this physical configuration. If the parameter is
	// changed afterward, continue publishing safe values because the output provider is already latched.
	const bool configuration_enabled = _param_sys_fc_type.get() == 1;
	const FlyingCarActuatorGateOutput gated = FlyingCarActuatorGate::apply(
		configuration_enabled, transition.mode, rotor_controls, wheel_controls);

	flying_car_actuator_motors_s output{};
	output.timestamp = now_us;
	output.timestamp_sample = flight_chain_ready ? _actuator_motors.timestamp_sample : now_us;
	output.reversible_flags = gated.reversible_flags | kWheelReversibleFlags;

	for (float &control : output.control) {
		control = NAN;
	}

	for (size_t i = 0; i < gated.controls.size(); ++i) {
		output.control[i] = gated.controls[i];
	}

	_actuator_pub.publish(output);
}

void FlyingCar::publishStatus(const FlyingCarTransitionResult &transition, bool flight_chain_ready,
		bool ground_chain_ready, uint64_t now_us, bool force)
{
	flying_car_status_s status{};
	status.mode = static_cast<uint8_t>(transition.mode);
	status.requested_mode = static_cast<uint8_t>(_requested_mode);
	status.rejection_reason = static_cast<uint8_t>(transition.rejection);
	status.transition_allowed = transition.transition_allowed;
	status.flight_chain_ready = flight_chain_ready;
	status.ground_chain_ready = ground_chain_ready;

	const bool changed = !_have_status
		|| status.mode != _last_status.mode
		|| status.requested_mode != _last_status.requested_mode
		|| status.rejection_reason != _last_status.rejection_reason
		|| status.transition_allowed != _last_status.transition_allowed
		|| status.flight_chain_ready != _last_status.flight_chain_ready
		|| status.ground_chain_ready != _last_status.ground_chain_ready;

	if (force || changed || now_us - _last_status_publish >= kStatusIntervalUs) {
		status.timestamp = now_us;
		_status_pub.publish(status);
		_last_status = status;
		_last_status_publish = now_us;
		_have_status = true;
	}
}

int FlyingCar::task_spawn(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	int32_t configuration_type = 0;
	const param_t configuration_param = param_find("SYS_FC_TYPE");

	if (configuration_param == PARAM_INVALID || param_get(configuration_param, &configuration_type) != PX4_OK
	    || !FlyingCarRuntimeHelpers::configurationAllowsStart(configuration_type)) {
		PX4_ERR("refusing start: SYS_FC_TYPE must be 1");
		return PX4_ERROR;
	}

	FlyingCar *instance = new FlyingCar();

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	PX4_ERR("initialization failed");
	return PX4_ERROR;
}

int FlyingCar::print_status()
{
	PX4_INFO("running, requested mode: %u", static_cast<unsigned>(_requested_mode));
	return 0;
}

int FlyingCar::custom_command(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	return print_usage("unknown command");
}

int FlyingCar::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Owns the isolated flying-car mode state, wheel mixing, and gated physical-motor topic.
It starts only when SYS_FC_TYPE is 1 and never starts or stops another PX4 module.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("flying_car", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int flying_car_main(int argc, char *argv[])
{
	return FlyingCar::main(argc, argv);
}
