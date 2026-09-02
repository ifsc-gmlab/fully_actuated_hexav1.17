/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include "FlyingCarModeManager.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

class FlyingCarRuntimeHelpers
{
public:
	static constexpr bool configurationAllowsStart(int32_t configuration_type)
	{
		return configuration_type == 1;
	}

	static FlyingCarMode selectRcRequest(FlyingCarMode previous_request, int32_t mode_channel,
			bool manual_valid, bool from_rc, const std::array<float, 6> &aux)
	{
		if (mode_channel < 1 || mode_channel > static_cast<int32_t>(aux.size()) || !manual_valid || !from_rc) {
			return previous_request;
		}

		const float value = aux[static_cast<size_t>(mode_channel - 1)];

		if (std::isfinite(value) && value < -0.5f) {
			return FlyingCarMode::Flight;
		}

		if (std::isfinite(value) && value > 0.5f) {
			return FlyingCarMode::Ground;
		}

		return previous_request;
	}

	static bool isFresh(uint64_t now_us, uint64_t timestamp_us, uint64_t timeout_us)
	{
		return timestamp_us != 0 && now_us >= timestamp_us && now_us - timestamp_us <= timeout_us;
	}

	static float horizontalSpeed(float vx, float vy)
	{
		return std::isfinite(vx) && std::isfinite(vy) ? std::sqrt(vx * vx + vy * vy)
		       : std::numeric_limits<float>::infinity();
	}

	static bool flightChainReady(uint64_t now_us, uint64_t publication_timestamp_us, uint64_t sample_timestamp_us,
			const std::array<float, 4> &rotor_controls)
	{
		if (!isFresh(now_us, publication_timestamp_us, kFlightInputTimeoutUs)
		    || !isFresh(now_us, sample_timestamp_us, kFlightInputTimeoutUs)) {
			return false;
		}

		for (float control : rotor_controls) {
			if (!std::isfinite(control)) {
				return false;
			}
		}

		return true;
	}

	static constexpr uint64_t kFlightInputTimeoutUs{200'000};
	static constexpr uint64_t kGroundInputTimeoutUs{500'000};
};

#ifndef FLYING_CAR_RUNTIME_HELPERS_ONLY

#include "FlyingCarActuatorGate.hpp"
#include "FlyingCarDifferentialControl.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/flying_car_actuator_motors.h>
#include <uORB/topics/flying_car_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

class FlyingCar : public ModuleBase<FlyingCar>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FlyingCar();
	~FlyingCar() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

private:
	void Run() override;
	void updateSubscriptions();
	void publishActuators(const FlyingCarTransitionResult &transition, bool flight_chain_ready,
			bool ground_chain_ready, uint64_t now_us);
	void publishInitialSafeOutput(uint64_t now_us);
	void publishStatus(const FlyingCarTransitionResult &transition, bool flight_chain_ready,
			bool ground_chain_ready, uint64_t now_us, bool force = false);

	static constexpr uint64_t kRunIntervalUs{10'000};
	static constexpr uint64_t kStatusIntervalUs{500'000};
	static constexpr uint64_t kTransitionTimeoutUs{5'000'000};
	static constexpr uint16_t kWheelReversibleFlags{(1u << 4) | (1u << 5)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1'000'000};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionCallbackWorkItem _actuator_motors_sub{this, ORB_ID(actuator_motors)};
	uORB::Subscription _rover_throttle_setpoint_sub{ORB_ID(rover_throttle_setpoint)};
	uORB::Subscription _rover_steering_setpoint_sub{ORB_ID(rover_steering_setpoint)};

	uORB::Publication<flying_car_actuator_motors_s> _actuator_pub{ORB_ID(flying_car_actuator_motors)};
	uORB::Publication<flying_car_status_s> _status_pub{ORB_ID(flying_car_status)};

	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	vehicle_local_position_s _local_position{};
	manual_control_setpoint_s _manual_control{};
	actuator_motors_s _actuator_motors{};
	rover_throttle_setpoint_s _throttle_setpoint{};
	rover_steering_setpoint_s _steering_setpoint{};

	FlyingCarModeManager _mode_manager{};
	FlyingCarMode _requested_mode{FlyingCarMode::Flight};
	flying_car_status_s _last_status{};
	uint64_t _last_status_publish{0};
	bool _have_status{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_FC_TYPE>) _param_sys_fc_type,
		(ParamInt<px4::params::FC_MODE_CH>) _param_fc_mode_ch,
		(ParamFloat<px4::params::FC_SW_VEL_MAX>) _param_fc_sw_vel_max,
		(ParamFloat<px4::params::FC_SW_DELAY>) _param_fc_sw_delay,
		(ParamFloat<px4::params::FC_WHEEL_THR_MAX>) _param_fc_wheel_thr_max,
		(ParamInt<px4::params::FC_WHEEL_REV>) _param_fc_wheel_rev
	)
};

#endif // FLYING_CAR_RUNTIME_HELPERS_ONLY
