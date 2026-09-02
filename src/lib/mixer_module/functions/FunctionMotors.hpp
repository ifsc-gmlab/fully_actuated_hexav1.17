/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#include "FunctionProviderBase.hpp"

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/flying_car_actuator_motors.h>

/**
 * Functions: Motor1 ... MotorMax
 */
class FunctionMotors : public FunctionProviderBase
{
public:
	static_assert(actuator_motors_s::NUM_CONTROLS == (int)OutputFunction::MotorMax - (int)OutputFunction::Motor1 + 1,
		      "Unexpected num motors");

	static_assert(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 == (int)OutputFunction::Motor1, "Unexpected motor idx");

	FunctionMotors(const Context &context) :
		_topic(&context.work_item, ORB_ID(actuator_motors)),
		_flying_car_topic(&context.work_item, ORB_ID(flying_car_actuator_motors)),
		_thrust_factor(context.thrust_factor)
	{
		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			_data.control[i] = NAN;
		}
	}

	~FunctionMotors() override
	{
		if (_flying_car_callback_registered) {
			_flying_car_topic.unregisterCallback();
		}
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionMotors(context); }

	void update() override
	{
		_updated = false;
		flying_car_actuator_motors_s flying_car_data{};

		// The dedicated gated source wins the first time it appears and remains selected until reboot.
		if (_flying_car_topic.update(&flying_car_data)) {
			_flying_car_source_latched = true;
			_flying_car_source_stale = false;
			_last_flying_car_update = hrt_absolute_time();
			_data.timestamp = flying_car_data.timestamp;
			_data.timestamp_sample = flying_car_data.timestamp_sample;
			_data.reversible_flags = flying_car_data.reversible_flags;

			for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
				_data.control[i] = flying_car_data.control[i];
			}

			updateValues(_data.reversible_flags, _thrust_factor, _data.control, actuator_motors_s::NUM_CONTROLS);
			_updated = true;

		} else if (!_flying_car_source_latched && _topic.update(&_data)) {
			updateValues(_data.reversible_flags, _thrust_factor, _data.control, actuator_motors_s::NUM_CONTROLS);
			_updated = true;
		}

		if (_flying_car_source_latched) {
			// Drain the standard callback without allowing it to overwrite the latched gated source.
			actuator_motors_s ignored_standard_data{};
			_topic.update(&ignored_standard_data);
			const hrt_abstime now = hrt_absolute_time();
			const hrt_abstime elapsed = now >= _last_flying_car_update ? now - _last_flying_car_update : 0;

			if (!_flying_car_source_stale && elapsed >= kFlyingCarSourceTimeoutUs) {
				setFlyingCarSafeValues(now);
				_flying_car_source_stale = true;
				_updated = true;
			}
		}
	}

	bool updated() const override { return _updated; }

	float value(OutputFunction func) override { return _data.control[(int)func - (int)OutputFunction::Motor1]; }

#if defined(CONFIG_DRIVERS_UAVCANNODE)
	bool allowPrearmControl() const override { return true; }
#else
	bool allowPrearmControl() const override { return false; }
#endif

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override
	{
		// Keep the standard callback as the provider callback so ordinary configurations behave exactly as before.
		// Register the dormant dedicated callback as well so its first sample can schedule the same work item.
		if (!_flying_car_callback_registered) {
			_flying_car_callback_registered = _flying_car_topic.registerCallback();
		}

		return &_topic;
	}

	bool getLatestSampleTimestamp(hrt_abstime &t) const override { t = _data.timestamp_sample; return t != 0; }

	static inline void updateValues(uint32_t reversible, float thrust_factor, float *values, int num_values)
	{
		if (thrust_factor > FLT_EPSILON && thrust_factor <= 1.f) {
			// thrust factor
			//  rel_thrust = factor * x^2 + (1-factor) * x,
			const float a = thrust_factor;
			const float b = (1.f - thrust_factor);

			// don't recompute for all values (ax^2+bx+c=0)
			const float tmp1 = b / (2.f * a);
			const float tmp2 = b * b / (4.f * a * a);

			for (int i = 0; i < num_values; ++i) {

				const float control = values[i];

				if (PX4_ISFINITE(control)) {
					if (control > FLT_EPSILON) {
						values[i] = -tmp1 + sqrtf(tmp2 + (control / a));

					} else if (control < -FLT_EPSILON) {
						values[i] =  tmp1 - sqrtf(tmp2 - (control / a));

					}
				}
			}
		}

		for (int i = 0; i < num_values; ++i) {
			if ((reversible & (1u << i)) == 0) {
				if (values[i] < -FLT_EPSILON) {
					values[i] = NAN;

				} else {
					// remap from [0, 1] to [-1, 1]
					values[i] = values[i] * 2.f - 1.f;
				}
			}
		}
	}

	bool reversible(OutputFunction func) const override { return _data.reversible_flags & (1u << ((int)func - (int)OutputFunction::Motor1)); }

private:
	void setFlyingCarSafeValues(hrt_abstime now)
	{
		_data.timestamp = now;
		_data.timestamp_sample = now;
		_data.reversible_flags = (1u << 4) | (1u << 5);

		for (float &control : _data.control) {
			control = NAN;
		}

		_data.control[4] = 0.f;
		_data.control[5] = 0.f;
		updateValues(_data.reversible_flags, _thrust_factor, _data.control, actuator_motors_s::NUM_CONTROLS);
	}

	static constexpr hrt_abstime kFlyingCarSourceTimeoutUs{200'000};
	uORB::SubscriptionCallbackWorkItem _topic;
	uORB::SubscriptionCallbackWorkItem _flying_car_topic;
	actuator_motors_s _data{};
	bool _flying_car_source_latched{false};
	bool _flying_car_source_stale{false};
	bool _updated{false};
	bool _flying_car_callback_registered{false};
	hrt_abstime _last_flying_car_update{0};
	const float &_thrust_factor;
};
