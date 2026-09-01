/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include "FlyingCarTypes.hpp"

#include <cstdint>

using FlyingCarRejection = FlyingCarRejectionReason;

struct FlyingCarTransitionInput {
	bool configuration_enabled;
	bool armed;
	bool landed;
	bool flight_chain_ready;
	bool ground_chain_ready;
	float horizontal_speed_m_s;
	FlyingCarMode requested_mode;
	uint64_t now_us;
};

struct FlyingCarTransitionResult {
	FlyingCarMode mode;
	FlyingCarRejection rejection;
	bool transition_allowed;
	bool arming_locked;
	bool flight_output_enabled;
	bool ground_output_enabled;
};

class FlyingCarModeManager
{
public:
	FlyingCarModeManager() = default;

	/**
	 * Advance the deterministic mode state machine.
	 *
	 * @param input current safety inputs and monotonic time in microseconds
	 * @param maximum_speed_m_s maximum allowed absolute horizontal speed in metres per second
	 * @param transition_delay_us request debounce and output-safe transition dwell in microseconds
	 * @param transition_timeout_us maximum transition-state duration in microseconds
	 */
	FlyingCarTransitionResult update(const FlyingCarTransitionInput &input, float maximum_speed_m_s,
					 uint64_t transition_delay_us, uint64_t transition_timeout_us);

	/** Reset a latched Fault only while disarmed and with a healthy flight chain. */
	bool resetFault(bool armed, bool flight_chain_ready);

private:
	FlyingCarTransitionResult result(FlyingCarRejection rejection, bool transition_allowed,
					 bool configuration_enabled) const;
	void clearPendingRequest();
	static uint64_t elapsedTime(uint64_t now_us, uint64_t started_us);

	FlyingCarMode _mode{FlyingCarMode::Flight};
	FlyingCarMode _pending_requested_mode{FlyingCarMode::Flight};
	uint64_t _pending_since_us{0};
	uint64_t _transition_started_us{0};
	bool _pending_request_active{false};
};
