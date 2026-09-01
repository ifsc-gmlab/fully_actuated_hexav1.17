/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include "FlyingCarModeManager.hpp"

#include <cmath>

FlyingCarTransitionResult FlyingCarModeManager::update(const FlyingCarTransitionInput &input,
		float maximum_speed_m_s, uint64_t transition_delay_us, uint64_t transition_timeout_us)
{
	if (!input.configuration_enabled) {
		clearPendingRequest();
		return result(FlyingCarRejection::NotFlyingCar, false, false);
	}

	if (_mode == FlyingCarMode::Fault) {
		return result(FlyingCarRejection::Timeout, false, true);
	}

	if (_mode == FlyingCarMode::TransitionToGround || _mode == FlyingCarMode::TransitionToFlight) {
		if (elapsedTime(input.now_us, _transition_started_us) > transition_timeout_us) {
			_mode = FlyingCarMode::Fault;
			return result(FlyingCarRejection::Timeout, false, true);
		}

		const bool target_chain_ready = _mode == FlyingCarMode::TransitionToGround
						? input.ground_chain_ready : input.flight_chain_ready;

		if (!target_chain_ready) {
			return result(FlyingCarRejection::ChainUnhealthy, false, true);
		}

		if (elapsedTime(input.now_us, _transition_started_us) >= transition_delay_us) {
			_mode = _mode == FlyingCarMode::TransitionToGround ? FlyingCarMode::Ground : FlyingCarMode::Flight;
		}

		return result(FlyingCarRejection::None, true, true);
	}

	if (input.requested_mode == _mode) {
		clearPendingRequest();
		return result(FlyingCarRejection::None, false, true);
	}

	if (input.requested_mode != FlyingCarMode::Flight && input.requested_mode != FlyingCarMode::Ground) {
		clearPendingRequest();
		return result(FlyingCarRejection::ChainUnhealthy, false, true);
	}

	if (input.armed) {
		clearPendingRequest();
		return result(FlyingCarRejection::Armed, false, true);
	}

	if (!input.landed) {
		clearPendingRequest();
		return result(FlyingCarRejection::NotLanded, false, true);
	}

	if (!std::isfinite(input.horizontal_speed_m_s)
	    || std::fabs(input.horizontal_speed_m_s) > maximum_speed_m_s) {
		clearPendingRequest();
		return result(FlyingCarRejection::Moving, false, true);
	}

	const bool target_chain_ready = input.requested_mode == FlyingCarMode::Ground
					? input.ground_chain_ready : input.flight_chain_ready;

	if (!target_chain_ready) {
		clearPendingRequest();
		return result(FlyingCarRejection::ChainUnhealthy, false, true);
	}

	if (!_pending_request_active || _pending_requested_mode != input.requested_mode) {
		_pending_request_active = true;
		_pending_requested_mode = input.requested_mode;
		_pending_since_us = input.now_us;
	}

	if (elapsedTime(input.now_us, _pending_since_us) >= transition_delay_us) {
		_mode = input.requested_mode == FlyingCarMode::Ground
			? FlyingCarMode::TransitionToGround : FlyingCarMode::TransitionToFlight;
		_transition_started_us = input.now_us;
		clearPendingRequest();
	}

	return result(FlyingCarRejection::None, true, true);
}

bool FlyingCarModeManager::resetFault(bool armed, bool flight_chain_ready)
{
	if (_mode != FlyingCarMode::Fault || armed || !flight_chain_ready) {
		return false;
	}

	_mode = FlyingCarMode::Flight;
	_transition_started_us = 0;
	clearPendingRequest();
	return true;
}

FlyingCarTransitionResult FlyingCarModeManager::result(FlyingCarRejection rejection, bool transition_allowed,
		bool configuration_enabled) const
{
	const bool arming_locked = _mode == FlyingCarMode::TransitionToGround
				   || _mode == FlyingCarMode::TransitionToFlight || _mode == FlyingCarMode::Fault;

	return {
		_mode,
		rejection,
		transition_allowed,
		arming_locked,
		configuration_enabled && _mode == FlyingCarMode::Flight,
		configuration_enabled && _mode == FlyingCarMode::Ground,
	};
}

void FlyingCarModeManager::clearPendingRequest()
{
	_pending_requested_mode = _mode;
	_pending_since_us = 0;
	_pending_request_active = false;
}

uint64_t FlyingCarModeManager::elapsedTime(uint64_t now_us, uint64_t started_us)
{
	return now_us >= started_us ? now_us - started_us : 0;
}
