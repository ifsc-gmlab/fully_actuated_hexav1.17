#pragma once

#include <cstdint>

class FlyingCarSafety
{
public:
	enum class Mode : uint8_t {
		Flight = 0,
		TransitionToGround = 1,
		Ground = 2,
		TransitionToFlight = 3,
		Fault = 4,
	};

	enum class StableType : uint8_t { Flight, Ground };

	struct Result { bool arming_locked; bool status_fresh; };

	static constexpr Result evaluate(bool enabled, bool received, uint64_t now, uint64_t timestamp, Mode mode)
	{
		const bool fresh = received && timestamp <= now && now - timestamp <= 1'000'000;
		const bool stable = mode == Mode::Flight || mode == Mode::Ground;
		return {enabled && (!fresh || !stable), fresh};
	}

	static constexpr bool armingEntryAllowed(bool enabled, bool arming_locked)
	{
		return !enabled || !arming_locked;
	}

	void acceptStableMode(Mode mode)
	{
		if (mode == Mode::Flight) { _stable_type = StableType::Flight; }
		else if (mode == Mode::Ground) { _stable_type = StableType::Ground; }
	}

	StableType stableType() const { return _stable_type; }
	void reset() { _stable_type = StableType::Flight; }

private:
	StableType _stable_type{StableType::Flight};
};
