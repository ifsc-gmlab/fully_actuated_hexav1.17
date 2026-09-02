#include "../src/modules/commander/FlyingCarSafety.hpp"

#include <cassert>
#include <cstdint>

int main()
{
	constexpr uint64_t now = 2'000'000;
	using Safety = FlyingCarSafety;

	assert(!Safety::evaluate(false, false, now, 0, Safety::Mode::Fault).arming_locked);
	assert(Safety::evaluate(true, false, now, 0, Safety::Mode::Flight).arming_locked);
	assert(Safety::evaluate(true, true, now, now - 1'000'001, Safety::Mode::Flight).arming_locked);
	assert(Safety::evaluate(true, true, now, now + 1, Safety::Mode::Flight).arming_locked);
	assert(!Safety::evaluate(true, true, now, now - 1'000'000, Safety::Mode::Flight).arming_locked);
	assert(!Safety::evaluate(true, true, now, now, Safety::Mode::Ground).arming_locked);
	assert(Safety::evaluate(true, true, now, now, Safety::Mode::TransitionToGround).arming_locked);
	assert(Safety::evaluate(true, true, now, now, Safety::Mode::TransitionToFlight).arming_locked);
	assert(Safety::evaluate(true, true, now, now, Safety::Mode::Fault).arming_locked);

	Safety tracker;
	assert(tracker.stableType() == Safety::StableType::Flight);
	tracker.acceptStableMode(Safety::Mode::Ground);
	assert(tracker.stableType() == Safety::StableType::Ground);
	tracker.acceptStableMode(Safety::Mode::Fault);
	assert(tracker.stableType() == Safety::StableType::Ground);
	tracker.reset();
	assert(tracker.stableType() == Safety::StableType::Flight);
}
