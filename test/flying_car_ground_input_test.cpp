#define FLYING_CAR_RUNTIME_HELPERS_ONLY
#include "src/modules/flying_car/FlyingCar.hpp"

#include <cassert>
#include <cmath>

int main()
{
	constexpr uint64_t now = 1'000'000;

	// The 80003 startup has no rover publisher: absent inputs must not become ready.
	auto input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, false, false,
		0, 0, NAN, NAN, 0, NAN, 0, NAN);
	assert(!input.ready && input.source == FlyingCarGroundSource::None);

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, true, true,
		900'000, 900'000, 0.25f, -0.5f, 0, NAN, 0, NAN);
	assert(input.source == FlyingCarGroundSource::ManualRc);
	assert(input.ready && input.throttle == 0.25f && input.steering == -0.5f);

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, false, true,
		900'000, 900'000, 0.25f, -0.5f, 0, NAN, 0, NAN);
	assert(!input.ready && input.source == FlyingCarGroundSource::None); // RC loss

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, false, true, true,
		900'000, 900'000, 0.25f, -0.5f, 0, NAN, 0, NAN);
	assert(!input.ready); // Flight request: sticks cannot drive wheels

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, true, true,
		100'000, 100'000, 0.25f, -0.5f, 950'000, 0.6f, 950'000, 0.2f);
	assert(input.ready && input.source == FlyingCarGroundSource::Rover);
	assert(input.throttle == 0.6f && input.steering == 0.2f); // paired rover source wins

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, true, true,
		900'000, 900'000, 0.25f, -0.5f, 950'000, 0.6f, 100'000, 0.2f);
	assert(input.ready && input.source == FlyingCarGroundSource::ManualRc); // no mixed rover pair

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, true, true,
		900'000, 900'000, NAN, -0.5f, 0, NAN, 0, NAN);
	assert(!input.ready); // NaN manual input

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, true, true,
		900'000, 100'000, 0.25f, -0.5f, 0, NAN, 0, NAN);
	assert(!input.ready); // stale raw sample even if publication is fresh

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, true, true,
		1'000'001, 900'000, 0.25f, -0.5f, 0, NAN, 0, NAN);
	assert(!input.ready); // future publication timestamps are invalid

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, true, true, true, true,
		900'000, 900'000, 0.25f, -0.5f, 950'000, NAN, 950'000, 0.2f);
	assert(input.ready && input.source == FlyingCarGroundSource::ManualRc); // invalid rover pair cannot win

	input = FlyingCarRuntimeHelpers::selectGroundInput(now, false, true, true, true,
		900'000, 900'000, 0.25f, -0.5f, 950'000, 0.6f, 950'000, 0.2f);
	assert(!input.ready); // ordinary configurations have zero behavior
}
