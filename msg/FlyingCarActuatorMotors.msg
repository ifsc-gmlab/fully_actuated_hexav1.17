# Gated physical-motor input for the flying-car output provider.
# This is an actuator output message, not a controller setpoint.

uint64 timestamp         # [us] Time since system start
uint64 timestamp_sample  # [us] Sampling timestamp of the source control response

uint16 reversible_flags  # [-] Bitset indicating reversible motors

float32[12] control      # [@range -1, 1] Normalized output; NaN disables the motor
