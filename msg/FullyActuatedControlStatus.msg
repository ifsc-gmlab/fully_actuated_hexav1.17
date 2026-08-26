# Fully actuated control status published by mc_att_control.
# Authoritative state consumed by Commander to suppress auto-disarm in ground taxi.

uint64 timestamp # time since system start (microseconds)

bool ground_taxi_active # true while ground taxi mode is active
