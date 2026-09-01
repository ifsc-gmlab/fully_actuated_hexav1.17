# Flying-car mode status published for system-level safety coordination.

uint64 timestamp # time since system start (microseconds)

uint8 MODE_FLIGHT = 0
uint8 MODE_TRANSITION_TO_GROUND = 1
uint8 MODE_GROUND = 2
uint8 MODE_TRANSITION_TO_FLIGHT = 3
uint8 MODE_FAULT = 4

uint8 REJECTION_NONE = 0
uint8 REJECTION_NOT_FLYING_CAR = 1
uint8 REJECTION_ARMED = 2
uint8 REJECTION_NOT_LANDED = 3
uint8 REJECTION_MOVING = 4
uint8 REJECTION_CHAIN_UNHEALTHY = 5
uint8 REJECTION_TIMEOUT = 6

uint8 mode # current mode, one of MODE_*
uint8 requested_mode # requested mode, one of MODE_*
uint8 rejection_reason # reason for refusing the request, one of REJECTION_*
bool transition_allowed # all safety gates currently allow a transition
bool flight_chain_ready # flight control chain is healthy and ready
bool ground_chain_ready # ground control chain is healthy and ready
