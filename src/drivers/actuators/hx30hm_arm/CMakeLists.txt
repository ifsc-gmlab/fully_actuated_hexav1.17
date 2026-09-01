############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE drivers__actuators__hx30hm_arm
	MAIN arm_control
	STACK_MAIN 3200
	SRCS
		arm_control.cpp
	MODULE_CONFIG
		module.yaml
)
