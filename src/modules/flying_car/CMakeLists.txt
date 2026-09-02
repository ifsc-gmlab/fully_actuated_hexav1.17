############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name PX4 nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
#
############################################################################

px4_add_library(flying_car_mode_manager
	FlyingCarModeManager.cpp
	FlyingCarModeManager.hpp
	FlyingCarTypes.hpp
)

px4_add_unit_gtest(
	SRC FlyingCarModeManagerTest.cpp
	LINKLIBS flying_car_mode_manager
)

px4_add_library(flying_car_differential_control
	FlyingCarDifferentialControl.cpp
	FlyingCarDifferentialControl.hpp
)

px4_add_unit_gtest(
	SRC FlyingCarDifferentialControlTest.cpp
	LINKLIBS flying_car_differential_control
)

px4_add_library(flying_car_actuator_gate
	FlyingCarActuatorGate.cpp
	FlyingCarActuatorGate.hpp
	FlyingCarTypes.hpp
)

px4_add_unit_gtest(
	SRC FlyingCarActuatorGateTest.cpp
	LINKLIBS flying_car_actuator_gate
)
