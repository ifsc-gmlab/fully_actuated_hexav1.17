/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#include "FullyActuatedControlAllocation.hpp"

#include <ActuatorEffectivenessMultirotor.hpp>
#include <ControlAllocationPseudoInverse.hpp>
#include <ControlAllocationPseudoInverseFullyActuated.hpp>
#include <px4_platform_common/log.h>

FullyActuatedControlAllocation::FullyActuatedControlAllocation(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FullyActuatedControlAllocation::updateAirframe()
{
	const bool enabled = px4::isFullyActuatedAirframe(_param_sys_autostart.get());
	const bool changed = (enabled != _enabled);
	_enabled = enabled;
	return changed;
}

ControlAllocation *FullyActuatedControlAllocation::createPseudoInverse() const
{
	if (_enabled) {
		return new ControlAllocationPseudoInverseFullyActuated();
	}

	return new ControlAllocationPseudoInverse();
}

ActuatorEffectiveness *FullyActuatedControlAllocation::createEffectivenessSource(ModuleParams *parent) const
{
	if (_enabled) {
		return new ActuatorEffectivenessMultirotor(parent,
				ActuatorEffectivenessRotors::AxisConfiguration::FixedFullyActuatedHexa);
	}

	PX4_WARN("CA_AIRFRAME=16 ignored outside airframes 6003/4026/22000");
	return new ActuatorEffectivenessMultirotor(parent);
}
