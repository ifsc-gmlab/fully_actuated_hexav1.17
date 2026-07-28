/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Vertical thrust required to hover
 *
 * Mapped to center throttle stick in Stabilized mode (see MPC_THR_CURVE).
 * Used for initialization of the hover thrust estimator (see MPC_USE_HTE).
 * The estimated hover thrust is used as base for zero vertical acceleration in altitude control.
 * The hover thrust is important for land detection to work correctly.
 *
 * @unit norm
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 0.5f);

/**
 * Fully actuated position-control output mode
 *
 * Selects whether the position controller maps horizontal thrust to vehicle
 * tilt, or sends all three body-frame thrust components independently from the
 * attitude setpoint. The independent modes require a control-effectiveness
 * matrix with controllable Fx, Fy, Fz, roll, pitch and yaw axes.
 *
 * Mode 1 commands level roll and pitch while retaining the normal yaw setpoint.
 * Mode 2 is for manual Position flight after takeoff: it holds the captured XYZ
 * position and maps roll, pitch and yaw sticks to attitude commands. Throttle
 * is ignored while the XYZ position is locked.
 *
 * The parameter is ignored for VTOL position control. Invalid or stale attitude
 * data causes an automatic fallback to the standard tilt-based mapping.
 *
 * When MPC_FA_RC_AUX is set to a non-zero AUX channel, that RC switch overrides
 * this parameter.
 *
 * @value 0 Standard tilt-based multicopter mapping
 * @value 1 Independent thrust, level roll/pitch and controlled yaw
 * @value 2 Hold XYZ position and control attitude with manual sticks
 * @min 0
 * @max 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_FA_MODE, 0);

/**
 * Fully actuated mode RC AUX channel
 *
 * Selects which manual_control_setpoint AUXn input overrides MPC_FA_MODE.
 * Map the physical RC switch with RC_MAP_AUXn first.
 *
 * Three-position switch mapping (normalized AUX in [-1, 1]):
 *   AUX < -0.8           -> mode 0 (standard)
 *   AUX in [-0.2, 0.2]   -> mode 1 (independent thrust, level attitude)
 *   AUX >  0.8           -> mode 2 (hold XYZ and control attitude)
 * Values between the bands keep the previous mode (hysteresis).
 * If the AUX input is invalid or stale, MPC_FA_MODE is used.
 *
 * @value 0 Disabled (use MPC_FA_MODE)
 * @value 1 Aux1
 * @value 2 Aux2
 * @value 3 Aux3
 * @value 4 Aux4
 * @value 5 Aux5
 * @value 6 Aux6
 * @min 0
 * @max 6
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_FA_RC_AUX, 0);

/**
 * Maximum manual tilt in fully actuated pose mode
 *
 * Limits the combined roll and pitch command generated from manual sticks when
 * MPC_FA_MODE is set to 2. A conservative limit is required because the
 * position controller must generate lateral body thrust to hold XYZ while the
 * vehicle is tilted.
 *
 * @unit deg
 * @min 1
 * @max 45
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_FA_TILT_MAX, 15.f);

/**
 * Use hover thrust estimate for altitude control
 *
 * Disable to use the fixed parameter MPC_THR_HOVER instead of the hover thrust estimate in the position controller.
 * This parameter does not influence Stabilized mode throttle curve (see MPC_THR_CURVE).
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_USE_HTE, 1);

/**
 * Horizontal thrust margin
 *
 * Margin that is kept for horizontal control when higher priority vertical thrust is saturated.
 * To avoid completely starving horizontal control with high vertical error.
 *
 * @unit norm
 * @min 0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_XY_MARG, 0.3f);

/**
 * Velocity low pass cutoff frequency
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_LP, 0.0f);

/**
 * Velocity notch filter frequency
 *
 * The center frequency for the 2nd order notch filter on the velocity.
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_NF_FRQ, 0.0f);

/**
 * Velocity notch filter bandwidth
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_NF_BW, 5.0f);

/**
 * Velocity derivative low pass cutoff frequency
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VELD_LP, 5.0f);
