/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 4.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 4.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_P, 2.8f);

/**
 * Yaw weight
 *
 * A fraction [0,1] deprioritizing yaw compared to roll and pitch in non-linear attitude control.
 * Deprioritizing yaw is necessary because multicopters have much less control authority
 * in yaw compared to the other axes and it makes sense because yaw is not critical for
 * stable hovering or 3D navigation.
 *
 * For yaw control tuning use MC_YAW_P. This ratio has no impact on the yaw gain.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_WEIGHT, 0.4f);

/**
 * Max roll rate
 *
 * Limit for roll rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_MAX, 220.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * Max yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX, 200.0f);

/**
 * Manual tilt input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_MAN_TILT_TAU, 0.0f);

/**
 * Fully actuated Stabilized mode
 *
 * 0: Off — standard Stabilized tilt mapping.
 * 1: Air — lock roll/pitch to zero; roll/pitch sticks command horizontal
 *    thrust in the yaw-setpoint frame (forward/right). Throttle sets Fz;
 *    |Fxy| is also capped by MPC_FA_XY_RATIO * |Fz|.
 * 2: Ground taxi — follow the ground-constrained roll/pitch attitude;
 *    roll/pitch sticks command body Fx/Fy. The controller adds the minimum
 *    upward force needed to keep the request inside the unilateral-rotor
 *    feasible cone. Throttle is ignored.
 *
 * Yaw stick keeps the normal Stabilized mapping in all cases.
 * Requires a control-effectiveness matrix with controllable Fx and Fy.
 * Only SYS_AUTOSTART 6003, 6004, 4026, and 22000 consume this parameter; all other airframes
 * always use the standard Stabilized tilt mapping. Ignored for VTOL attitude control.
 * Ignored when MPC_FA_STAB_AUX selects an AUX input.
 *
 * @value 0 Off
 * @value 1 Air (level + horizontal force)
 * @value 2 Ground taxi
 * @min 0
 * @max 2
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(MPC_FA_STAB, 0);

/**
 * RC AUX input selector for MPC_FA_STAB
 *
 * 0: disabled — use MPC_FA_STAB parameter only.
 * 1..6 selects the logical AUX1..AUX6 input, not a physical RC channel.
 * First map the physical switch to AUXn with RC_MAP_AUXn. For example, to use
 * physical RC channel 8 through AUX1, set RC_MAP_AUX1=8 and this parameter=1.
 * When enabled, AUX is authoritative and MPC_FA_STAB is ignored.
 *
 * Three-position switch mapping follows the MPC_FA_STAB mode numbers:
 *   AUX in [-1, -0.6] → mode 0 (off)
 *   AUX in [-0.2, 0.2] → mode 1 (air)
 *   AUX in [0.6, 1] → mode 2 (ground taxi)
 * Values in the gaps keep the previous valid AUX mode (hysteresis). An
 * invalid/unmapped AUX, or a gap before the first valid position, selects off.
 *
 * @min 0
 * @max 6
 * @value 0 Disabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(MPC_FA_STAB_AUX, 0);

/**
 * Max lateral thrust for fully actuated level flight
 *
 * Absolute magnitude limit for horizontal thrust in fully actuated Stabilized
 * air mode and in level direct-thrust Position/Altitude mode. The effective
 * limit is also capped by MPC_FA_XY_RATIO * |Fz|. In ground taxi mode it is
 * instead coupled to MPC_FA_GND_XY_Z and MPC_FA_GND_ZMAX.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MPC_FA_XY_THR, 0.15f);

/**
 * Ground taxi horizontal-to-vertical force ratio
 *
 * Maximum |Fxy| / |Fz| used by the ground taxi setpoint generator. This must
 * not exceed the physical force-cone ratio of the rotor geometry and should
 * include margin for torque allocation. Although the rotor-axis force cone has
 * an upper bound near 0.90, the fixed fully actuated hex geometry can guarantee
 * only about 0.35 in every direction while also commanding zero body torque.
 *
 * @min 0.05
 * @max 0.90
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MPC_FA_GND_XY_Z, 0.30f);

/**
 * Ground taxi maximum upward force
 *
 * Maximum normalized upward force magnitude allowed while generating ground
 * taxi horizontal force. The controller additionally caps this value at 80%
 * of MPC_THR_HOVER. Neutral roll/pitch sticks always command zero force.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MPC_FA_GND_ZMAX, 0.20f);

/**
 * Lateral-to-vertical thrust ratio for fully actuated level flight
 *
 * In Stabilized air mode and level direct-thrust Position/Altitude mode,
 * horizontal thrust is capped to this fraction of the commanded |Fz|. Keep
 * below the geometry's zero-torque force-cone boundary so torque authority
 * remains for holding level attitude.
 * Ignored in ground taxi mode (MPC_FA_STAB=2), which uses MPC_FA_GND_XY_Z.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MPC_FA_XY_RATIO, 0.30f);
