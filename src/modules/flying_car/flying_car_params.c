/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * Flying-car physical configuration.
 *
 * @value 0 Non-flying-car
 * @value 1 Flying-car
 * @group Flying Car
 */
PARAM_DEFINE_INT32(SYS_FC_TYPE, 0);

/**
 * Flying-car mode switch RC channel. Set to 0 to disable RC switching.
 *
 * @min 0
 * @max 18
 * @group Flying Car
 */
PARAM_DEFINE_INT32(FC_MODE_CH, 0);

/**
 * Flying-car boot mode. The initial implementation supports Flight only.
 *
 * @value 0 Flight
 * @group Flying Car
 */
PARAM_DEFINE_INT32(FC_BOOT_MODE, 0);

/**
 * Maximum local horizontal speed that permits a mode switch.
 *
 * @unit m/s
 * @min 0
 * @max 5
 * @increment 0.01
 * @decimal 2
 * @group Flying Car
 */
PARAM_DEFINE_FLOAT(FC_SW_VEL_MAX, 0.2f);

/**
 * Safety-output hold and request debounce duration before a mode switch.
 *
 * @unit s
 * @min 0
 * @max 10
 * @increment 0.1
 * @decimal 2
 * @group Flying Car
 */
PARAM_DEFINE_FLOAT(FC_SW_DELAY, 0.5f);

/**
 * Distance between the centers of the left and right wheels.
 *
 * @unit m
 * @min 0.01
 * @max 10
 * @increment 0.01
 * @decimal 2
 * @group Flying Car
 */
PARAM_DEFINE_FLOAT(FC_WHEEL_TRACK, 0.5f);

/**
 * Maximum commanded ground speed for the differential wheel controller.
 *
 * @unit m/s
 * @min 0
 * @max 20
 * @increment 0.1
 * @decimal 2
 * @group Flying Car
 */
PARAM_DEFINE_FLOAT(FC_WHEEL_SPD_MAX, 2.0f);

/**
 * Maximum magnitude of the normalized wheel output.
 *
 * @unit norm
 * @min 0
 * @max 1
 * @increment 0.01
 * @decimal 2
 * @group Flying Car
 */
PARAM_DEFINE_FLOAT(FC_WHEEL_THR_MAX, 0.5f);

/**
 * Wheel direction reversal bitmask: bit 0 is left and bit 1 is right.
 *
 * @min 0
 * @max 3
 * @group Flying Car
 */
PARAM_DEFINE_INT32(FC_WHEEL_REV, 0);
