# HX-30HM arm control on Pixhawk 6C

This PX4 v1.17 driver is ported from the verified PX4 v1.13.3 package. It
controls three HX-30HM serial-bus servos through Pixhawk 6C TELEM2 and a
BusLinker. It does not generate PWM servo outputs.

## Wiring and configuration

- Pixhawk 6C TELEM2 (`/dev/ttyS3`) TX/RX/GND connects to the BusLinker UART.
- The BusLinker connects to HX-30HM servo IDs 1, 2, and 3.
- Use an external supply sized for the servos and connect grounds together.
  Do not power the servo bus from a Pixhawk serial-port power pin.
- Select airframe `6004 Fully Actuated Hexarotor + HX-30HM Arm`.
- The airframe reserves TELEM2 (`ARM_SER_CFG=102`) and maps receiver channels
  5, 7, and 9 to `RC_MAP_AUX3`, `RC_MAP_AUX4`, and `RC_MAP_AUX5` by default.
  Recheck these mappings after radio calibration.

The driver configures the selected port for 1,000,000 baud, 8 data bits, no
parity, and 1 stop bit. The v1.17 serial-port framework supplies the device path,
so the implementation is not tied to a hard-coded NuttX UART name.

## Build and flash

```sh
make px4_fmu-v6c_arm PYTHON_EXECUTABLE=/usr/bin/python3
```

Flash `build/px4_fmu-v6c_arm/px4_fmu-v6c_arm.px4`, then select airframe 6004
and reboot. The airframe starts the driver with RC control enabled after the
serial-device configuration is applied.

## Safe bench check

Remove propellers, support the arm, and initially keep the servo torque off.
The following commands are available in the MAVLink/NSH console:

```sh
arm_control status
arm_control ping
arm_control read
arm_control torque off
arm_control torque on
arm_control zero
arm_control joint 0 0 0
arm_control raw 3117 1033 2059
arm_control rc on
arm_control rc off
```

`AUX3` commands joint 1 over -90 to +90 degrees, `AUX4` commands joint 2 over
-120 to +120 degrees, and `AUX5` switches the gripper between 90 and 0 degrees.
Commands are sent as HX-30HM `SYNC_WRITE` frames.

The retained calibration is:

| Servo ID | Zero raw | Direction | Joint limit |
|---:|---:|---:|---:|
| 1 | 3117 | -1 | -90 to +90 deg |
| 2 | 1033 | +1 | -120 to +120 deg |
| 3 | 2059 | +1 | 0 to +90 deg |

Confirm the linkage direction and mechanical zero before enabling RC control.
Change `ZERO`, `DIR`, or the joint limits in `arm_control.cpp` only after a
torque-off measurement and a low-speed bench test.
