# 全驱动 Position 控制 — 代码改动

目标：位置环输出完整 NED 三维推力，姿态设定值独立；`Fx/Fy/Fz` 统一归一化尺度。

| `MPC_FA_MODE` | 行为 |
|---:|---|
| 0 | 锁 roll/pitch，三轴推力改位置，yaw 可控 |
| 1 | 锁 XYZ，摇杆控姿态（Pose） |

---

## 1. `ControlMath.hpp` — 新增声明

```cpp
bool thrustNedToBody(const matrix::Vector3f &thr_sp_ned,
		     const matrix::Quatf &q_current, const matrix::Quatf &q_desired,
		     vehicle_attitude_setpoint_s &att_sp);
```

## 2. `ControlMath.cpp` — 新增函数

```cpp
bool thrustNedToBody(const Vector3f &thr_sp_ned, const Quatf &q_current,
		     const Quatf &q_desired, vehicle_attitude_setpoint_s &att_sp)
{
	const float current_norm_sq = q_current.norm_squared();
	const float desired_norm_sq = q_desired.norm_squared();

	if (!thr_sp_ned.isAllFinite() || !q_current.isAllFinite() || !q_desired.isAllFinite()
	    || current_norm_sq < FLT_EPSILON || desired_norm_sq < FLT_EPSILON) {
		return false;
	}

	// 必须用 q_current 做坐标变换，不能用 q_desired
	Quatf q_current_normalized{q_current};
	q_current_normalized.normalize();

	Quatf q_desired_normalized{q_desired};
	q_desired_normalized.normalize();

	const Vector3f thrust_body = q_current_normalized.rotateVectorInverse(thr_sp_ned);

	if (!thrust_body.isAllFinite()) {
		return false;
	}

	q_desired_normalized.copyTo(att_sp.q_d);
	thrust_body.copyTo(att_sp.thrust_body);
	return true;
}
```

## 3. `PositionControl.hpp` — 新增

```cpp
void setIndependentThrustControl(bool enabled) { _independent_thrust_control = enabled; }

// private:
void _accelerationControlIndependent();
bool _independent_thrust_control{false};
```

## 4. `PositionControl.cpp` — 修改 + 新增

**修改 `_velocityControl()`：**

```cpp
	if (_independent_thrust_control) {
		_accelerationControlIndependent();

	} else {
		_accelerationControl();
	}
```

**新增：**

```cpp
void PositionControl::_accelerationControlIndependent()
{
	const float acceleration_to_thrust = _hover_thrust / CONSTANTS_ONE_G;
	_thr_sp.xy() = _acc_sp.xy() * acceleration_to_thrust;
	_thr_sp(2) = _acc_sp(2) * acceleration_to_thrust - _hover_thrust;
	_thr_sp(2) = math::min(_thr_sp(2), -_lim_thr_min);
}
```

## 5. `MulticopterPositionControl.hpp` — 新增成员 / 订阅 / 参数 / 函数

```cpp
	const bool _vtol;

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_attitude_s _vehicle_attitude{};
	matrix::Quatf _full_actuated_attitude_hold{};
	matrix::Vector3f _full_actuated_position_hold{};
	bool _full_actuated_attitude_hold_valid{false};
	bool _full_actuated_position_hold_valid{false};
	bool _full_actuated_tilt_filter_initialized{false};
	int32_t _full_actuated_mode{0};

	// DEFINE_PARAMETERS 内新增：
	(ParamInt<px4::params::MPC_FA_MODE>)         _param_mpc_fa_mode,
	(ParamInt<px4::params::MPC_FA_RC_AUX>)       _param_mpc_fa_rc_aux,
	(ParamFloat<px4::params::MPC_FA_TILT_MAX>)   _param_mpc_fa_tilt_max,
	(ParamFloat<px4::params::MC_MAN_TILT_TAU>)  _param_mc_man_tilt_tau,
	(ParamFloat<px4::params::MPC_HOLD_DZ>)       _param_mpc_hold_dz,
	(ParamFloat<px4::params::MPC_XY_MAN_EXPO>)   _param_mpc_xy_man_expo,

	AlphaFilter<matrix::Vector2f> _full_actuated_tilt_filter{};
	hrt_abstime _last_full_actuated_warn{0};

	bool generateFullActuatedAttitudeSetpoint(const vehicle_local_position_setpoint_s &local_pos_sp,
			const float dt, vehicle_attitude_setpoint_s &attitude_setpoint);
	int32_t resolveFullActuatedMode(int32_t previous_mode) const;
```

## 6. `MulticopterPositionControl.cpp` — 核心新增 / 修改

### 6.1 构造函数

```cpp
MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_vtol(vtol)
{
	...
}
```

### 6.2 `generateFullActuatedAttitudeSetpoint()` — 新增

```cpp
bool MulticopterPositionControl::generateFullActuatedAttitudeSetpoint(
	const vehicle_local_position_setpoint_s &local_pos_sp, const float dt,
	vehicle_attitude_setpoint_s &attitude_setpoint)
{
	if ((_vehicle_attitude.timestamp == 0) || (hrt_elapsed_time(&_vehicle_attitude.timestamp) > 100_ms)) {
		_full_actuated_attitude_hold_valid = false;
		return false;
	}

	matrix::Quatf q_current{_vehicle_attitude.q};

	if (!q_current.isAllFinite() || q_current.norm_squared() < FLT_EPSILON) {
		_full_actuated_attitude_hold_valid = false;
		return false;
	}

	q_current.normalize();
	matrix::Quatf q_desired{};

	if (_full_actuated_mode == 1) {
		if (!PX4_ISFINITE(local_pos_sp.yaw)) {
			return false;
		}

		if (!_full_actuated_attitude_hold_valid) {
			_full_actuated_attitude_hold = q_current;
			_full_actuated_attitude_hold_valid = true;
		}

		const matrix::Eulerf held_attitude{_full_actuated_attitude_hold};
		q_desired = matrix::Quatf{matrix::Eulerf{held_attitude.phi(), held_attitude.theta(), local_pos_sp.yaw}};

	} else if (_full_actuated_mode == 2) {
		if (!_manual_control_setpoint.valid || (_manual_control_setpoint.timestamp == 0)
		    || (hrt_elapsed_time(&_manual_control_setpoint.timestamp) > 500_ms)
		    || !PX4_ISFINITE(_manual_control_setpoint.roll)
		    || !PX4_ISFINITE(_manual_control_setpoint.pitch)
		    || !PX4_ISFINITE(local_pos_sp.yaw)) {
			return false;
		}

		const float roll_input = math::expo_deadzone(_manual_control_setpoint.roll,
					 _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
		const float pitch_input = math::expo_deadzone(_manual_control_setpoint.pitch,
					  _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
		matrix::Vector2f tilt_target{roll_input, -pitch_input};

		if (tilt_target.norm() > 1.f) {
			tilt_target.normalize();
		}

		const float maximum_tilt = math::radians(math::constrain(_param_mpc_fa_tilt_max.get(), 1.f, 45.f));
		tilt_target *= maximum_tilt;

		if (!_full_actuated_tilt_filter_initialized) {
			const matrix::Eulerf current_euler{q_current};
			matrix::Vector2f current_tilt{current_euler.phi(), current_euler.theta()};

			if (current_tilt.norm() > maximum_tilt) {
				current_tilt = current_tilt.normalized() * maximum_tilt;
			}

			_full_actuated_tilt_filter.reset(current_tilt);
			_full_actuated_tilt_filter_initialized = true;
		}

		_full_actuated_tilt_filter.setParameters(dt, math::max(_param_mc_man_tilt_tau.get(), 0.f));
		const matrix::Vector2f tilt_setpoint = _full_actuated_tilt_filter.update(tilt_target);
		const matrix::Quatf q_roll_pitch{matrix::AxisAnglef{tilt_setpoint(0), tilt_setpoint(1), 0.f}};
		const matrix::Quatf q_yaw{cosf(local_pos_sp.yaw * 0.5f), 0.f, 0.f, sinf(local_pos_sp.yaw * 0.5f)};
		q_desired = q_yaw * q_roll_pitch;

	} else {
		return false;
	}

	if (!ControlMath::thrustNedToBody(matrix::Vector3f{local_pos_sp.thrust}, q_current, q_desired,
			attitude_setpoint)) {
		return false;
	}

	attitude_setpoint.yaw_sp_move_rate = PX4_ISFINITE(local_pos_sp.yawspeed) ? local_pos_sp.yawspeed : 0.f;
	return true;
}
```

### 6.3 `resolveFullActuatedMode()` — 新增

```cpp
int32_t MulticopterPositionControl::resolveFullActuatedMode(int32_t previous_mode) const
{
	const int32_t param_mode = math::constrain(_param_mpc_fa_mode.get(), (int32_t)0, (int32_t)2);
	const int32_t aux_channel = _param_mpc_fa_rc_aux.get();

	if ((aux_channel < 1) || (aux_channel > 6)) {
		return param_mode;
	}

	if (!_manual_control_setpoint.valid || (_manual_control_setpoint.timestamp == 0)
	    || (hrt_elapsed_time(&_manual_control_setpoint.timestamp) > 500_ms)) {
		return param_mode;
	}

	float aux = NAN;

	switch (aux_channel) {
	case 1: aux = _manual_control_setpoint.aux1; break;
	case 2: aux = _manual_control_setpoint.aux2; break;
	case 3: aux = _manual_control_setpoint.aux3; break;
	case 4: aux = _manual_control_setpoint.aux4; break;
	case 5: aux = _manual_control_setpoint.aux5; break;
	case 6: aux = _manual_control_setpoint.aux6; break;
	default: return param_mode;
	}

	if (!PX4_ISFINITE(aux)) {
		return param_mode;
	}

	if (aux < -0.8f) {
		return 0;
	} else if ((aux >= -0.2f) && (aux <= 0.2f)) {
		return 1;
	} else if (aux > 0.8f) {
		return 2;
	}

	return math::constrain(previous_mode, (int32_t)0, (int32_t)2);
}
```

### 6.4 `Run()` — 关键插入点

**姿态订阅 + 模式解析：**

```cpp
		vehicle_attitude_s vehicle_attitude{};

		if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
			if ((_vehicle_attitude.timestamp != 0)
			    && (vehicle_attitude.quat_reset_counter != _vehicle_attitude.quat_reset_counter)) {
				_full_actuated_attitude_hold_valid = false;
				_full_actuated_tilt_filter_initialized = false;
			}

			_vehicle_attitude = vehicle_attitude;
		}

		_manual_control_setpoint_sub.update(&_manual_control_setpoint);

		const int32_t requested_full_actuated_mode = resolveFullActuatedMode(_full_actuated_mode);

		if (requested_full_actuated_mode != _full_actuated_mode) {
			_full_actuated_mode = requested_full_actuated_mode;
			_full_actuated_attitude_hold_valid = false;
			_full_actuated_position_hold_valid = false;
			_full_actuated_tilt_filter_initialized = false;
		}

		// 位置控使能开关变化时同样清 lock / tilt 滤波：
		// _full_actuated_attitude_hold_valid = false;
		// _full_actuated_position_hold_valid = false;
		// _full_actuated_tilt_filter_initialized = false;
```

**Pose 锁位 + 独立推力开关：**

```cpp
			const bool manual_pose_mode_active = !_vtol && (_full_actuated_mode == 2)
							     && _vehicle_control_mode.flag_control_manual_enabled
							     && flying && !flying_but_ground_contact;

			if (manual_pose_mode_active && states.position.isAllFinite()) {
				if (!_full_actuated_position_hold_valid) {
					_full_actuated_position_hold = states.position;
					_full_actuated_position_hold_valid = true;
				}

				_full_actuated_position_hold.copyTo(_setpoint.position);
				matrix::Vector3f{}.copyTo(_setpoint.velocity);
				matrix::Vector3f acceleration_setpoint{};
				acceleration_setpoint.setNaN();
				acceleration_setpoint.copyTo(_setpoint.acceleration);

			} else {
				_full_actuated_position_hold_valid = false;
				_full_actuated_tilt_filter_initialized = false;
			}

			const bool full_actuated_requested = !_vtol
							     && (_full_actuated_mode == 1
									     || (manual_pose_mode_active && _full_actuated_position_hold_valid));
			const matrix::Quatf current_attitude{_vehicle_attitude.q};
			const bool current_attitude_valid = (_vehicle_attitude.timestamp != 0)
							    && (hrt_elapsed_time(&_vehicle_attitude.timestamp) <= 100_ms)
							    && current_attitude.isAllFinite()
							    && (current_attitude.norm_squared() > FLT_EPSILON);
			const bool manual_attitude_input_valid = (_full_actuated_mode != 2)
					|| (_manual_control_setpoint.valid
					    && (_manual_control_setpoint.timestamp != 0)
					    && (hrt_elapsed_time(&_manual_control_setpoint.timestamp) <= 500_ms)
					    && PX4_ISFINITE(_manual_control_setpoint.roll)
					    && PX4_ISFINITE(_manual_control_setpoint.pitch));
			_control.setIndependentThrustControl(full_actuated_requested && current_attitude_valid
							     && manual_attitude_input_valid);
```

**发布姿态设定值：**

```cpp
			vehicle_attitude_setpoint_s attitude_setpoint{};

			if (!full_actuated_requested
			    || !generateFullActuatedAttitudeSetpoint(local_pos_sp, dt, attitude_setpoint)) {
				_control.getAttitudeSetpoint(attitude_setpoint);

				if (full_actuated_requested
				    && (hrt_elapsed_time(&_last_full_actuated_warn) > 2_s)) {
					PX4_WARN("full-actuated output invalid, using standard mapping");
					_last_full_actuated_warn = hrt_absolute_time();
				}
			}

			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
```

### 6.5 `adjustSetpointForEKFResets()` — 新增开头

```cpp
	if (_full_actuated_position_hold_valid) {
		if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
			_full_actuated_position_hold(0) += vehicle_local_position.delta_xy[0];
			_full_actuated_position_hold(1) += vehicle_local_position.delta_xy[1];
		}

		if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
			_full_actuated_position_hold(2) += vehicle_local_position.delta_z;
		}
	}
```

## 7. `multicopter_position_control_params.c` — 新增参数

```c
PARAM_DEFINE_INT32(MPC_FA_MODE, 0);
// 0 水平全驱动 / 1 Pose

PARAM_DEFINE_INT32(MPC_FA_RC_AUX, 0);
// 0 禁用；1..6 对应 AUX1..6
// AUX < -0.2 → 0；> 0.2 → 1；中间带滞回

PARAM_DEFINE_FLOAT(MPC_FA_TILT_MAX, 15.f);
```

## 8. `multicopter_position_mode_params.c` — 回补（1.17 Pose 摇杆整形需要）

```c
PARAM_DEFINE_FLOAT(MPC_HOLD_DZ, 0.1f);
PARAM_DEFINE_FLOAT(MPC_XY_MAN_EXPO, 0.6f);
```

## 9. 控制分配 — `Fx/Fy/Fz` 统一尺度

### `ControlAllocationPseudoInverse.cpp` — 替换推力归一化段

```cpp
	// Use one shared scale for thrust X/Y/Z on all airframes
	float thrust_scale = 1.f;

	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		int num_non_zero_thrust = 0;
		float norm_sum = 0.f;

		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(_mix(i, THRUST_X + axis_idx));
			norm_sum += norm;

			if (norm > FLT_EPSILON) {
				++num_non_zero_thrust;
			}
		}

		if (num_non_zero_thrust > 0) {
			thrust_scale = norm_sum / num_non_zero_thrust;
			break;
		}
	}

	_control_allocation_scale(THRUST_X) = thrust_scale;
	_control_allocation_scale(THRUST_Y) = thrust_scale;
	_control_allocation_scale(THRUST_Z) = thrust_scale;
```

### `ControlAllocation.hpp` — 新增

```cpp
	virtual bool getMixMatrix(matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mix) { return false; }
	const matrix::Vector<float, NUM_AXES> &getControlAllocationScale() const { return _control_allocation_scale; }
```

### `ControlAllocationPseudoInverse.hpp` — 新增

```cpp
	bool getMixMatrix(matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mix) override
	{
		updatePseudoInverse();
		mix = _mix;
		return true;
	}
```

### `ControlAllocator.cpp` — `print_status` 增加

```cpp
		if (_control_allocation[i]->getMixMatrix(mix)) {
			PX4_INFO("  Mix (normalized) =");
			mix.print();
			PX4_INFO("  Allocation scale (roll pitch yaw Fx Fy Fz) =");
			_control_allocation[i]->getControlAllocationScale().T().print();
		}
```

## 10. 单元测试 — 新增用例

### `ControlMathTest.cpp`

```cpp
TEST(ControlMathTest, IndependentThrustAttitudeMappingLevel)
{
	const Vector3f thrust_ned{0.2f, -0.1f, -0.7f};
	const Quatf q_current{};
	const Quatf q_desired{Eulerf{0.f, 0.f, M_PI_2_F}};
	vehicle_attitude_setpoint_s att{};

	ASSERT_TRUE(thrustNedToBody(thrust_ned, q_current, q_desired, att));
	EXPECT_NEAR(att.thrust_body[0], thrust_ned(0), 1e-6f);
	EXPECT_NEAR(att.thrust_body[1], thrust_ned(1), 1e-6f);
	EXPECT_NEAR(att.thrust_body[2], thrust_ned(2), 1e-6f);
	...
}

TEST(ControlMathTest, IndependentThrustAttitudeMappingRotated) { ... }
TEST(ControlMathTest, IndependentThrustAttitudeMappingRejectsInvalidInput) { ... }
```

### `PositionControlTest.cpp`

```cpp
TEST_F(PositionControlBasicTest, IndependentThrustBypassesTiltCone) { ... }
TEST_F(PositionControlBasicTest, IndependentThrustMapsAxesDirectly) { ... }
```

### `ControlAllocationPseudoInverseTest.cpp`

```cpp
TEST(ControlAllocationTest, ThrustVectorNormalizationPreservesDirection) { ... }
TEST(ControlAllocationTest, ThrustVectorNormalizationUsesAvailableAxis) { ... }
```

## 11. 机架 `6003_fully_actuated_hexa` / `4026_gz_fully_actuated_hexa`

量产机架脚本：`ROMFS/px4fmu_common/init.d/airframes/6003_fully_actuated_hexa`
SITL 包装：`ROMFS/.../init.d-posix/airframes/4026_gz_fully_actuated_hexa`（source 6003）

```sh
param set-default CA_AIRFRAME 16   # Fully Actuated Hexarotor（推力轴固件写死）
param set-default CA_ROTOR_COUNT 6
# 仅保留位置 / 转向：CA_ROTOR*_PX/PY/PZ/KM
# CA_ROTOR*_AX/AY/AZ 已从机架脚本删除（见 §14）

param set-default CA_METHOD 0
param set-default MPC_FA_MODE 1
param set-default MPC_FA_TILT_MAX 10.0
param set-default MC_YAW_TQ_CUTOFF 0.0
```

Gazebo 模型：`Tools/simulation/gz/models/fully_actuated_hexa/`。

## 12. 控制链路

```
位置/速度 PID → NED 推力 T_N
  └─ mode 0/1 → _accelerationControlIndependent()
                 + thrustNedToBody(q_cur, q_des)
                 → (q_d, thrust_body[3])
                     → att/rate → control_allocator（Fx/Fy/Fz 统一尺度）→ 电机
  （姿态无效时回退 thrustToAttitude）
```

## 13. 编译 / 启动

```sh
make px4_sitl gz_fully_actuated_hexa
# 或
CCACHE_DIR=/tmp/px4_ccache cmake --build build/px4_sitl_default -j2

param show MPC_FA_MODE
param set MPC_FA_MODE 0   # 水平全驱动：锁姿态改位置
param set MPC_FA_MODE 1   # Pose：锁 XYZ，摇杆控姿态
param set MPC_FA_RC_AUX 1 # 可选：AUX1 两段开关覆盖
```

## 14. 商业化：推力轴 `CA_ROTOR*_AX/AY/AZ` 固件写死

目标：全矢量六旋翼的推力轴方向不进参数、不在 QGC 展示，避免被改。

### 做法

新增 `CA_AIRFRAME = 16`（Fully Actuated Hexarotor），轴向量改为 C++ 常量；普通 `Multirotor(0)` 仍可读参数，互不影响。

| 文件　　　　　　　　　　　　　　　　　　　 | 改动　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　|
| --------------------------------------------| -------------------------------------------------------------------|
| `ActuatorEffectivenessRotors.hpp`　　　　　| `AxisConfiguration` 增加 `FixedFullyActuatedHexa`　　　　　　　　 |
| `ActuatorEffectivenessRotors.cpp`　　　　　| `k_axes[6][3]` 硬编码 FRD 推力轴；该模式下不 `param_get` AX/AY/AZ |
| `ActuatorEffectivenessMultirotor.hpp/.cpp` | 构造函数可传入 `AxisConfiguration`　　　　　　　　　　　　　　　　|
| `ControlAllocator.hpp`　　　　　　　　　　 | `EffectivenessSource::FULLY_ACTUATED_HEXA = 16`　　　　　　　　　 |
| `ControlAllocator.cpp`　　　　　　　　　　 | case 16 → Multirotor + `FixedFullyActuatedHexa`　　　　　　　　　 |
| `control_allocator/module.yaml`　　　　　　| 枚举增加 16；type 16 执行器 UI **无** Axis X/Y/Z（仅位置 + KM）　 |
| `6003_fully_actuated_hexa`　　　　　　　　 | `CA_AIRFRAME 16`；删除全部 `CA_ROTOR*_AX/AY/AZ`　　　　　　　　　 |

### 硬编码轴（与原机架参数一致，FRD）

```cpp
// ActuatorEffectivenessRotors.cpp — FixedFullyActuatedHexa
static constexpr float k_axes[6][3] = {
	{-0.519837f, -0.422618f, -0.742404f}, // M1 right CW
	{-0.519837f,  0.422618f, -0.742404f}, // M2 left CCW
	{-0.106080f,  0.661501f, -0.742404f}, // M3 front-left CW
	{ 0.625917f,  0.238883f, -0.742404f}, // M4 rear-right CCW
	{-0.106080f, -0.661501f, -0.742404f}, // M5 front-right CCW
	{ 0.625917f, -0.238883f, -0.742404f}, // M6 rear-left CW
};
```

### 说明

- **改轴**：改 `k_axes` 后重新编译烧录，不能再靠 QGC / `param set`。
- **QGC**：选 `CA_AIRFRAME=16` 时执行器页不显示 Axis；位置 `PX/PY/PZ`、转向 `KM` 仍可配。
- **参数元数据**：`CA_ROTOR*_AX/AY/AZ` 定义仍保留（UUV / Spacecraft 等机型还要用）；本机架路径不读、不展示。
- 旧闪存若残留 Axis 参数可忽略；建议确认 `CA_AIRFRAME` 为 16，必要时参数重置。
