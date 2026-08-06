# 全驱动 Position 控制 — 替换 / 新增了什么

目标：位置环输出完整 NED 三维推力，姿态设定值独立；`Fx/Fy/Fz` 统一归一化尺度。

**仿真机架 / 模型（本快照仅此一套）：**
- 机架：`ROMFS/.../airframes/4026_gz_fully_actuated_hexa`
- Gazebo：`Tools/simulation/gz/models/fully_actuated_hexa/`
- 启动：`make px4_sitl gz_fully_actuated_hexa`

| `MPC_FA_MODE` | 行为 |
|---:|---|
| 0 | 标准 tilt 映射（原路径） |
| 1 | 锁 roll/pitch，三轴推力改位置，yaw 可控 |
| 2 | 锁 XYZ，摇杆控姿态（Pose；4026 默认） |

---

## 0. 替换对照总表

| 位置 | 原行为（被替换 / 被旁路） | 全驱动后 |
|------|---------------------------|----------|
| `PositionControl::_velocityControl()` | 始终调用 `_accelerationControl()` | FA 开时改走 `_accelerationControlIndependent()` |
| `_accelerationControl()` | 水平力压进 tilt cone，再靠倾斜产生水平力 | 保留作 FA off 回退；FA on **不调用** |
| `ControlMath::thrustToAttitude()` | 推力方向 → 姿态（力姿耦合） | FA on 改用 **新增** `thrustNedToBody()` |
| 姿态设定值发布 | 只用 `_control.getAttitudeSetpoint()` | FA on 优先 `generateFullActuatedAttitudeSetpoint()`，失败再回退 |
| 控制分配推力尺度 | `Fx/Fy/Fz` **各自**算 scale | **同一** `thrust_scale`（优先 Z） |
| 机架 / 模型 | （无此机） | **新增** 4026 + `fully_actuated_hexa` |

以下各节：标 **【替换】** 的是改写原逻辑；标 **【新增】** 的是新代码路径。

---

## 1. 【新增】`ControlMath.hpp` / `ControlMath.cpp` — `thrustNedToBody`

原只有 `thrustToAttitude()`（推力方向决定姿态）。全驱动旁路它，新增：

```cpp
bool thrustNedToBody(const matrix::Vector3f &thr_sp_ned,
		     const matrix::Quatf &q_current, const matrix::Quatf &q_desired,
		     vehicle_attitude_setpoint_s &att_sp);
```

```cpp
bool thrustNedToBody(const Vector3f &thr_sp_ned, const Quatf &q_current,
		     const Quatf &q_desired, vehicle_attitude_setpoint_s &att_sp)
{
	// ... 合法性检查 ...
	// 必须用 q_current 做坐标变换，不能用 q_desired
	const Vector3f thrust_body = q_current_normalized.rotateVectorInverse(thr_sp_ned);
	q_desired_normalized.copyTo(att_sp.q_d);
	thrust_body.copyTo(att_sp.thrust_body);
	return true;
}
```

---

## 2. 【替换入口 + 新增】`PositionControl`

### 【替换】`_velocityControl()` 分支

原：

```cpp
	_accelerationControl();
```

现：

```cpp
	if (_independent_thrust_control) {
		_accelerationControlIndependent();
	} else {
		_accelerationControl();
	}
```

### 【新增】开关与独立映射

```cpp
void setIndependentThrustControl(bool enabled) { _independent_thrust_control = enabled; }
void _accelerationControlIndependent();
bool _independent_thrust_control{false};
```

```cpp
void PositionControl::_accelerationControlIndependent()
{
	const float acceleration_to_thrust = _hover_thrust / CONSTANTS_ONE_G;
	_thr_sp.xy() = _acc_sp.xy() * acceleration_to_thrust;
	_thr_sp(2) = _acc_sp(2) * acceleration_to_thrust - _hover_thrust;
	_thr_sp(2) = math::min(_thr_sp(2), -_lim_thr_min);
}
```

`getAttitudeSetpoint()` 里仍可能调 `thrustToAttitude()`；FA 开启时外层 **不再用** 这条路径生成最终姿态设定值。

---

## 3. 【新增】`MulticopterPositionControl.hpp` 成员 / 参数 / 函数

```cpp
	const bool _vtol;
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	// ... hold / filter / mode ...
	(ParamInt<px4::params::MPC_FA_MODE>)         _param_mpc_fa_mode,
	(ParamInt<px4::params::MPC_FA_RC_AUX>)       _param_mpc_fa_rc_aux,
	(ParamFloat<px4::params::MPC_FA_TILT_MAX>)   _param_mpc_fa_tilt_max,
	bool generateFullActuatedAttitudeSetpoint(...);
	int32_t resolveFullActuatedMode(int32_t previous_mode) const;
```

---

## 4. 【新增 + 替换发布路径】`MulticopterPositionControl.cpp`

### 【新增】`generateFullActuatedAttitudeSetpoint()`

- mode 1：锁存进入时的 roll/pitch，yaw 跟位置设定；位置只靠三轴推力
- mode 2：摇杆 → roll/pitch（`MPC_FA_TILT_MAX` + `MC_MAN_TILT_TAU`），再合成 `q_desired`
- 调用 `ControlMath::thrustNedToBody(...)` 写 `att_sp`

### 【新增】`resolveFullActuatedMode()`

默认读 `MPC_FA_MODE`；`MPC_FA_RC_AUX=1..6` 时用 AUX 三段开关覆盖（带滞回）。

### 【替换】`Run()` 里姿态设定值发布

原：始终 `_control.getAttitudeSetpoint(attitude_setpoint)`。

现：

```cpp
			if (!full_actuated_requested
			    || !generateFullActuatedAttitudeSetpoint(local_pos_sp, dt, attitude_setpoint)) {
				_control.getAttitudeSetpoint(attitude_setpoint);  // 回退原路径
			}
```

同函数内还插入：姿态/摇杆订阅、模式解析、Pose 锁 XYZ、`setIndependentThrustControl(...)`、EKF reset 时修正 lock。

---

## 5. 【新增】参数

`multicopter_position_control_params.c`：

```c
PARAM_DEFINE_INT32(MPC_FA_MODE, 0);      // 0 标准 / 1 水平全驱动 / 2 Pose
PARAM_DEFINE_INT32(MPC_FA_RC_AUX, 0);    // 0 关；1..6 = AUX 覆盖
PARAM_DEFINE_FLOAT(MPC_FA_TILT_MAX, 15.f);
```

`multicopter_position_mode_params.c`（1.17 Pose 摇杆整形回补）：

```c
PARAM_DEFINE_FLOAT(MPC_HOLD_DZ, 0.1f);
PARAM_DEFINE_FLOAT(MPC_XY_MAN_EXPO, 0.6f);
```

---

## 6. 【替换】控制分配推力归一化

文件：`ControlAllocationPseudoInverse.cpp`

原：对 `THRUST_X/Y/Z` **分别**算 `_control_allocation_scale(3+axis)`，会扭曲 \(F_x:F_y:F_z\) 方向。

现：共用一个 `thrust_scale`（优先有非零列的 Z，再 Y/X）：

```cpp
	float thrust_scale = 1.f;
	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		// ... 对该轴 mix 列求平均范数 ...
		if (num_non_zero_thrust > 0) {
			thrust_scale = norm_sum / num_non_zero_thrust;
			break;
		}
	}
	_control_allocation_scale(THRUST_X) = thrust_scale;
	_control_allocation_scale(THRUST_Y) = thrust_scale;
	_control_allocation_scale(THRUST_Z) = thrust_scale;
```

配套【新增】：`getMixMatrix()` / `getControlAllocationScale()`，以及 `ControlAllocator::print_status` 打印 mix 与 scale。

---

## 7. 【新增】单元测试

- `ControlMathTest.cpp`：`thrustNedToBody` 水平 / 旋转 / 非法输入
- `PositionControlTest.cpp`：独立推力绕过 tilt cone、三轴直映
- `ControlAllocationPseudoInverseTest.cpp`：统一推力尺度保方向

---

## 8. 【新增】机架 `4026_gz_fully_actuated_hexa` + 模型

**仅此机架 / 模型**（无 typhoon_h480、无 hex600）。

默认要点：

```sh
PX4_SIM_MODEL=fully_actuated_hexa
CA_AIRFRAME 0
CA_ROTOR_COUNT 6
# CA_ROTOR*_PX/PY/PZ/KM/AX/AY/AZ：固定倾转六旋翼（α≈35°, β≈±25°），PX4 FRD
THR_MDL_FAC 1.0
CA_METHOD 0
MPC_THR_HOVER 0.32
MPC_FA_MODE 2
MPC_FA_TILT_MAX 10.0
MC_MAN_TILT_TAU 0.25
# 以及保守 XY 速度/加速度限制、偏航抑抖参数等
```

`airframes/CMakeLists.txt` 注册：`4026_gz_fully_actuated_hexa`。

Gazebo 模型目录：`Tools/simulation/gz/models/fully_actuated_hexa/`（`model.sdf` + meshes）。

---

## 9. 控制链路（改后）

```
位置/速度 PID → NED 推力 T_N
  ├─ FA off → _accelerationControl + thrustToAttitude（原耦合路径）
  └─ FA on  → _accelerationControlIndependent
              + thrustNedToBody(q_cur, q_des)
              → (q_d, thrust_body[3])
                  → att/rate → control_allocator（Fx/Fy/Fz 统一尺度）→ 电机
```

---

## 10. 编译 / 启动

```sh
make px4_sitl gz_fully_actuated_hexa

param show MPC_FA_MODE
param set MPC_FA_MODE 2   # 4026 默认；0=标准；1=锁姿态改位置
param set MPC_FA_RC_AUX 1 # 可选：AUX1 三段开关覆盖
```

---

## 11. 本目录文件清单（相对 PX4 根覆盖即可）

```
src/modules/mc_pos_control/...
src/lib/control_allocation/control_allocation/...
src/modules/control_allocator/...
ROMFS/px4fmu_common/init.d-posix/airframes/4026_gz_fully_actuated_hexa
ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
Tools/simulation/gz/models/fully_actuated_hexa/
```
