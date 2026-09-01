# 全矢量（全驱动）飞行器修改记录

> **移植状态（2026-07-28）**  
> 源分支：`fullycontrol`（基于 `v1.16.2`）  
> 目标分支：`fullycontrol-1.17.0`（基于稳定版 `v1.17.0` / `official/v1.17.0`）  
> 已并入：位置环解耦、`thrustNedToBody`、`MPC_FA_*`（含 `MPC_FA_RC_AUX`）、控制分配统一推力尺度、机架 4024/4025/4026、Gazebo `fully_actuated_hexa`。  
> 保留：`v1.17.0` 的 goto setpoint 与 trajectory_setpoint 互斥逻辑。  
> 1.17 适配：回补 `MPC_HOLD_DZ` / `MPC_XY_MAN_EXPO`（上游从 position mode params 删除，Pose 摇杆整形仍需要）。  
> 验证：`make px4_sitl_default` 通过。

---

## 1. 改动目标

标准多旋翼：水平力靠倾斜产生（`thrustToAttitude`）。
全驱动布局：效能矩阵满秩，水平力可由 \(F_x,F_y\) 直接产生。

本改动让位置环输出完整 NED 三维推力，姿态设定值独立给定，二者解耦；控制分配对 \(F_x,F_y,F_z\) 使用统一归一化尺度，避免推力方向被改变。

---

## 2. 修改文件清单

### 2.1 位置控制 `src/modules/mc_pos_control/`

#### `MulticopterPositionControl.hpp`

**新增成员 / 订阅 / 参数绑定**
- `_vtol`：构造时记下是否 VTOL（全驱动逻辑对 VTOL 禁用）
- `_vehicle_attitude_sub` / `_vehicle_attitude`：订阅当前姿态，供 NED→Body 变换
- `_manual_control_setpoint_sub` / `_manual_control_setpoint`：Pose 模式读摇杆
- `_full_actuated_mode`：当前生效模式 0/1（水平全驱动 / Pose）
- `_full_actuated_position_hold` + `_full_actuated_position_hold_valid`：Pose 锁定的 NED 位置
- `_full_actuated_tilt_filter` + `_full_actuated_tilt_filter_initialized`：Pose 倾角一阶滤波
- `_last_full_actuated_warn`：解耦失败告警限频
- 参数：`_param_mpc_fa_mode`、`_param_mpc_fa_tilt_max`、`_param_mpc_fa_rc_aux`

**新增函数声明**
- `generateFullActuatedAttitudeSetpoint(...)`：生成解耦姿态设定值
- `resolveFullActuatedMode(previous_mode)`：参数 + RC AUX 解析模式

#### `MulticopterPositionControl.cpp`

**新增函数**
1. `generateFullActuatedAttitudeSetpoint()`
   - mode 0：进入模式时锁存当前滚转角和俯仰角，偏航仍跟随位置设定值；位置变化仅由三轴独立推力实现
   - mode 1：摇杆 → roll/pitch（`MPC_FA_TILT_MAX` + `MC_MAN_TILT_TAU` 滤波），再与 yaw 合成 `q_desired`
   - 调用 `ControlMath::thrustNedToBody(thrust_ned, q_current, q_desired, att_sp)`
   - 写入 `yaw_sp_move_rate`；姿态超时/非法则返回 false

2. `resolveFullActuatedMode()`
   - 默认读 `MPC_FA_MODE`（仅 0/1）
   - 若 `MPC_FA_RC_AUX=1..6`，用对应 AUX 两段开关覆盖（带滞回）

**修改已有函数**
1. 构造函数：增加 `_vtol(vtol)` 初始化
2. `Run()`
   - 订阅并缓存 `vehicle_attitude`、`manual_control_setpoint`
   - 调用 `resolveFullActuatedMode()`，模式变化时清 lock / tilt 滤波
   - 位置控使能开关变化时同样清 lock / tilt 滤波
   - **Pose 锁位**：mode 2 且手动、已起飞时，把 `_full_actuated_position_hold` 写入 `_setpoint.position`，速度置 0，加速度置 NaN
   - `_control.setIndependentThrustControl(...)`：条件满足时打开三维推力直映
   - 发布姿态设定值处：优先 `generateFullActuatedAttitudeSetpoint()`；失败则回退 `_control.getAttitudeSetpoint()` 并限频告警
3. `adjustSetpointForEKFResets()`
   - 对 `_full_actuated_position_hold` 同步叠加 EKF 的 `delta_xy` / `delta_z`

#### `PositionControl/PositionControl.hpp`

**新增**
- 公有：`setIndependentThrustControl(bool enabled)` —— 开关三维推力直映
- 私有：`_accelerationControlIndependent()` 声明
- 私有成员：`_independent_thrust_control`（默认 false）

#### `PositionControl/PositionControl.cpp`

**新增函数**
- `_accelerationControlIndependent()`  
  \(T_{xy}=a_{xy}\cdot T_{hover}/g\)，\(T_z=a_z\cdot T_{hover}/g-T_{hover}\)，并限制 \(T_z\le -T_{min}\)；**不**走 tilt cone

**修改已有函数**
- `_velocityControl()`：若 `_independent_thrust_control` 为真，调用 `_accelerationControlIndependent()`，否则仍调原 `_accelerationControl()`

#### `PositionControl/ControlMath.hpp` / `ControlMath.cpp`

**新增函数**
- `thrustNedToBody(thr_sp_ned, q_current, q_desired, att_sp)`  
  （原名 `thrustToIndependentAttitude`，已改名）
  - 合法性检查 + 四元数归一化
  - `thrust_body = q_current.rotateVectorInverse(thr_sp_ned)`（NED→机体 FRD）
  - `att_sp.q_d = q_desired`，`att_sp.thrust_body = thrust_body`
  - 与现有 `thrustToAttitude()` 并列：后者把推力方向变成倾斜姿态；本函数推力与姿态解耦

#### `PositionControl/ControlMathTest.cpp`

**新增用例**
- `IndependentThrustAttitudeMappingLevel`：水平姿态下 body 推力等于 NED 推力
- `IndependentThrustAttitudeMappingRotated`：旋转姿态下 NED 可被重建
- `IndependentThrustAttitudeMappingRejectsInvalidInput`：非法四元数/NaN 推力返回 false

#### `PositionControl/PositionControlTest.cpp`

**新增用例**
- `IndependentThrustBypassesTiltCone`：独立推力不受 tilt limit 限制
- `IndependentThrustMapsAxesDirectly`：三轴加速度按悬停推力比例直映

#### `multicopter_position_control_params.c`

**新增参数定义**
- `MPC_FA_MODE`（int，默认 0）：0 水平全驱动 / 1 Pose
- `MPC_FA_TILT_MAX`（float，默认 15）：Pose 最大倾角
- `MPC_FA_RC_AUX`（int，默认 0）：AUX 通道覆盖模式

### 2.2 控制分配

| 文件 | 改动 |
|------|------|
| `ControlAllocationPseudoInverse.cpp` | \(F_x,F_y,F_z\) 共用一个 `thrust_scale`（优先取 Z 轴尺度） |
| `ControlAllocation.hpp` / `ControlAllocationPseudoInverse.hpp` | `getMixMatrix()`、`getControlAllocationScale()` |
| `ControlAllocationPseudoInverseTest.cpp` | 统一推力尺度相关测试 |
| `ControlAllocator.cpp` | `print_status` 打印 mix 矩阵与 allocation scale |

### 2.3 机架与仿真

| 文件 | 改动 |
|------|------|
| `ROMFS/.../airframes/4024_gz_hex600` | hex600 SITL 机架：6 电机倾斜推力轴 + `MPC_FA_MODE=0` |
| `ROMFS/.../airframes/4025_gz_typhoon_h480` | Typhoon 外观 + hex600 几何：`MPC_FA_MODE=1` |
| `ROMFS/.../airframes/CMakeLists.txt` | 注册上述机架 |
| `Tools/simulation/gz/models/fully_actuated_hexa/` | Gazebo 全驱动六旋翼模型（SDF / meshes） |

---

## 3. 位置控制核心改动

### 3.1 独立推力映射（`PositionControl`）

- 新增 `setIndependentThrustControl(bool)`。
- 开启时走 `_accelerationControlIndependent()`：
  - \(T_{xy} = a_{xy} \cdot T_{hover}/g\)
  - \(T_z = a_z \cdot T_{hover}/g - T_{hover}\)
- 不再把水平加速度压进 tilt cone。

### 3.2 NED → Body（`ControlMath::thrustNedToBody`）

原名 `thrustToIndependentAttitude`，已改名。

```
thrust_body = q_current.rotateVectorInverse(thr_sp_ned)   // NED → FRD
att_sp.q_d  = q_desired                                  // 姿态独立
att_sp.thrust_body = thrust_body
```

必须用 **当前姿态** `q_current` 做坐标变换，不能用 `q_desired`，否则姿态跟踪误差会重新引入力—姿态耦合。

### 3.3 外层模式（`MulticopterPositionControl`）

| `MPC_FA_MODE` | 行为 |
|---------------|------|
| 0 | 锁存并保持进入模式时的当前滚转角和俯仰角，偏航仍可控；三轴推力独立改变位置 |
| 1 | Pose：起飞后锁 XYZ；摇杆控姿态；油门不再改高度 |

配套逻辑：
- `generateFullActuatedAttitudeSetpoint()` 生成解耦输出
- 姿态超时 / 映射失败 → 回退标准 `getAttitudeSetpoint()`
- EKF xy/z reset 时同步修正 `_full_actuated_position_hold`
- 模式切换、控使能变化时重置 lock 与 tilt 滤波

### 3.4 新增参数

| 参数 | 默认 | 含义 |
|------|------|------|
| `MPC_FA_MODE` | 0 | 0=水平全驱动 / 1=Pose |
| `MPC_FA_TILT_MAX` | 15° | Pose 模式摇杆最大倾角 |
| `MPC_FA_RC_AUX` | 0 | AUX1–6 两段开关覆盖模式 |

`MPC_FA_RC_AUX` 映射（归一化 AUX）：
- `< -0.2` → mode 0（水平全驱动）
- `> 0.2` → mode 1（Pose）
- 中间带滞回保持上一模式

---

## 4. 控制分配改动

原先按轴分别算推力归一化尺度，会改变 \(F_x:F_y:F_z\) 比例，扭曲力方向。

现改为：对推力三轴使用**同一** `thrust_scale`（有 Z 通道优先用 Z，否则 Y/X）。力矩轴（roll/pitch/yaw）归一化逻辑不变。

---

## 5. 机架默认配置摘要

**4024 hex600**
- 6 电机固定倾转（α≈35°, β≈±25°）
- `CA_ROTOR*_AX/AY/AZ` 为 PX4 FRD 推力方向
- `MPC_FA_MODE=0`，`MPC_FA_TILT_MAX=15`

**4025 typhoon_h480**
- 外观 Typhoon，控制几何同全驱动 hex
- `MPC_FA_MODE=1`，`MPC_FA_TILT_MAX=10`
- 偏航相关：`MC_YAW_TQ_CUTOFF=0`，`MC_YAWRATE_K=0.30`（抑抖）

---

## 6. 控制链路（改后）

```
位置/速度 PID
    → NED 推力 T_N
        ├─ FA off → thrustToAttitude（倾斜耦合）
        └─ FA on  → Independent 加速度映射
                    + thrustNedToBody(q_cur, q_des)
                    → (q_d, thrust_body[3])
                        → 姿态/速率环
                        → 控制分配（Fx/Fy/Fz 统一尺度）
                        → 电机
```

---

## 7. 未改 / 复用部分

- 位置 PID 增益结构未改
- 姿态环、速率环消息接口未改（仍用 `vehicle_attitude_setpoint`）
- VTOL 位置控制忽略 `MPC_FA_*`（`!_vtol` 才启用）

---

## 8. 当前工作区相对提交的额外改动

相对 `b7ae1ae55b` 尚未提交：
1. 函数改名：`thrustToIndependentAttitude` → `thrustNedToBody`
2. 新增 `MPC_FA_RC_AUX` + `resolveFullActuatedMode()`（RC 两段开关覆盖模式）
3. 删除可选标准欠驱动：`MPC_FA_MODE` 仅 0/1 两种全驱动位置模式
