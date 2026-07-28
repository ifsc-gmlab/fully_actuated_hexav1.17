# 全驱动 Position 控制与偏航稳定性修改说明

## 1. 修改目标

本修改面向固定倾转旋翼、具有六自由度控制能力的 `hex600`：

- 在 Position 控制启用时，位置变化不再必须依靠滚转/俯仰倾斜；
- 位置环的三轴推力与姿态设定值独立；
- 默认锁定进入 Position 控制瞬间的完整姿态；
- 可切换为锁定 XYZ 位置、使用手动摇杆改变姿态的 Pose 模式；
- 保留标准 PX4 映射，出现无效数据时自动回退；
- 针对 `2026-07-21/01_50_11.ulg` 中的偏航振荡使用更保守的默认参数。

这里的“不改变姿态”是指姿态**设定值**保持不变。实际姿态仍会存在姿态环跟踪误差、气动扰动和执行器饱和造成的小幅变化。

## 2. 原始问题

### 2.1 标准 PX4 为什么会倾斜

位置控制器内部已经计算了 NED 坐标系三轴归一化推力：

```text
位置/速度误差 -> acceleration setpoint -> thrust_NED = [Tx, Ty, Tz]
```

但是标准多旋翼输出在 `ControlMath::thrustToAttitude()` 中把该向量转换为姿态，并只保留机体 Z 轴推力：

```text
thrust_NED -> 倾斜后的 q_d
thrust_body = [0, 0, -|thrust_NED|]
```

这适合普通欠驱动多旋翼，却会浪费全驱动 hex600 已有的机体 X/Y 推力能力。旧日志也能看到：

- `vehicle_local_position_setpoint.thrust[0/1]` 有水平控制量；
- `vehicle_attitude_setpoint.thrust_body[0/1]` 被置零；
- 水平运动最终由滚转/俯仰变化实现。

### 2.2 偏航抖动判断

对日志 `build/px4_sitl_default/rootfs/log/2026-07-21/01_50_11.ulg` 的分析结果是：

- 偏航设定值基本恒定，摇杆偏航输入为零；
- 估计姿态与 Gazebo 真值一起振荡，不像磁罗盘或 EKF 单独跳变；
- 主振荡约为 `2.7~2.9 Hz`，振幅随时间增长，符合闭环稳定裕度不足；
- 当时 `MC_YAW_TQ_CUTOFF=2 Hz`，一阶低通在约 `2.8 Hz` 处引入约 `atan(2.8/2)=54.5°` 的附加相位滞后；
- 倾转旋翼几何的偏航控制效能较强，原偏航速率增益对该模型偏激进。

因此 airframe 默认值中关闭了该额外低通，并把偏航速率 PID 总体增益降至原来的 30%。这不是通过隐藏真实姿态来消除日志抖动，而是直接改善偏航闭环。

## 3. 控制结构修改

### 3.1 新的坐标变换

位置环继续输出 NED 推力 `T_N`，不修改原 PID、积分器、推力模长限制和倾角等效限制。新增映射使用实时估计姿态 `q_NB` 把推力转换到机体 FRD：

```text
T_B = R_NB(q_current)^T * T_N
```

其中：

- `T_N`：NED 坐标系推力；
- `T_B`：机体 FRD 坐标系推力；
- `q_current`：实时机体 FRD 到 NED 的姿态四元数；
- `q_desired`：独立生成的姿态设定值。

关键点是推力坐标变换必须使用 `q_current`，不能使用 `q_desired`。如果使用期望姿态，姿态存在跟踪误差时，实际 NED 推力方向会发生错误，力与姿态也会重新耦合。

新的输出为：

```text
vehicle_attitude_setpoint.q_d         = q_desired
vehicle_attitude_setpoint.thrust_body = T_B
```

现有下游链路已经支持三轴推力，无需修改 uORB 消息，但控制分配必须保持三轴推力的统一量纲：

```text
vehicle_attitude_setpoint
  -> mc_att_control
  -> vehicle_rates_setpoint（保留 thrust_body XYZ）
  -> mc_rate_control
  -> vehicle_thrust_setpoint（保留 XYZ）
  -> control_allocator（Fx, Fy, Fz, Mx, My, Mz）
```

### 3.2 三轴推力的控制分配归一化

PX4 原先会按各轴各自的控制裕度归一化伪逆矩阵。这会改变三维力向量的方向。2026-07-22 的 Typhoon H480 日志中，X/Y 推力的实际增益只有 Z 推力的约 `37.6%`；机体倾斜 10° 时，即使 NED 推力设定值保持竖直，执行器实现的水平补偿也只有约三分之一。

现在控制分配器对所有机型强制 `Fx = Fy = Fz` 的共用归一化尺度，不设置机型开关。有 Z 推力的飞行器优先使用 Z 轴尺度；无 Z 推力的机型自动回退到存在的 Y 或 X 轴，避免改变固定翼前向油门标度。这样可以保持三维推力设定值的方向，同时不改变普通多旋翼原有的 Z 轴标度。共用尺度可能使弱轴混控系数大于 1，这是对真实控制裕度的正确反映；激烈指令仍需通过分配残差和电机上下限判断可行性。

### 3.3 `MPC_FA_MODE`

新增参数 `MPC_FA_MODE`，普通 PX4 默认值仍为 `0`。

| 值 | 行为 | 适用场景 |
|---:|---|---|
| 0 | 标准 PX4：水平推力转换为倾斜姿态 | 普通欠驱动多旋翼、回退验证 |
| 1 | 三轴推力独立；滚转/俯仰设为 0；保留正常偏航设定值和偏航速率前馈 | 要求机体保持水平，但仍需用偏航摇杆转向 |
| 2 | 起飞后锁定当前 NED XYZ；滚转、俯仰、偏航摇杆改为姿态指令 | 位置不变而姿态可独立改变的手动 Pose 模式 |

hex600 默认使用模式 1。

模式 2 只在手动 Position 控制并确认飞行器已经进入飞行状态后生效：

- 进入时捕获当前 NED XYZ，位置环持续保持该点；
- 滚转摇杆控制正/负滚转角；
- 俯仰摇杆向前对应负俯仰角（机头向下）；
- 偏航摇杆沿用 PX4 Position 模式的航向积分和速率限制；
- 油门摇杆在 Pose 模式生效后不改变高度；
- 起飞、接地或非手动控制阶段自动采用标准 Position 行为。

这样可以先使用正常油门起飞，再把 `MPC_FA_MODE` 切到 2。不要在需要继续爬升/下降时启用模式 2；着陆前必须先切回模式 1 或 0，否则油门下降指令不会解除 XYZ 锁定。

### 3.4 安全措施

本修改包含以下保护：

1. `MPC_FA_MODE=0` 时完全执行原 PX4 代码路径。
2. VTOL 调用 `mc_pos_control vtol` 时忽略全驱动模式。
3. 实时姿态超过 `100 ms` 未更新时回退标准映射。
4. 当前/期望四元数非法、模长接近零或推力含 NaN 时回退标准映射。
5. 回退时以 2 秒间隔输出：

   ```text
   full-actuated output invalid, using standard mapping
   ```

6. 模式 2 捕获的 NED 位置会跟随 EKF XY/Z reset 修正，避免位置设定值阶跃。
7. 模式 2 的滚转/俯仰合成角受 `MPC_FA_TILT_MAX` 限制，并使用 `MC_MAN_TILT_TAU` 平滑。
8. 模式 2 的手动输入无效或超过 500 ms 未更新时回退标准映射。
9. 估计器四元数重置时重新初始化倾斜滤波器，避免姿态指令阶跃。
10. 保留原位置控制器的推力模长限制、水平积分抗饱和和 takeoff ramp。

需要注意：回退标准映射时飞行器可能重新通过倾斜产生水平力。这是有意的安全选择，因为姿态数据不可用时继续进行错误的 NED/FRD 变换风险更高。

## 4. 修改的文件

### `src/modules/mc_pos_control/PositionControl/ControlMath.hpp/.cpp`

新增：

```cpp
bool thrustToIndependentAttitude(
    const matrix::Vector3f &thr_sp_ned,
    const matrix::Quatf &q_current,
    const matrix::Quatf &q_desired,
    vehicle_attitude_setpoint_s &att_sp);
```

功能包括输入检查、四元数归一化、NED 到 FRD 推力转换和独立姿态输出。

### `src/modules/mc_pos_control/MulticopterPositionControl.hpp/.cpp`

主要变化：

- 订阅并缓存 `vehicle_attitude`；
- 捕获模式 2 的 NED 位置；
- 根据 `MPC_FA_MODE` 选择标准或全驱动输出；
- 实现 100 ms 超时和标准映射回退；
- VTOL 保持原行为。

### `src/modules/mc_pos_control/multicopter_position_control_params.c`

定义 `MPC_FA_MODE` 及参数元数据。

### `src/lib/control_allocation` 与 `src/modules/control_allocator`

将所有机型的 X/Y/Z 推力统一为同一归一化尺度，并增加单元测试确认不同轴物理控制效能下的推力方向保持不变。

### `src/modules/mc_pos_control/PositionControl/ControlMathTest.cpp`

新增测试覆盖：

- 水平机体下三轴推力直接映射；
- 任意滚转/俯仰/偏航下的 NED→FRD→NED 往返一致性；
- 坐标旋转前后推力模长不变；
- 期望姿态与推力方向相互独立；
- NaN 和零四元数被拒绝。

### `ROMFS/px4fmu_common/init.d-posix/airframes/4024_gz_hex600`

为 hex600 增加全驱动模式、保守运动限制和偏航稳定性默认值。

## 5. Hex600 默认参数

| 参数　　　　　　　 | 新默认值 | 原因　　　　　　　　　　　　　　　　　　　　　　　 |
| --------------------| ---------:| ----------------------------------------------------|
| `MPC_FA_MODE`　　　| 1　　　　| 保持水平，通过 Fx/Fy 实现水平位置修正　　　　　　　 |
| `MPC_FA_TILT_MAX`　| 15°　　　| 限制模式 2 的滚转/俯仰合成角，保留位置保持推力裕度 |
| `THR_MDL_FAC` | 1.0 | 线性化 Gazebo 的转速平方推力模型 |
| `MC_MAN_TILT_TAU`　| 0.25 s　 | 平滑模式 2 手动姿态指令　　　　　　　　　　　　　　|
| `MPC_XY_VEL_MAX`　 | 3.0 m/s　| 限制所有位置控制模式的最大水平速度　　　　　　　　 |
| `MPC_VEL_MANUAL`　 | 2.0 m/s　| 首次手动 Position 测试使用较低速度　　　　　　　　 |
| `MPC_ACC_HOR`　　　| 2.0 m/s² | 限制自动/加速度式手动控制的水平加速度　　　　　　　|
| `MPC_ACC_HOR_MAX`　| 2.0 m/s² | 限制 Position 模式水平加速度　　　　　　　　　　　 |
| `MPC_JERK_MAX`　　 | 4.0 m/s³ | 避免横向推力突变　　　　　　　　　　　　　　　　　 |
| `MPC_TILTMAX_AIR`　| 20°　　　| 在全驱动模式中仍作为推力向量锥角限制　　　　　　　 |
| `MC_YAW_TQ_CUTOFF` | 0 Hz　　 | 移除 2 Hz 低通在振荡频段引入的相位滞后　　　　　　 |
| `MC_YAWRATE_K`　　 | 0.30　　 | 把偏航速率 P/I/D 总体作用缩放到 30%　　　　　　　　|

虽然全驱动模式不再用 `MPC_TILTMAX_AIR` 产生姿态倾角，位置控制器仍用它限制 NED 推力向量的水平/垂直比例。因此这里保留该参数作为第一阶段的横向推力安全限制。

当前六旋翼控制效能矩阵按 airframe 配置计算为满秩 6，奇异值条件数约为 10.2，具备六自由度控制能力，但不同轴的控制裕度并不相同。初始参数刻意偏保守，不能因为矩阵满秩就直接使用很大的水平加速度。

## 6. 编译和测试

主 SITL 构建：

```sh
CCACHE_DIR=/tmp/px4_ccache \
CCACHE_TEMPDIR=/tmp/px4_ccache_tmp \
cmake --build build/px4_sitl_default -j2
```

本次修改已经通过参数生成、`ControlMath`、`mc_pos_control` 编译和最终 `bin/px4` 链接。

目标单元测试：

```sh
CCACHE_DIR=/tmp/px4_ccache \
CCACHE_TEMPDIR=/tmp/px4_ccache_tmp \
cmake --build build/px4_sitl_test --target unit-ControlMath -j2

build/px4_sitl_test/unit-ControlMath
```

结果为 `12/12` 通过，其中 3 项为本次新增的全驱动映射测试。

## 7. 启动和参数生效

启动 hex600 SITL：

```sh
make px4_sitl gz_hex600
```

`param set-default` 只定义 airframe 默认值。如果旧 SITL 参数文件已保存同名参数，旧值可能覆盖新的默认值。首次验证时在 PX4 shell 检查：

```sh
param show MPC_FA_MODE
param show THR_MDL_FAC
param show MC_YAW_TQ_CUTOFF
param show MC_YAWRATE_K
```

三轴推力共用尺度已固化在控制分配代码中，无需设置额外参数。至少确认 `THR_MDL_FAC=1`。若旧 SITL 参数文件覆盖了 airframe 新默认值，可以显式设置并保存：

```sh
param set MPC_FA_MODE 1
param set THR_MDL_FAC 1
param set MC_YAW_TQ_CUTOFF 0
param set MC_YAWRATE_K 0.30
param save
```

## 8. SITL 验证步骤

建议按以下顺序测试，不要一开始就给大幅水平指令。

1. 起飞至 1.5~2 m，保持悬停至少 20 秒。
2. 记录进入 Position 后的滚转、俯仰和偏航设定值。
3. 先给前向小输入，再给侧向小输入，每次保持 2~3 秒。
4. 确认机体位置改变，但 `q_d` 没有随水平推力方向倾斜。
5. 观察电机输出是否长期达到 0 或 1；如有持续饱和，降低 `MPC_ACC_HOR_MAX`、速度或位置环增益。
6. 悬停 30~60 秒，确认偏航振荡不再增长。

PX4 shell 可观察：

```sh
listener vehicle_local_position_setpoint
listener vehicle_attitude_setpoint
listener vehicle_thrust_setpoint
listener control_allocator_status
```

模式 1 的预期现象：

- `vehicle_local_position_setpoint.thrust[0/1]` 在水平移动时非零；
- `vehicle_attitude_setpoint.thrust_body[0/1]` 通常非零；
- `vehicle_attitude_setpoint.q_d` 保持水平（滚转/俯仰为 0），偏航可按设定值变化；
- `vehicle_thrust_setpoint.xyz[0/1]` 保留横向推力；
- `control_allocator_status.unallocated_thrust` 不应长期偏大。
- `control_allocator status` 打印的 `Tx Ty Tz` allocation scale 应三者相等。

模式 2 的验证方法：

```sh
param set MPC_FA_MODE 2
```

确认飞行器已经悬停后，分别给小幅滚转、俯仰和偏航输入。预期：

- `vehicle_attitude_setpoint.q_d` 随摇杆平滑改变；
- `trajectory_setpoint` 即使因滚俯摇杆产生移动意图，位置控制内部输出仍锁定模式 2 捕获的 XYZ；
- `vehicle_local_position` 保持在捕获点附近；
- 倾斜时 `vehicle_attitude_setpoint.thrust_body[0/1]` 会主动补偿重力和位置误差；
- 油门输入不改变锁定高度；退出模式 2 后恢复标准行为。
- 着陆前先把 `MPC_FA_MODE` 切回 1 或 0，再使用正常下降操作。

NED 与机体 FRD 是不同坐标系，飞行器存在非零姿态/航向时，不应直接逐元素比较两条消息中的推力；应按实时四元数旋转后比较。

## 9. ULog 验收标准

新日志至少检查以下项目：

- `vehicle_attitude_setpoint.q_d`：水平移动阶段无由位置指令引起的滚转/俯仰变化；
- `vehicle_attitude.q`：实际姿态跟踪稳定，无持续发散；
- `vehicle_local_position_setpoint.thrust`：NED 水平推力存在；
- `vehicle_attitude_setpoint.thrust_body`：FRD 三轴推力存在；
- `vehicle_thrust_setpoint.xyz`：下游仍保留三轴推力；
- `control_allocator_status.unallocated_thrust/torque`：无长期大残差；
- `vehicle_angular_velocity.xyz[2]` 与偏航角：不再出现 2.7~2.9 Hz 振幅持续增长；
- `actuator_motors.control`：无多个电机长期贴近上下限。

建议第一阶段接受标准：

- 小幅水平阶跃时滚转/俯仰设定值变化小于 0.5°；
- 悬停 30 秒内偏航振荡不增长；
- 横向移动时无连续超过 0.5 秒的执行器饱和；
- 位置误差收敛且 `unallocated_thrust` 不持续累积。

## 10. 调参与回退

### 快速回退到标准 PX4

无需撤销代码，直接执行：

```sh
param set MPC_FA_MODE 0
```

飞行器会恢复“水平位置变化依靠倾斜姿态”的标准行为。

### 保持水平但允许偏航

```sh
param set MPC_FA_MODE 1
```

### 保持位置并用摇杆改变姿态

飞行器正常起飞并稳定悬停后执行：

```sh
param set MPC_FA_MODE 2
```

模式 2 最大滚转/俯仰合成角由以下参数控制：

```sh
param set MPC_FA_TILT_MAX 15
```

首次实机测试建议从 `5~10°` 开始，不要直接提高到参数上限。

### 建议调参顺序

1. 先保持当前保守速度、加速度和 jerk 限制。
2. 单独确认偏航速率环稳定，再提高 `MC_YAWRATE_K`，每次不超过约 10%。
3. 再检查姿态环，不要同时改变位置和姿态增益。
4. 最后逐步提高 `MPC_ACC_HOR_MAX`，每次检查分配残差和电机饱和。
5. 如果横向控制方向错误，优先检查 `CA_ROTOR*_AX/AY/AZ` 的 FRD 符号，不要用反向位置增益补偿几何错误。

## 11. 当前限制

- 位置 PID 的抗饱和依据总推力球形限制，并不知道六个单向旋翼构成的真实六维可行力/力矩多面体；激烈机动时仍可能发生分配饱和。
- 伪逆分配器在执行器裁剪后可能同时损失力和力矩精度，因此必须检查 `unallocated_thrust/torque`。
- 模式 2 锁定的是进入飞行后的估计位置，不是外部绝对参考；估计误差仍会反映到实际位置保持。
- 本修改没有改变姿态控制器和速率控制器。偏航参数只是针对给定日志与当前 Gazebo 模型的保守初值，实机仍需重新辨识和逐步调参。
