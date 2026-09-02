# PX4 v1.17 飞行汽车隔离融合设计

## 1. 目标

以当前全驱动 PX4 v1.17 源码为唯一基线，将另一套同版本源码中的飞行汽车能力整合进同一代码树。融合后通过机架参数选择构型：

- PX4 原生四旋翼继续使用原生多旋翼控制链；
- `6003` 使用全驱动六旋翼控制链；
- `6004` 使用全驱动六旋翼控制链和 HX-30HM 机械臂；
- `80003` 使用四旋翼与差速车组合的飞行汽车控制链。

本项目不是把六个全驱动旋翼与车轮组合成一个新载具。`6003`、`6004` 和 `80003` 是互斥机架，运行时只启用所选机架对应的功能。

## 2. 不变量

融合必须保持以下行为不变：

1. 普通四旋翼不启动飞行汽车模块，不响应飞行汽车形态切换。
2. `6003` 的全驱动控制器、控制分配矩阵、参数和 Motor1–6 含义不变。
3. `6004` 在 `6003` 基础上继续独立启动 HX-30HM 机械臂驱动。
4. 普通 Rover 继续使用其原始执行器编号和控制行为。
5. 只有 `80003` 将 Motor1–4 用作旋翼、Motor5–6 用作左右车轮。
6. 飞行汽车代码不能用动态变化的 `MAV_TYPE` 作为物理构型身份。
7. 已解锁、未落地或仍在运动时不得切换飞行与地面形态。

## 3. 构型识别

新增整数参数 `SYS_FC_TYPE`：

- `0`：非飞行汽车，默认值；
- `1`：四旋翼与差速车组合构型。

只有 `80003_flying_car` 设置 `SYS_FC_TYPE=1`。其他机架使用默认值 `0`。C++ 代码不得在多个模块中散布 `SYS_AUTOSTART == 80003` 判断；机架编号只负责在启动阶段选择功能参数。

`MAV_TYPE` 仍用于向外报告当前工作形态：飞行形态报告 Quadrotor，地面形态报告 Rover，但不用于判断固件是否属于飞行汽车。

## 4. 模块边界

新增独立的 `src/modules/flying_car/` 模块，负责飞行汽车特有行为：

- `FlyingCar`：模块生命周期、订阅、发布和参数更新；
- `FlyingCarModeManager`：形态状态机和切换条件；
- `FlyingCarDifferentialControl`：仅为 `80003` 生成 Motor5–6 差速输出；
- `FlyingCarActuatorGate`：保证飞行旋翼与车轮输出互斥。

运行时不能订阅后重发同一个 `actuator_motors` 实例，否则会形成自反馈，也不能依赖 `PublicationMulti` 获得固定实例。因此增加专用 `flying_car_actuator_motors` uORB 主题：`flying_car` 订阅原生 `actuator_motors` 和车轮设定值，经门控后发布专用主题；`FunctionMotors` 在收到首个专用主题样本后锁定该输入直到重启。普通构型从不发布专用主题，继续使用原生输入，行为不变。

以下模块不应包含飞行汽车专用执行器索引：

- 原生多旋翼位置、姿态和速率控制器；
- 全驱动位置、姿态和控制分配模块；
- 通用 `rover_differential`；
- 通用输出驱动。

Commander 只承担系统级安全职责：接收飞行汽车状态、在切换期间禁止解锁、更新对外报告的车辆类型。Commander 不直接停止或启动控制模块，也不直接写执行器输出。

## 5. 80003 控制链

### 5.1 飞行形态

`80003` 的飞行部分使用 PX4 原生四旋翼控制链，不使用全驱动控制器：

```text
flight_mode_manager
  -> mc_pos_control
  -> mc_att_control
  -> mc_rate_control
  -> control_allocator
  -> Motor1–4
```

飞行形态下 Motor5–6 必须处于可逆 ESC 的中立停止值。

原生控制分配器仍发布 Motor1–4 到 `actuator_motors`。飞行汽车模块将其门控为专用输出主题，物理输出层只消费门控后的样本。

### 5.2 地面形态

地面部分使用飞行汽车专用差速控制：

```text
地面位置/速度/转向设定值
  -> FlyingCarDifferentialControl
  -> actuator_motors[4], actuator_motors[5]
  -> Motor5, Motor6
```

地面形态下 Motor1–4 必须停转。普通 Rover 的 Motor1–2 输出不受影响。

飞行汽车专用差速控制不修改通用 `rover_differential`，而是直接订阅 `rover_throttle_setpoint` 和 `rover_steering_setpoint`，产生 Motor5–6。

### 5.3 物理输出

`80003` 保留原飞行汽车映射：

| 逻辑执行器 | 功能 | 目标物理输出 |
|---|---|---|
| Motor1–4 | 四个飞行电机 | AUX1–4，DShot600 |
| Motor5 | 左车轮 | AUX5，50 Hz 可逆 PWM |
| Motor6 | 右车轮 | AUX6，50 Hz 可逆 PWM |

`CA_R_REV=48` 表示 actuator motor 索引 4 和 5 可逆。文档和注释必须使用一致的零基索引或 Motor5/Motor6 表述，避免写成 Motor1/Motor2。

物理定时器分组以目标 Pixhawk 6X 板级定义为准。构建和台架测试必须确认 AUX1–4 与 AUX5–6 可以使用所配置的不同协议和频率。

## 6. 状态机

飞行汽车模块维护以下状态：

```cpp
enum class FlyingCarMode : uint8_t {
    Flight,
    TransitionToGround,
    Ground,
    TransitionToFlight,
    Fault
};
```

初始版本只支持未解锁状态切换，不支持空中变形或行驶中切换。允许切换必须同时满足：

- `SYS_FC_TYPE == 1`；
- 当前处于未解锁状态；
- `vehicle_land_detected.landed == true`；
- 本地水平速度低于 `FC_SW_VEL_MAX`；
- 执行器和目标控制链无已知故障；
- 切换请求稳定保持规定时间。

切换顺序为：阻止解锁、所有六路输出进入安全值、重置目标控制器状态、改变输出所有权、验证目标控制链、更新车辆类型、解除切换锁。任一步失败进入 `Fault`，六路输出保持安全值。

## 7. 输出所有权与安全值

| 飞行汽车状态 | Motor1–4 | Motor5–6 |
|---|---|---|
| Flight | 原生 MC 输出 | 中立停止 |
| TransitionToGround | 停止 | 中立停止 |
| Ground | 停止 | 差速车输出 |
| TransitionToFlight | 停止 | 中立停止 |
| Fault/未知 | 停止 | 中立停止 |

飞行汽车输出门控仅在 `SYS_FC_TYPE=1` 时启用。`SYS_FC_TYPE=0` 时不得订阅后重发或改写普通机架的执行器数据。

## 8. 参数和消息

飞行汽车专用参数使用 `FC_` 前缀，至少包括：

- `FC_MODE_CH`：形态切换 RC 通道；
- `FC_BOOT_MODE`：上电默认形态，首版只允许 Flight；
- `FC_SW_VEL_MAX`：允许切换的最大水平速度；
- `FC_SW_DELAY`：安全输出保持与消抖时间；
- `FC_WHEEL_TRACK`：左右轮距；
- `FC_WHEEL_SPD_MAX`：最大地面速度；
- `FC_WHEEL_THR_MAX`：车轮最大归一化输出；
- `FC_WHEEL_REV`：左右轮方向配置。

新增 `FlyingCarStatus.msg`，至少发布当前状态、请求状态、切换许可、拒绝原因、飞行链健康状态和地面链健康状态。切换请求不能通过改写 `MAV_TYPE` 实现；RC 或 MAVLink 请求必须转换为明确的内部请求。

## 9. 启动和板级配置

融合以下 `80003` 资源：

- 真机与 SITL airframe 文件；
- `rc.flying_car_defaults`、`rc.flying_car_apps` 和 SITL defaults；
- airframe 与 ROMFS CMake 注册；
- Gazebo 模型和启动目标；
- Pixhawk 6X 所需的 Commander、MC、控制分配、DShot、PWM 和飞行汽车模块构建选项。

`6003/6004` 的 Pixhawk 6C 配置不引入飞行汽车模块。普通默认板级配置可以编译飞行汽车模块，但它在 `SYS_FC_TYPE=0` 时必须立即拒绝启动或保持完全非活动。

## 10. 错误处理

- 不满足切换条件：保持原状态，发布具体拒绝原因。
- 目标控制链未就绪：进入 `Fault`，禁止解锁。
- 状态消息超时：Commander 禁止解锁，飞行汽车输出进入安全值。
- RC 请求抖动：使用迟滞和时间消抖，不连续切换。
- 参数无效：拒绝启动飞行汽车模块并记录错误，不回退到不确定映射。
- 输出源冲突：检测到非当前控制链仍发布有效值时，门控层覆盖为安全值并记录事件。

## 11. 测试策略

### 11.1 静态和构建验证

- 构建原生 SITL 四旋翼；
- 构建 `6003`、`6004` 对应目标；
- 构建 `80003` SITL；
- 构建 Pixhawk 6C 与 Pixhawk 6X 固件；
- 检查固件尺寸、未解析符号和模块依赖。

### 11.2 单元测试

- `SYS_FC_TYPE=0` 时飞行汽车逻辑完全不介入；
- 差速运动学正确写入索引 4、5；
- `CA_R_REV=48` 与 Motor5/6 对应；
- 各状态下输出所有权符合安全表；
- 已解锁、未落地、高速和故障状态下拒绝切换；
- 切换超时进入 `Fault`；
- 状态机恢复与重启行为确定。

### 11.3 回归验证

- 原生四旋翼的主要控制器测试通过；
- 全驱动数学、姿态和控制分配测试通过；
- 普通 Rover 测试通过；
- `6003/6004` 的参数默认值和 effectiveness matrix 与融合前一致。

### 11.4 SITL 和真机验证

SITL 必须实际驱动四个旋翼和两个轮子，而不是只修改 `MAV_TYPE`。真机依次进行去桨台架、轮子离地、无桨地面运行、系留飞行和完整任务验证，并保存 ULog、参数快照及测试记录。

## 12. 实施边界

首版不包含：

- 已解锁状态的形态切换；
- 空中自动变形；
- 地面行驶到起飞的自动任务编排；
- `6003/6004` 与车轮组合；
- 与当前指定 Pixhawk 6C/6X 无关的新飞控板适配。

这些能力只有在首版输出隔离和故障处理完成实机验证后才能单独设计。
