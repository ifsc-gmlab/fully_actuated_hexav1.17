# Pixhawk 6X 飞行汽车分阶段台架准入程序

## 1. 范围、状态与禁止事项

本文仅适用于 **Pixhawk 6X / FMUv6X**、机架 `80003`、`SYS_FC_TYPE=1` 的四旋翼加双差速轮构型。不得把本文套用于其他飞控板、6003、6004 或普通多旋翼。

当前状态：**NOT RUN**。仓库所在 Windows 主机没有受支持的 Linux PX4 构建工具链，尚未生成或刷写 `px4_fmu-v6x_default` 固件，尚未确认实板 AUX 定时器分组，也没有完成动力台架、实车或实飞验证。代码合入不等于硬件支持认证。

本文最多授权完成“动力无桨/车辆架空”阶段。低能地面和系留飞行只给出后续准入门，不因本文存在而获准执行。任何阶段失败都退回上一阶段；不得带缺陷继续。

## 2. 人员、场地与通用中止规则

- 至少两人：操作员负责 QGC/遥控器，安全员独立核对接线并始终控制机械急停或动力总开关。
- 拆除全部螺旋桨；车辆用非导电支架可靠架空，六个执行器均不能接触人员或台面。
- 设置物理急停和可直接拔除的动力电池连接；软件停止、USB 拔除或遥控器开关不能替代物理急停。
- 使用限流电源或带保险丝的动力支路。飞控/USB 与 ESC 动力分开供电，**禁止由飞控 5 V 或 AUX 接口给电机、ESC 动力级供电**。
- 任一电机非预期启动、轮输出不能回中、过热、异味、冒烟、异常电流、协议/电压不符、QGC 参数与记录不符，安全员立即断开动力，随后停止测试并保留日志。
- 不得在已解锁状态改变模式、参数、接线或 ESC 校准。DShot 电调不得按 PWM 方法校准。

## 3. 固件与证据前置门

在受支持的 Linux PX4 环境、干净工作树中执行并记录：

```sh
git rev-parse HEAD
make px4_fmu-v6x_default
sha256sum build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
git status --short
```

准入条件：构建退出码为 0；`git status --short` 仅含经批准的本地记录或为空；固件文件存在；提交完整哈希和 SHA-256 已填写到附录。任一条件不满足立即中止，**不得借用其他提交或其他板型的固件继续刷写**。

刷写前在 QGC 确认目标识别为 Pixhawk 6X/FMUv6X。刷写后记录版本信息和参数快照，再重启一次。若板型、提交哈希或固件哈希无法对应，立即中止。

## 4. 接线与供电

| AUX 通道 | 功能 | 信号协议 | 物理负载 |
|---|---:|---|---|
| AUX1..4 | Motor1..4 (`101..104`) | DShot600 | 四个旋翼 ESC，测试时无桨 |
| AUX5 | Motor5 (`105`) | 50 Hz reversible PWM | 左轮 ESC，1500 us 中位 |
| AUX6 | Motor6 (`106`) | 50 Hz reversible PWM | 右轮 ESC，1500 us 中位 |

信号地必须在飞控与所有 ESC 之间有单点公共参考，但动力回流不得经过飞控或 USB 地线。先断开动力电池完成信号线/地线检查；先给飞控/USB 供电并确认安全状态，再接 ESC 动力；结束时先断 ESC 动力，再断飞控/USB。若出现 USB 屏蔽层或地线承载动力回流、地电位差异常或接地环路电流，立即中止并修正供电拓扑。

首次接轮 ESC 前，按厂家文档在独立低风险夹具上确认双向模式、1500 us 中位、1100..1900 us 范围和失联回中/停机行为。若需要校准，只校准 AUX5/6 的 reversible PWM ESC；校准时不得连接旋翼 ESC 动力。中位漂移、死区不足或上电自走均为不通过。

## 5. 参数门

加载机架 `80003` 并重启，导出完整参数文件。逐项核对：

```text
SYS_AUTOSTART=80003
SYS_FC_TYPE=1
CA_R_REV=48
PWM_AUX_TIM0=-3
PWM_AUX_TIM1=50
PWM_AUX_FUNC1..6=101,102,103,104,105,106
PWM_AUX_DIS5=1500  PWM_AUX_MIN5=1100  PWM_AUX_MAX5=1900
PWM_AUX_DIS6=1500  PWM_AUX_MIN6=1100  PWM_AUX_MAX6=1900
```

飞行汽车参数必须按下表逐项记录。“代码默认值”来自 `src/modules/flying_car/module.yaml`；“本次批准值”是本文台架范围的判定基准，不等同于未来实车标定值。

| 参数 | 代码默认值 | 本次批准试验值/判定基准 |
|---|---:|---|
| `FC_MODE_CH` | `0`（禁用 RC 切换） | 现场确认未被其他功能占用的 RC AUX 编号 `1..6`；例如确认 RC AUX1 空闲后设 `1`。不能确认映射则 FAIL |
| `FC_BOOT_MODE` | `0` | `0`（Flight） |
| `FC_SW_VEL_MAX` | `0.2 m/s` | `0.2 m/s` |
| `FC_SW_DELAY` | `0.5 s` | `0.5 s` |
| `FC_WHEEL_TRACK` | `0.5 m` | 实测轮距，记录到 `0.01 m`；只有实测为 `0.50 m` 时才沿用默认值，未测量则 FAIL |
| `FC_WHEEL_SPD_MAX` | `2.0 m/s` | `0.2 m/s`，仅用于低能架空台架 |
| `FC_WHEEL_THR_MAX` | `0.5` | `0.05`，仅用于低能架空台架 |
| `FC_WHEEL_REV` | `0` | 根据架空方向检查选择 `0..3`，左右位定义必须与接线记录一致；未验证方向则 FAIL |

修改前导出并保存原值。设置 `FC_MODE_CH` 后，在 QGC RC 校准/通道监视页确认所选物理开关只驱动对应 `manual_control_setpoint.auxN`：值 `< -0.5` 请求 Flight，值 `> 0.5` 请求 Ground，`[-0.5, 0.5]` 保持上一个请求。若通道串扰、端点达不到阈值、方向与标签不符或被飞行模式等功能复用，立即中止并重新选择未占用 AUX；禁止猜测通道编号。

任何实际值不满足上表时保持动力断开，修正、重启、重新导出并复核。全部台架测试结束后恢复测试前参数快照、重启，再次导出并逐项确认恢复；禁止仅凭 QGC 页面缓存继续。

`CA_R_REV=48` 表示 Motor5/6 可逆，不表示轮方向已经正确；方向用 `FC_WHEEL_REV` 和 ESC 接线在架空测试中确认。Motor5/6 的归一化零值物理上应为 1500 us 中位。Motor1..4 的 `NaN`/停止命令物理上应由 DShot 发送停止/禁用语义，不应被解释为最小持续转速；必须用仪器和无桨电机观察共同确认。

## 6. 分阶段测试

每一行都要填写“实际结果/证据编号/操作员/安全员/日期”。未满足预期或触发中止条件时记 FAIL。

### A. 静态电气检查（允许执行）

| 操作 | 预期 | 立即中止条件 |
|---|---|---|
| 动力断开，逐线核对 AUX1..6、信号地、供电极性和保险丝 | 与第 4 节完全一致，无短路 | 任何通道错接、动力回流经过飞控、极性/绝缘不确定 |
| 测量飞控地与各 ESC 信号地 | 稳定公共参考，无异常压降 | 地电位差或环路电流异常 |
| 检查急停和动力插头可达性 | 安全员可在一秒内物理断电 | 急停失效、被运动部件遮挡 |

### B. 无动力信号检查（允许执行）

ESC 动力保持断开，用示波器/逻辑分析仪在 AUX1..6 对地测量。

| 操作 | 预期 | 立即中止条件 |
|---|---|---|
| 上电、保持未解锁 | AUX1..4 呈 DShot600 或安全静默；AUX5/6 为约 50 Hz、约 1500 us | AUX5/6 非中位，或任一协议出现在错误通道 |
| QGC Actuator Test 逐通道、低幅短时测试 | AUX1..4 每次仅对应一路 DShot 改变；AUX5/6 每次仅对应一路 PWM 改变 | 多通道联动、编号错位、停止命令仍产生推进命令 |
| 扫描 AUX5/6 的正负小指令 | 中位约 1500 us，端点受 1100..1900 us 限制且方向连续 | 越界、跳变、中位不稳定 |

必须记录实板实际定时器分组，并验证 DShot 的 AUX1..4 与 50 Hz PWM 的 AUX5..6 没有落入不可混用的同一定时器组。若板级映射不允许该协议组合，立即中止；不得通过改动未验证的定时器参数绕过。

### C. 动力无桨/车辆架空（本文允许的最高阶段）

先让 QGC Actuator Test 超时退出并确认未解锁，再由安全员接通限流动力。

| 操作 | 预期 | 立即中止条件 |
|---|---|---|
| 上电静置 | 旋翼不转，双轮保持中位不自走 | 任一执行器动作或电流异常 |
| QGC 逐测 Motor1..4 | 每次只有对应无桨电机低速转动，停止测试后立即停止 | 编号/方向错误、其他执行器动作、停止失败 |
| QGC 逐测 Motor5/6 | 对应车轮独立动作；零值/停止回到 1500 us；小正负指令方向相反 | 自走、不能回中、超出 1100..1900 us、旋翼联动 |
| Flight 稳态、未解锁观察 | 轮保持中位；输出与 `flying_car_status` 的 Flight 一致 | 轮动作、状态陈旧/缺失、输出不符 |
| Ground 稳态、未解锁观察 | 旋翼保持 DShot 停止；仅轮链允许命令 | 旋翼转动或模式/输出不一致 |
| 请求 Flight↔Ground 过渡 | 过渡期间旋翼停止、轮回中；稳定状态仅在延时和全部门满足后出现 | 过渡期间任何推进输出、超时未 Fault/拒绝 |
| 已解锁时请求切换 | 切换被拒绝，当前稳定模式保持，记录拒绝原因 | 已解锁仍进入过渡或改变输出所有权 |

#### C1. Ground 低能集成输出链

只有 A、B 和 C 前述项目全部 PASS，且仍满足“无桨、双轮架空、`FC_WHEEL_THR_MAX=0.05`、限流动力、双人、物理急停就绪”，才允许执行本项。先进入稳定 Ground，使用 `listener flying_car_status` 确认 `mode=MODE_GROUND`、链路 ready 且无拒绝；再执行 `listener vehicle_control_mode`，确认本版本实际字段 `flag_control_climb_rate_enabled: true`。记录 QGC 飞行模式名称、完整 `vehicle_control_mode` 输出和采用的解锁方法。若该标志为 false 或话题/字段无法确认，本项立即记 FAIL，不得解锁。

启动 ULog 和示波器记录后，把 throttle 保持在归一化中心 `0`，roll/steering 保持在中心 `0`。使用已经独立验证且未复用为模式选择的 Arm switch，或在 NSH 执行不带强制选项的 `commander arm`；严禁 `commander arm -f`。若中心零位不能正常通过预检并解锁，本项记 FAIL 并停止，不得把 throttle 拉到 `-1`/传统低位绕过。非 climb-rate 手动模式要求传统低油门解锁，该低位会成为反向轮指令，因此不允许用于本测试。不得发布伪造 uORB 消息，不得使用手工 PWM/DShot 命令。

1. throttle、roll/steering 保持零，解锁不超过 3 s；解锁瞬间先确认 Motor1..4 始终不动、双轮无动作、AUX5/6 均为约 1500 us。任一项偏离立即执行 `commander disarm`，同时由安全员物理急停，记 FAIL，禁止继续阶跃。
2. 给约 `+0.03` 归一化油门阶跃，持续不超过 1 s，然后回零至少 2 s；再给约 `-0.03`，同样不超过 1 s并回零。
3. 油门为零时给约 `+0.02`、`-0.02` 转向阶跃，各不超过 1 s并在其间回零至少 2 s。总解锁时间不得超过 15 s。
4. 当前 80003 不启动完整 Rover 链。用 `listener manual_control_setpoint` 确认 `valid=true`、`data_source=SOURCE_RC`、时间戳持续更新，前后杆只改变 `throttle`、左右杆只改变 `roll`；再对照 `flying_car_status`、`flying_car_actuator_motors`、最终可用的 `actuator_outputs` 与 AUX5/6 仪器波形，证明命令经过内部 Ground 输入选择与 `flying_car_actuator_motors → FunctionMotors → AUX5/6`。若现场另有明确发布的 Rover 两路设定值，必须确认两路同时新鲜且由同一控制源产生，它们会成对优先，禁止只注入一路。轮输出幅值不得超过 `0.05`，零值必须回到约 1500 us，旋翼必须始终无动作。
5. 每组阶跃后立即执行 `commander disarm`（或同一独立 Arm switch 切至 disarm），从 `vehicle_status` 确认已解除，再确认两轮回中和四旋翼停止，最后断 ESC 动力。

任一旋翼动作、错误车轮动作、输出超过 `0.05`/PWM 范围、回零超过 200 ms、话题链或仪器证据中断、模式离开稳定 Ground、电流超过现场批准限值，安全员立即物理断电；记 FAIL，不得重复加大指令排查。

### D. 故障注入（仍须无桨且架空）

故障注入只允许用下列可恢复方法；不得带动力拔插信号线、短接接口、伪造实板 uORB 或发送手工 PWM。除 D1 中专门说明的低能输出观察外均保持 disarmed；Commander 重启和参数错误测试还必须断开 ESC 动力。

#### D1. 模块停止、最终安全帧与 200 ms stale

- 前置：完成 C1，稳定 Ground、轮已回零；若需看动力响应，保持架空、限流、`FC_WHEEL_THR_MAX=0.05`，只施加不超过 `0.03`、1 s 的轮命令后立即回零并 disarm。
- 注入：在 NSH 执行 `flying_car stop`，不终止 mixer/output driver。
- 观察：保存控制台；连续监听 `flying_car_actuator_motors`，同时看 `actuator_outputs`（若记录可用）和 AUX 波形。停止瞬间应收到最终安全帧：Motor1..4 为 `NaN`/禁用，Motor5/6 为零，`reversible_flags=48`；超过 200 ms 没有新专用帧后，FunctionMotors 仍必须保持同一物理安全输出且不得回退原生 `actuator_motors`。
- 中止：任一旋翼动作、轮离开中位、旧推进值锁存或回退原生源，立即物理断电。
- 恢复：保持 disarmed，执行 `flying_car start`；确认 `flying_car status` 为运行、新鲜 `flying_car_status` 和安全专用帧恢复，随后才可进入下一项。

#### D2. status stale 与 Commander 重启

- 独立制造“仅 `flying_car_status` 陈旧而执行器源仍新鲜”需要开发插桩，只允许在 SITL/开发构建中完成；实板禁止伪造或拦截 uORB。实板以 D1 的 module stop 同时覆盖状态陈旧，并在超过 1 s 后尝试正常 arm：Commander 必须拒绝，控制台/事件和 `vehicle_status` 应显示未解锁；车辆类型报告保持上一个新鲜稳定类型。
- Commander 重启只在 disarmed 且 ESC 动力断开时执行：记录当前稳定模式，执行 `commander stop`，确认已停止，再执行 `commander start`。新鲜稳定 `flying_car_status` 到来前任何正常 arm 请求都必须被拒绝，RC 重解锁宽限和跳过可选预检也不得绕过。若命令不受该固件支持或 Commander 未能干净重启，立即执行整机 `reboot`，本项记 FAIL。
- 恢复：确认 `commander status`、`flying_car status` 正常，`listener flying_car_status` 连续更新且处于稳定模式，再运行完整预检；仍保持 disarmed，之后才可重新连接 ESC 动力。

#### D3. RC loss

- 前置：disarmed、稳定 Flight 或 Ground，输出已安全；优先断开 ESC 动力。记录 `input_rc`、`manual_control_setpoint`、`flying_car_status` 和 `vehicle_status`。
- 注入：关闭发射机或使用接收机厂家规定的失联操作，禁止拔信号线。等待系统配置的 RC loss 检测时间。
- 预期：RC/手动输入明确报告失联或无效；不发生未经授权的 Flight/Ground 请求，不能解锁，物理输出保持安全。
- 若测试从稳定 Ground 开始，断开 RC 后 `ground_chain_ready` 必须在一次有效性更新或最迟 500 ms 新鲜度窗口内变为 false，AUX5/6 回到约 1500 us；不得由最后一次摇杆值继续驱动车轮。恢复 RC 后先保持轮输入为零并重新完成模式/解锁检查。
- 中止：模式擅自切换、仍能解锁或任何推进输出。恢复时重新开启发射机，确认 RC valid、所选 AUX 的三段值和稳定状态恢复；保持 disarmed。

#### D4. 参数错误与恢复

- 前置：disarmed、ESC 动力物理断开，保存完整参数快照。选择可安全恢复的构型身份错误：记录原 `SYS_FC_TYPE=1`，设置 `SYS_FC_TYPE=0` 并 `reboot`；不得故意写错误 AUX function、PWM 端点或定时器并连接动力。
- 预期：`flying_car start` 拒绝以飞行汽车方式运行或系统保持非飞行汽车旁路；控制台与 `flying_car status` 提供可记录的失败/未运行证据。此状态绝不进入执行器测试。
- 恢复：设置 `SYS_FC_TYPE=1`，重新选择/加载 `SYS_AUTOSTART=80003` 默认值，恢复本节开头保存的全部参数快照并 `reboot`。重新导出参数，逐项通过第 5 节，确认 `flying_car status`、新鲜稳定状态和全部输出安全后，本项才算恢复成功。

#### D5. 两种稳定模式整机重启

分别在 Flight、Ground 稳态执行，始终 disarmed、无桨、架空且限流。重启前轮回零；执行 `reboot` 后预期启动期间旋翼停止、轮回中，只有新鲜稳定状态到来后才解除相应解锁锁定。上电脉冲、自走或接受陈旧状态均立即物理断电并记 FAIL。每次重启后完成参数、状态和输出复核才进入下一种模式。

## 7. 记录与 ULog

每次运行保存：固件 Git 完整哈希和 SHA-256、板序列号、参数快照、QGC 版本、接线全景及 AUX1..6 特写照片、示波器截图、ESC 型号/固件/校准记录、测试表和 ULog。

优先记录/检查：`flying_car_status`、`flying_car_actuator_motors`、`actuator_motors`、`actuator_outputs`、`vehicle_status`、`vehicle_control_mode`、`input_rc`。话题是否出现在 ULog 取决于 logger 配置、消息是否发布及目标构建；开始前用 `listener <topic>` 和 logger 配置确认真实可用性。缺失话题不得伪造为“零值”，应记为“不可用”，并用对应实时 listener、示波器或逻辑分析仪证据补足。`actuator_outputs` 也可能反映驱动映射后的最终值而非原始 DShot 帧，不能替代协议测量。

## 8. 后续阶段准入（本文不批准执行）

- **低能地面**：需 A–D 全部 PASS、缺陷关闭、独立评审批准；使用限速/限流、隔离场地和远程物理急停。首次不得安装螺旋桨。
- **系留飞行**：还需 Linux SITL/Gazebo 动态验证、FMUv6X 完整回归、低能地面报告、结构/动力/失效安全评审以及正式试飞计划。必须在适航场地、专业系留设施和独立安全负责人监督下进行。
- **非系留实飞或载人测试**：不在本项目和本文授权范围内。

## 9. 执行记录与签字模板

```text
测试编号：                         日期/时区：
Git 完整哈希：                    固件 SHA-256：
Pixhawk 6X 序列号：               QGC 版本：
旋翼 ESC/固件：                   轮 ESC/固件：
参数快照：                        ULog：
接线照片：                        示波器/逻辑分析仪证据：
实板 AUX 定时器分组：

阶段 A：PASS / FAIL / NOT RUN     备注：
阶段 B：PASS / FAIL / NOT RUN     备注：
阶段 C：PASS / FAIL / NOT RUN     备注：
阶段 D：PASS / FAIL / NOT RUN     备注：
遗留缺陷与处置：

操作员签字：                      日期：
安全员签字：                      日期：
独立评审人签字：                  日期：
最终结论：仅台架通过 / 失败 / 未运行（不得填写“实车支持”或“实飞支持”）
```
