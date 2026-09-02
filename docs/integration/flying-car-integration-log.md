# 飞行汽车融合日志

本日志持续记录以全驱动 PX4 v1.17 为基线整合 `80003` 飞行汽车的变动。每次代码修改必须记录文件、目的、验证结果和遗留风险。

## 2026-09-01：建立融合设计基线

### 已完成

- 确认源码实际根目录为二层同名目录。
- 确认当前全驱动源码和飞行汽车参考源码都不是可识别的 Git 工作树，无法使用共同祖先、提交记录或 cherry-pick 还原功能改动。
- 确认融合目标是同一代码树中的多个互斥机架，而不是六旋翼与车轮的组合构型。
- 固定机架边界：原生四旋翼保持不变，`6003` 为全驱动六旋翼，`6004` 为全驱动六旋翼加机械臂，`80003` 为四旋翼加差速车。
- 记录全驱动真机目标为 Pixhawk 6C；机械臂通过 TELEM2、BusLinker 和 HX-30HM 驱动。
- 记录飞行汽车参考实现目标为 Pixhawk 6X；Motor1–4 为 DShot 旋翼，Motor5–6 为 50 Hz 可逆 PWM 车轮。
- 发现参考飞行汽车实现将模式切换直接放入 Commander，并且 POSIX/SITL 仅修改 `MAV_TYPE`，未验证完整控制链切换。
- 发现参考实现将左右车轮写入 `actuator_motors[4]` 和 `[5]`，与 `CA_R_REV=48` 一致，但原注释存在电机编号表述歧义。
- 新建设计文档 `docs/superpowers/specs/2026-09-01-flying-car-isolated-integration-design.md`。

### 设计决定

- 使用 `SYS_FC_TYPE` 作为飞行汽车物理构型标识，避免使用会动态变化的 `MAV_TYPE`。
- `80003` 保留 Motor1–4 旋翼、Motor5–6 车轮布局。
- 新建隔离的 `flying_car` 模块，不把完整模式管理和执行器路由堆入 Commander。
- 不修改通用四旋翼控制器、全驱动控制器和普通 Rover 的执行器语义。
- 初版只允许未解锁、已落地、低速时切换。

### 验证状态

- 已完成源码结构和关键文件的只读检查。
- 尚未修改产品代码。
- 尚未运行构建或测试。

### 遗留风险

- 两个源码树没有 Git 历史，必须通过结构化差分人工识别真实功能改动。
- 需要核实 Pixhawk 6X AUX 定时器分组是否支持 AUX1–4 DShot600 与 AUX5–6 50 Hz PWM 同时运行。
- 需要确认参考实现的 Gazebo 模型是否真实包含轮子动力学与六路执行器，而非仅启动脚本。
- 需要确定飞行汽车状态消息与 Commander 解锁接口的最小侵入方式。

## 2026-09-01：形成逐文件实施计划

### 已完成

- 检查全驱动基线的 Rover、全驱动单元测试、消息注册、模块 Kconfig 和 FMUv6X 构建配置。
- 确认参考飞行汽车产品逻辑的主要侵入点为 `Commander.cpp` 和 `RoverDifferential.cpp`；计划拒绝直接复制这两处改动。
- 确定由独立 `src/modules/flying_car/` 模块负责状态机、Motor5/6 差速控制和执行器门控。
- 建立十个可独立验证的实施任务，覆盖基线、消息与参数、状态机、执行器、运行模块、机架、Commander、安全仿真、回归和真机台架文档。
- 新建实施计划 `docs/superpowers/plans/2026-09-01-flying-car-isolated-integration.md`。

### 验证状态

- 实施计划已映射设计文档的构型隔离、控制链、输出所有权、错误处理和测试要求。
- 尚未修改产品代码。
- 尚未初始化 Git；这被列为实施任务 1 的前置交付物。

## 后续记录模板

### YYYY-MM-DD：变动主题

#### 修改文件

- `path/to/file`：具体修改及理由。

#### 验证

- 命令：`具体命令`
- 结果：通过或失败，以及关键输出。

#### 风险与后续

- 尚未解决的问题及下一步。

## 2026-09-01：建立可恢复基线与参考差分清单

### 修改文件

- `docs/integration/flying-car-reference-manifest.md`：记录只读参考树中飞行汽车相关资源的 `copy`、`rewrite` 或 `reject` 处置；明确排除生成内容、子模块、翻译、行尾差异和无关版本漂移。
- `docs/integration/flying-car-integration-log.md`：记录本次基线与清单工作。

### 验证

- 命令：`git add -A`，`git commit -m "chore: snapshot fully actuated PX4 v1.17 baseline"`
- 结果：创建未修改产品代码的全驱动 PX4 v1.17 初始提交。
- 命令：`rg -l 'Flying Car|flying_car|actuator_motors.control\[4\]' D:\flying_car\PX4-Autopilot\PX4-Autopilot\src D:\flying_car\PX4-Autopilot\PX4-Autopilot\ROMFS --glob '!**/mavlink/**'`
- 结果：所有飞行汽车产品相关命中均已在差分清单中处置；Commander 和通用 RoverDifferential 均为拒绝直接复制。
- 命令：`git diff --no-index D:/flying_car/fully_actuated_hexav1.17-fullycontrol-1.17.0/fully_actuated_hexav1.17-fullycontrol-1.17.0 D:/flying_car/PX4-Autopilot/PX4-Autopilot`，并按 ROMFS、`msg`、`src/modules`、`boards/px4` 和仿真资源进行针对性比较。
- 结果：确认参考树存在大量无关版本差异；仅将飞行汽车功能映射为后续隔离任务。
- 命令：`git status --short`、`git log --oneline --decorate -2`、`git diff --check`。
- 结果：完成提交 `673cee3`（`docs: map flying car reference changes`）后的自审；工作树干净且未发现空白错误。

### 风险与后续

- 参考 `80003` POSIX 启动路径没有真实的六执行器 Gazebo 模型，不能作为飞行/车轮切换验证依据。
- FMUv6X 的混合 DShot/PWM 定时器配置仍需构建和无桨台架验证。

## 2026-09-01：定义飞行汽车配置、参数和状态消息

### 修改文件

- `msg/FlyingCarStatus.msg`：定义飞行汽车模式、拒绝原因和控制链就绪状态的 uORB 契约。
- `msg/CMakeLists.txt`：注册 `FlyingCarStatus.msg`，使 uORB 生成流程包含该消息。
- `src/modules/flying_car/FlyingCarTypes.hpp`：提供与消息契约一致的强类型模式与拒绝原因枚举。
- `src/modules/flying_car/module.yaml`：飞行汽车身份、安全切换与轮式控制参数的唯一权威定义，包含默认值和边界。
- `src/modules/flying_car/FlyingCarModeManagerTest.cpp`：固定模式和拒绝原因的枚举契约。

### 验证

- 先确认 `uORB/topics/flying_car_status.h` 不存在；在 `FlyingCarTypes.hpp` 创建前添加枚举一致性测试，并记录头文件缺失的结构化 RED。
- 使用 `rg` 和参数元数据解析检查消息常量、字段、参数默认值与消息注册。
- 运行 `git diff --check` 检查空白错误。
- 本机没有可用 WSL 发行版，无法运行原生 PX4 `make px4_sitl_default` 或 `make metadata_parameters`；未声称构建通过。

### 风险与后续

- 仍需在具备 Linux/PX4 构建环境的主机上生成 uORB 头文件、参数元数据并编译验证。

### 配置自审修正

- `module.yaml` 是飞行汽车九个参数的唯一权威定义；已删除重复的 `flying_car_params.c`，避免 PX4 参数元数据生成出现重复参数。
- `FlyingCarModeManagerTest.cpp` 现在包含 `<uORB/topics/flying_car_status.h>`，并将两个 C++ 枚举逐项直接比较到 `flying_car_status_s` 的消息常量，以捕获消息与类型契约漂移。
- 命令：`Test-Path uORB/topics/flying_car_status.h`、`rg -n '#include <uORB/topics/flying_car_status\\.h>|flying_car_status_s::(MODE_|REJECTION_)' src/modules/flying_car/FlyingCarModeManagerTest.cpp`。
- 结果：生成头文件仍不存在，已记录结构化 RED；本机无 WSL 发行版，未声称原生 PX4 编译通过。
- `.superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-2-report.md` 已从 Git 索引移除但保留本地，并由 `.superpowers/sdd/.gitignore` 忽略。
- 提交：`94b4fbf`（`fix: align flying car configuration metadata`）删除重复的 C 参数源，并将枚举测试直接绑定到 `flying_car_status_s` 消息常量。
- 自审命令：`git diff HEAD --check`、`git status --short`、`git show --check --stat --oneline HEAD`；结果：无空白错误、工作树干净、修正提交仅包含预期的元数据/测试/记录变动。

## 2026-09-01：实现纯飞行汽车安全状态机

### 修改文件

- `src/modules/flying_car/FlyingCarModeManager.hpp`：定义纯状态机输入、结果、微秒/米每秒单位契约和显式 Fault 复位接口；保持 Task 2 的 `FlyingCarRejectionReason`，并通过别名提供计划要求的 `FlyingCarRejection` 名称。
- `src/modules/flying_car/FlyingCarModeManager.cpp`：实现 Flight、双向过渡、Ground 和锁存 Fault；对构型、解锁、落地、速度、目标控制链、消抖、过渡保持和超时进行确定性判定。
- `src/modules/flying_car/FlyingCarModeManagerTest.cpp`：在产品实现前增加逐行为 GoogleTest，覆盖全部安全门、双向过渡、输出互斥、超时和 Fault 复位。
- `src/modules/flying_car/CMakeLists.txt`：按本地 PX4 约定注册纯库和 `px4_add_unit_gtest`，未创建运行时模块。
- `src/modules/flying_car/Kconfig`：注册飞行汽车安全原语配置项，并注明运行时模块由后续任务提供。
- `docs/integration/flying-car-integration-log.md`：记录 Task 3 的 TDD 证据、限制和风险。

### 验证

- RED 命令：`D:/Down/msys2/ucrt64/bin/g++.exe -std=c++17 -Wall -Wextra -Werror -pedantic -I. .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-3-harness.cpp -o .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-3-harness.exe`。
- RED 结果：失败，`FlyingCarModeManager.hpp: No such file or directory`，证明测试在产品接口不存在时捕获缺失功能。
- GREEN 命令：`D:/Down/msys2/ucrt64/bin/g++.exe -std=c++17 -Wall -Wextra -Werror -pedantic -I. .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-3-harness.cpp src/modules/flying_car/FlyingCarModeManager.cpp -o .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-3-harness.exe`，随后运行该可执行文件。
- GREEN 结果：通过，共 20 项行为检查。
- 静态检查：检查 GoogleTest 行为用例、`px4_add_library`/`px4_add_unit_gtest` 注册和禁用依赖词；检查 `git diff --check`。
- 构建限制：本机没有可用的 Linux/WSL PX4 工具链，生成的 uORB 头文件也不存在，因此未运行或声称 PX4 Linux/SITL 构建通过。

### 风险与后续

- 需在 Linux PX4 环境中生成 `uORB/topics/flying_car_status.h`，编译并运行 `FlyingCarModeManagerTest`，同时确认 Kconfig 选中后的库链接。
- 当前纯状态机只声明输出所有权；控制器重置、车辆类型发布和真实执行器安全值由后续运行模块及门控任务实现。
- `FC_SW_DELAY` 同时作为请求消抖和过渡安全输出保持时间；后续参数集成若拆分时序，必须保持现有安全下限和测试覆盖。

## 2026-09-01：修正活动过渡的连续安全门

### 修改文件

- `src/modules/flying_car/FlyingCarModeManager.cpp`：活动过渡现在持续检查目标请求、构型、解锁、落地、速度和目标控制链；任一条件失效立即回退到来源稳定形态并清除过渡/消抖计时。
- `src/modules/flying_car/FlyingCarModeManager.hpp`：增加无分配的内部过渡取消辅助函数。
- `src/modules/flying_car/FlyingCarModeManagerTest.cpp`：增加双向请求反转、活动过渡期间各安全门失效、恢复后完整重新消抖、精确超时边界和超时/完成竞争用例。
- `docs/integration/flying-car-integration-log.md`：记录安全审查修正证据。

### 验证

- RED 命令：`D:/Down/msys2/ucrt64/bin/g++.exe -std=c++17 -Wall -Wextra -Werror -pedantic -I. .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-3-harness.cpp src/modules/flying_car/FlyingCarModeManager.cpp -o .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-3-harness.exe`，随后运行该程序。
- RED 结果：失败，62 项检查中 29 项失败；失败集中在过渡取消、具体拒绝原因和恢复后的完整重新消抖，复现了过时过渡可以继续完成的根因。
- GREEN 结果：修正后同一严格 C++17 命令通过；增加最终双向重新消抖断言后共 65 项检查。
- 边界语义：活动过渡先判定严格 `elapsed > timeout`；精确等于超时时不进入 Fault，精确同时达到完成和超时时允许完成，超过超时且完成也已到期时 Timeout/Fault 优先。
- 静态检查：使用忽略的最小 GoogleTest/uORB 契约桩执行 GoogleTest 源码语法检查，并检查禁用依赖、CMake 注册和 `git diff --check`。
- 构建限制：本机仍无 Linux/WSL PX4 环境和生成的 uORB 头文件，未运行或声称 PX4 Linux/SITL 构建通过。

### 风险与后续

- 仍需在 Linux PX4 环境运行注册的 GoogleTest 和完整 PX4 回归。
- 已知的 `uint64_t` 时间回绕属于轻微后续项，本次按审查裁定保持不变。

## 2026-09-02：隔离飞行汽车车轮混控与执行器门控

### 修改文件

- `src/modules/flying_car/FlyingCarDifferentialControl.hpp/.cpp`：新增无状态差速混控；按 `throttle - yaw` 和 `throttle + yaw` 生成左右轮输出，限制输出幅度、处理左右独立反向，并将所有无效输入归零。
- `src/modules/flying_car/FlyingCarDifferentialControlTest.cpp`：覆盖直行前进/后退、双向转弯、原地转向、独立/同时反向、正负饱和、无效输入和零/负/超范围限制。
- `src/modules/flying_car/FlyingCarActuatorGate.hpp/.cpp`：新增不依赖生成 uORB 的纯值门控；构型关闭时只报告旁路，Flight、Ground、过渡、Fault 和非法状态分别生成规定的安全输出和可逆标志。
- `src/modules/flying_car/FlyingCarActuatorGateTest.cpp`：覆盖旁路、飞行旋翼透传、非有限旋翼禁用、地面车轮限制/无效值归零、过渡/Fault/非法状态安全输出以及精确可逆位 `48`。
- `src/modules/flying_car/CMakeLists.txt`：注册两个纯库及两个 `px4_add_unit_gtest` 测试。

### 验证

- RED 命令：`D:/Down/msys2/ucrt64/bin/g++.exe -std=c++17 -Wall -Wextra -Werror -pedantic -I. -Isrc/lib .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-4-harness.cpp -o .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-4-harness.exe`。
- RED 结果：产品文件创建前编译失败，首个错误为 `FlyingCarActuatorGate.hpp: No such file or directory`，证明独立测试能捕获缺失接口。
- GREEN 命令：`D:/Down/msys2/ucrt64/bin/g++.exe -std=c++17 -Wall -Wextra -Werror -pedantic -I.superpowers/sdd/2026-09-01-flying-car-isolated-integration/stubs -I. -Isrc/lib .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-4-harness.cpp src/modules/flying_car/FlyingCarDifferentialControl.cpp src/modules/flying_car/FlyingCarActuatorGate.cpp -o .superpowers/sdd/2026-09-01-flying-car-isolated-integration/task-4-harness.exe`，随后运行该程序。
- GREEN 结果：`Task 4 harness: 87 checks, 0 failures`。
- GoogleTest 语法检查：分别对 `FlyingCarDifferentialControlTest.cpp` 和 `FlyingCarActuatorGateTest.cpp` 使用相同严格 C++17 标志、忽略的本地 GoogleTest/PX4 平台桩和 `-fsyntax-only`；结果均为退出码 `0`。
- 隔离检查：`git diff -- src/modules/rover_differential src/modules/control_allocator` 输出为空；禁止依赖扫描未发现 uORB、参数、时钟、模块、动态分配或 PX4 运行时调用。

### 风险与后续

- `wsl.exe -l -q` 返回退出码 `1` 并提示安装 WSL；本机没有 Linux/PX4 构建环境，因此未运行或声称完整 PX4 CMake 构建和真实 GoogleTest 执行通过。
- 后续运行模块必须在 `bypass=true` 时保留原始执行器发布而不发布门控返回值，并将纯门控结果显式复制到生成的 `actuator_motors_s`。
- AUX1–4 DShot 与 AUX5–6 可逆 PWM 的混合定时器能力仍需 Linux 固件构建和无桨硬件台架验证。

## 2026-09-02：运行时模块与专用物理电机输出路径

### 修改文件

- `msg/FlyingCarActuatorMotors.msg` 与 `msg/CMakeLists.txt`：注册只供物理电机输出层消费的 12 路门控消息；它不是控制器设定值。
- `src/modules/flying_car/FlyingCar.hpp/.cpp`：新增 100 Hz `ModuleBase`/`ModuleParams`/`ScheduledWorkItem` 运行时，按原生电机新样本额外调度；启动前检查 `SYS_FC_TYPE=1`，先发布一次安全专用样本，再发布状态。
- 运行时对原生旋翼输入采用 200 ms 新鲜度上限，对两个 Rover 设定值采用 500 ms 上限，过渡超时固定为 5 s；无效水平速度按无穷大处理。
- RC 仅接受有效且来源为 RC 的 `AUX1..AUX6`；小于 `-0.5` 请求 Flight，大于 `0.5` 请求 Ground，中间区间保持原请求。
- `src/lib/mixer_module/functions/FunctionMotors.hpp`：保留原 `actuator_motors` 回调和转换，首个 `flying_car_actuator_motors` 样本到达后锁存专用源，后续原生样本不再覆盖。
- `src/lib/mixer_module/mixer_module_tests.cpp`：增加真实 PX4 功能测试，覆盖原生源、首个安全专用样本接管、后续原生样本不可覆盖、专用可逆位和安全轮中立值；本机未运行该 PX4 测试。
- `boards/px4/fmu-v6x/default.px4board`：仅为 FMUv6X 选择 `CONFIG_MODULES_FLYING_CAR=y` 和 `CONFIG_MODULES_ROVER_DIFFERENTIAL=y`。
- 保留并纳入本任务提交前已存在的设计/计划修订：专用主题隔离、禁止同实例自重发以及 `FunctionMotors` 锁存契约。

### TDD 与验证

- RED：严格 C++17 helper harness 在 `FlyingCar.hpp` 尚不存在时编译失败；增加启动门测试后又以 `configurationAllowsStart` 不存在按预期失败。
- GREEN：同一 helper harness 以 `-std=c++17 -Wall -Wextra -Werror -pedantic` 编译并运行通过，覆盖启动构型、RC 滞环、时间戳新鲜度、有限水平速度和旋翼链就绪。
- 忽略桩语法检查：用最小 PX4/uORB/生成消息桩分别对 `FlyingCar.cpp` 和 `FunctionMotors.hpp` 执行相同严格标志的 `-fsyntax-only`，退出码均为 `0`；该结果不是原生 PX4 构建。
- 纯回归：Task 3 状态机 harness `65 checks` 全部通过；Task 4 差速与门控 harness `87 checks, 0 failures`。
- 结构检查：消息字段与注册、模块 CMake/Kconfig/YAML、FMUv6X 标志、专用发布、禁止模块 main 调用、禁止 `PublicationMulti`/原生电机发布、通用控制器无 diff 均通过；`git diff --check` 单独在提交前执行。
- 构建限制：`Get-Command make` 返回不可用，因此没有运行或声称 `make px4_sitl_default`、`make px4_fmu-v6x_default` 以及真实 `mixer_module_tests` 通过。

### 风险与后续

- 必须在 Linux PX4 环境生成 uORB/参数头并完成 SITL、FMUv6X 和 `mixer_module_tests` 原生编译运行。
- AUX1–4 DShot600 与 AUX5–6 50 Hz 可逆 PWM 的硬件定时器分组、首个专用样本接管和失效安全值仍需无桨台架确认。

## 2026-09-02：Task 5 安全审查修复

- `FunctionMotors` 在首个专用样本后永久锁存飞行汽车源；200 ms 未更新时不回退原生 `actuator_motors`，而是输出旋翼 `NaN`、车轮零值及可逆位 `48`。
- 专用话题回调参与 mixer 的更新判定，支持仅有飞行汽车样本时驱动物理/仿真输出；原有 provider 通过默认 `updated()` 保持接口兼容。
- 模块正常退出前发布最终安全帧；地面链路同时要求时间戳新鲜且油门/转向有限，Ground 的 `timestamp_sample` 取两路 Rover 输入中较早者。
- 对速度阈值、切换延时和车轮限幅做有限值与范围清洗；切换超时调整为 12 s，至少比最大 10 s 驻留延时多 1 s。
- 所有非旁路飞行汽车模式均保持 Motor5/6 可逆位 `48`，避免零轮速被非可逆映射转换为反向满量程。
- 本轮严格独立 helper/gate/provider/stop harness 已通过，`git diff --check` 通过；由于本机缺少 Linux PX4 工具链，原生 `mixer_module_tests`、SITL 和 FMUv6X 构建仍留到 Task 9。

## 2026-09-02：注册 80003 机架与隔离启动

### 修改文件

- `ROMFS/px4fmu_common/init.d/airframes/80003_flying_car`：增加真机 80003；使用原生四旋翼分配、Motor1–4 旋翼、Motor5–6 可逆车轮、`CA_R_REV=48`，并设置 Pixhawk 6X AUX1–4 DShot600、AUX5–6 50 Hz PWM 与车轮 1500/1100..1900 微秒安全范围。
- `ROMFS/px4fmu_common/init.d-posix/airframes/80003_flying_car`：增加同一构型身份和六路仿真输出映射，专用 `flying_car` GZ 模型由 Task 8 提供。
- `rc.flying_car_defaults`、`rc.flying_car_sitl_defaults`：在不改写 `rc.mc_defaults` 的前提下设置 `VEHICLE_TYPE=flying_car` 与 `SYS_FC_TYPE=1`。
- `rc.flying_car_apps`、`rc.vehicle_setup`：复用未修改的 `rc.mc_apps`，仅在 flying-car 分支追加一次 `flying_car start`。
- 三处 ROMFS CMake 注册与 `test/romfs/test_flying_car_airframe.py`：按模块配置收录脚本/airframe，并固定注册唯一性、真机映射及普通机架启动隔离。
- `boards/px4/sitl/default.px4board`：选择飞行汽车模块，使 POSIX 80003 注册与其启动命令在默认 SITL 目标中同时可用。

### 验证

- RED：产品脚本创建前运行 `python test/romfs/test_flying_car_airframe.py`，得到预期的 `FAILED (failures=1, errors=3)`；缺失 80003 注册及三个新脚本触发失败，普通启动脚本隔离检查已通过。
- GREEN：实现后运行同一命令，结果 `Ran 5 tests ... OK`。
- 追加 POSIX 构建选择检查先得到预期的单项 RED，加入 SITL 模块选择后最终结果为 `Ran 6 tests ... OK`。
- `rg -n "80003_flying_car"` 显示硬件与 POSIX CMake 注册表各恰好一次；`rg -n "^flying_car start$" ... -g "rc.*_apps"` 仅命中 `rc.flying_car_apps`。
- 通用 MC、Rover、控制分配及 6003/6004 定向 `git diff` 为空；`git diff --check` 通过。

### 限制与后续

- 本机 `make` 不可用，`wsl.exe -l -q` 以退出码 1 报告未安装 Linux 发行版，因此没有运行或声称 `metadata_airframes`、SITL 启动或 FMUv6X 固件构建通过。
- POSIX airframe 已声明 `PX4_SIM_MODEL=flying_car`，其四旋翼加双轮模型和六路动力学验证属于 Task 8；真机混合协议仍必须通过无桨台架确认。

## 2026-09-02：Commander 窄安全集成

- Commander 增加唯一的 `flying_car_status` 订阅；仅当 `SYS_FC_TYPE=1` 时消费，普通机架不改变解锁检查或车辆类型。
- 状态未收到、时间戳超过 1 s、时间戳位于未来、Transition、Fault 或未知模式均锁定解锁；稳定 Flight/Ground 继续执行原有全部健康检查。
- 启动时尚无稳定状态以 Flight/四旋翼作为安全报告默认；只有新鲜稳定状态可更新记忆类型。Transition、Fault、陈旧状态继续报告上一个稳定类型。
- Flight 对外报告 `VEHICLE_TYPE_ROTARY_WING/MAV_TYPE_QUADROTOR`，Ground 报告 `VEHICLE_TYPE_ROVER/MAV_TYPE_GROUND_ROVER`；参数运行时关闭会立即清除飞行汽车解锁锁定并恢复原生 `MAV_TYPE` 推导。
- HealthAndArmingChecks 不另行订阅 uORB，只接收 Commander 判定出的锁定布尔值；Commander 未包含 flying_car 模块头，也未调用任何模块 main。

### TDD 与验证

- RED：严格 C++17 helper 测试首先因 `FlyingCarSafety.hpp` 不存在而编译失败。
- GREEN：`g++.exe -std=c++17 -Wall -Wextra -Werror test/flying_car_commander_safety_test.cpp` 编译运行退出码 0，覆盖普通构型零干预、缺失/陈旧/未来状态、两个过渡、Fault、稳定状态及稳定类型保持/复位。
- `rg -n 'flying_car_switch_to_|rover_differential_main|control_allocator_main' src/modules/commander` 无命中；`git diff --check` 通过。
- 本机没有可用 `make` 且 WSL 无 Linux 发行版，因此未运行或声称 Commander 原生 GoogleTest、SITL 或 FMUv6X 构建通过；这些验证保留到 Task 9 的受支持 Linux PX4 环境。

### Task 7 安全复审修复

- 复审发现 `arm()` 的 RC 五秒重解锁宽限和 `run_preflight_checks=false` 路径会跳过 SystemChecks。现已在宽限与可选预检分支之前增加独立、无条件的飞行汽车入口门。
- `SYS_FC_TYPE=1` 时，Commander 从构造/参数启用起默认保持入口锁定；只有同一循环中处理到新鲜稳定状态后才允许任何来源进入解锁。SystemChecks 原诊断继续保留，但不再是安全门的唯一执行点。
- 新增 RED/GREEN 用例固定入口真值表：普通构型旁路，飞行汽车仅在锁定为 false 时允许；严格 helper 编译运行退出码 0。

### Task 7 复审测试补强

- 在原生 `HealthAndArmingChecksTest.cpp` 增加 `SystemChecks + Context + Reporter` 集成测试：飞行汽车锁定布尔为 true 时最终 `canArm()` 为 false，普通构型/稳定状态传入 false 时不产生该拒绝。
- 增加可执行 Commander 源码结构回归，明确要求飞行汽车入口门位于 `run_preflight_checks=false` 的 RC 宽限改写和 `if (run_preflight_checks)` 之前，并用故意把门放到末尾的样例证明检查器会拒绝 `arm(false)` 绕过结构。
- 结构测试初次因检查模块不存在而 RED；实现后 `python -m unittest discover -s test -p 'test_flying_car_commander_arm_gate.py' -v` 运行 2 项全部通过。真实 PX4 GoogleTest 源已保留，但本机缺少生成头与 Linux 构建工具链，未声称原生执行通过。

## 2026-09-02：Gazebo 六执行器飞行汽车模型

### 模型与映射

- 拒绝参考树的 Iris 兼容替代：参考树没有 `Tools/simulation/gz/models/flying_car`，无法产生 Motor5/6 车轮动力学或验证飞行/地面输出隔离。
- 新增自包含 `flying_car/model.sdf`：四个独立旋翼 link/joint 与 `MulticopterMotorModel` 插件、两个带碰撞和 ODE 摩擦的轮 link/joint、IMU、NavSat、磁力计和气压计。
- ESC bridge 的四元素数组映射 Motor1–4 到模型 `motorNumber=0..3`；Wheel bridge 的两元素数组通过 `SIM_GZ_WH_FUNC1/2=105/106` 映射 Motor5/6，模型侧 `actuator_number=0/1`。两套 bridge topic 命名空间彼此独立，未修改通用仿真 bridge。
- POSIX airframe 更名为 `80003_gz_flying_car` 以进入现有 `*_gz_*` CMake 扫描并生成 `gz_flying_car` 目标；`px4-rc.gzsim` 仅把 airframe 默认的 `flying_car` 名称规范化为 `gz_flying_car`，其他模型路径不变。

### TDD 与验证

- RED 1：模型缺失时运行 `python Tools/simulation/gz/models/flying_car/model_test.py`，按预期因 `model.sdf` 不存在失败。
- GREEN 1：模型实现后六项 XML 结构契约通过；随后新增 bridge 分流和启动名测试，先因 EC5/6 错误路由及缺少规范化分支得到两项预期失败。
- GREEN 2：修正为 ESC1–4 与 Wheel1–2 后，`Ran 8 tests ... OK`；`xml.etree.ElementTree` 分别解析 `model.config` 和 `model.sdf` 成功。
- `git diff --check` 通过。`git status`/后续 `git ls-files` 用于确认本地基线没有该路径的 gitlink，新增模型可由主仓提交跟踪。

### 限制与风险

- 本机没有 Linux PX4/Gazebo Harmonic 工具链，未运行或声称 `make px4_sitl gz_flying_car`、传感器健康、两种稳定模式动力学或 armed 切换拒绝的 SITL 结果；这些属于 Task 9 的支持环境验证。
- `.gitmodules` 仍把 `Tools/simulation/gz` 描述为外部模型仓，但本项目初始快照没有记录对应 gitlink。当前普通文件可由本仓跟踪；若未来恢复上游子模块布局，必须把本模型移植为该子模块的独立提交或建立明确的模型覆盖目录，不能直接覆盖 gitlink。
