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
