# 飞行汽车隔离集成验证报告

## 验证结论

验证于 2026-09-02 18:36–18:45（Asia/Shanghai，UTC+08:00）在 Windows 工作树执行。被验证提交为 `b64c454f565af46d9cabd26581e6f23bcb889007`，融合基线为 `1810a5507c9f8676376be583f7a7625189297a0c`。

所有本机可执行的独立 C++、Python、shell 语法、XML、机架元数据和代码隔离检查均以退出码 0 通过，未发现产品缺陷。由于本机没有 PX4 所需的 Linux 构建环境，原生 PX4 GoogleTest、SITL、FMUv6C 和 FMUv6X 构建均为 **NOT RUN**。因此本报告不宣称固件已成功构建，也不宣称已经支持指定真实飞控板或实车/实机硬件。

## 环境和工具

| 项目 | 结果 |
|---|---|
| 主机 | Windows PowerShell，Asia/Shanghai |
| Python | 3.12.7 |
| 独立 C++ 编译器 | MSYS2 UCRT64 g++ 14.2.0 |
| shell 解析器 | `D:/Down/msys2/usr/bin/bash.exe` |
| `make` / CMake / Ninja | NOT FOUND |
| `arm-none-eabi-gcc` | NOT FOUND |
| Gazebo `gz` | NOT FOUND |
| WSL | `wsl.exe -l -q` 退出码 1；没有安装 Linux 发行版 |

长路径下的 Task 5 桩头文件超过 Windows 工具链路径处理能力，因此验证命令临时用 `subst X:` 将同一只读测试输入映射为短路径；没有改变产品源文件或验证语义。

## 已执行验证

以下命令均在仓库根目录执行；除特别说明外，退出码均为 0。

干净克隆中的统一入口为：

```powershell
$env:PATH = 'D:\Down\msys2\ucrt64\bin;D:\Down\msys2\usr\bin;' + $env:PATH
& .\Tools\validation\run_flying_car_validation.ps1
```

第一行只是本次主机的工具发现设置；受跟踪脚本本身不包含本机绝对路径。脚本创建唯一的系统临时目录并保留元数据和测试可执行文件，不删除或覆盖不确定路径。

| 范围 | 命令摘要 | 结果 |
|---|---|---|
| Task 2 消息/参数契约 | PowerShell 解析 `FlyingCarStatus.msg`、`msg/CMakeLists.txt`、`module.yaml` | PASS：12 个状态/拒绝常量、消息注册、9 个参数默认值 |
| Task 3 状态机 | `g++.exe -std=c++17 -Wall -Wextra -Werror -pedantic -I. task-3-harness.cpp FlyingCarModeManager.cpp ...`，然后运行 | PASS：65/65 |
| Task 4 差速和输出门 | 同一严格选项，附 `-Isrc/lib` 和桩目录，编译运行 Task 4 harness | PASS：88/88 |
| Task 5 runtime helper | 严格编译运行 `task-5-harness.cpp` | PASS |
| Task 5 FunctionMotors | 严格编译运行 `task-5-function-motors-syntax.cpp`（生成主题/uORB 桩） | PASS |
| Task 5 停止安全输出 | 严格编译运行 `task-5-runtime-stop.cpp` 及三个纯逻辑源 | PASS |
| Task 6 ROMFS | `python -m unittest discover -s test/romfs -p 'test_flying_car_airframe.py' -v` | PASS：6/6 |
| Task 6 shell | `bash.exe -n` 检查 80003 硬件/POSIX、defaults、apps 四个脚本 | PASS：4/4 |
| 机架元数据 | `python Tools/px_process_airframes.py -a ROMFS/px4fmu_common/init.d/airframes -x <temp>`，再解析 XML 并查找 80003 | PASS |
| Task 7 安全 helper | 严格编译运行 `test/flying_car_commander_safety_test.cpp` | PASS |
| Task 7 解锁门顺序 | `python -m unittest discover -s test -p 'test_flying_car_commander_arm_gate.py' -v` | PASS：2/2 |
| Task 8 模型 | `python Tools/simulation/gz_custom_models/flying_car/model_test.py` | PASS：10/10 |
| Task 8 XML | Python `xml.etree.ElementTree` 解析 `model.config`、`model.sdf` | PASS：2/2 |
| 通用控制器隔离 | `git diff --name-only 1810a55..HEAD -- src/modules/mc_pos_control src/modules/mc_att_control src/modules/mc_rate_control src/modules/rover_differential src/modules/control_allocator` | PASS：空差异 |
| 6003/6004 保持 | `git diff --name-only 1810a55..HEAD -- .../6003_fully_actuated_hexa .../6004_fully_actuated_hexa_arm` | PASS：空差异 |
| 禁止模块控制 | 扫描 flying-car 源和启动脚本中的 Commander、MC、allocator、rover `start/stop` | PASS：无匹配 |
| 补丁格式 | `git diff --check` | PASS |

Task 5 完整命令使用短路径工作树：`g++.exe -std=c++17 -Wall -Wextra -Werror -pedantic -I<task-5-stubs> -I. ...`。所有桩验证只证明被测接口和独立逻辑在严格 C++17 编译下自洽，不等价于 PX4 生成头、链接和调度环境中的原生构建。

### 会话级 standalone harness 证据

Task 3/4/5 的 harness 源和桩位于被 `.superpowers/sdd/.gitignore` 排除的会话目录，不能承诺在干净克隆中存在。以下是本次会话实际使用的完整 PowerShell 命令；它们补充而不替代上述受跟踪验证套件。

```text
$d = '.superpowers/sdd/2026-09-01-flying-car-isolated-integration'
$s = "$d/task-5-stubs"
$gpp = 'D:/Down/msys2/ucrt64/bin/g++.exe'
& $gpp -std=c++17 -Wall -Wextra -Werror -pedantic -I. "$d/task-3-harness.cpp" src/modules/flying_car/FlyingCarModeManager.cpp -o "$d/task-3-harness.exe"
& "$d/task-3-harness.exe"
& $gpp -std=c++17 -Wall -Wextra -Werror -pedantic "-I$d/stubs" -I. -Isrc/lib "$d/task-4-harness.cpp" src/modules/flying_car/FlyingCarDifferentialControl.cpp src/modules/flying_car/FlyingCarActuatorGate.cpp -o "$d/task-4-harness.exe"
& "$d/task-4-harness.exe"
& $gpp -std=c++17 -Wall -Wextra -Werror -pedantic -I. "$d/task-5-harness.cpp" -o "$d/task-5-harness.exe"
& "$d/task-5-harness.exe"
cmd.exe /c subst X: "$PWD"
Set-Location X:\
& $gpp -std=c++17 -Wall -Wextra -Werror -pedantic "-I$s" -I. -include "$s/task5_common.hpp" "$d/task-5-function-motors-syntax.cpp" -o "$d/task-5-function-motors.exe"
& "$d/task-5-function-motors.exe"
& $gpp -std=c++17 -Wall -Wextra -Werror -pedantic "-I$s" -I. -Isrc/lib "$d/task-5-runtime-stop.cpp" src/modules/flying_car/FlyingCarModeManager.cpp src/modules/flying_car/FlyingCarDifferentialControl.cpp src/modules/flying_car/FlyingCarActuatorGate.cpp -o "$d/task-5-runtime-stop.exe"
& "$d/task-5-runtime-stop.exe"
```

## 四类配置隔离对照

“运行模块”表示启动脚本静态契约，未在 SITL 或真实飞控上动态枚举进程。

| 构型 | `SYS_AUTOSTART` | `SYS_FC_TYPE` | 控制模块/附加模块 | 输出映射 | 飞行汽车切换请求 |
|---|---:|---:|---|---|---|
| 原生四旋翼（对照 4001） | 4001 | 0（模块默认） | 原生 `rc.mc_apps`；不启动 `flying_car` | 原生四旋翼 Motor1–4 | flying-car runtime 拒绝启动，Commander flying-car 门旁路，无切换作用 |
| 全驱六旋翼 | 6003 | 0 | 既有全驱控制链 | Motor1–6 全部是六个旋翼 | 无切换作用；机架文件相对基线无差异 |
| 全驱六旋翼 + HX-30HM | 6004 | 0 | 继承 6003；按既有串口配置启动独立 `arm_control` | Motor1–6 仍为六个旋翼，机械臂不占用飞控电机语义 | 无切换作用；机架文件相对基线无差异 |
| 飞行汽车 | 80003 | 1 | 原生 `rc.mc_apps` + 隔离 `flying_car` runtime；不直接启停 Commander/控制器 | Motor1–4 四旋翼；Motor5 左轮；Motor6 右轮；可逆位掩码 48 | 仅 80003 接受；已解锁、未落地、移动、链路异常、超时均由状态机拒绝/进入安全输出 |

80003 硬件脚本将 AUX1–4 配为 DShot600，将 AUX5–6 配为 50 Hz 可逆 PWM，轮侧失能/中位为 1500 μs、范围为 1100–1900 μs。POSIX 脚本对应 ESC1–4 和 Wheel1–2，并选择主仓自有 `flying_car` 模型。该映射目前只有静态和独立测试证据。

## 未运行的原生构建门

以下计划命令均为 **NOT RUN（环境不具备）**，不是失败，也不是通过：

```text
make px4_sitl_default tests TESTFILTER=FlyingCar
make px4_sitl_default tests TESTFILTER=FullyActuated
make px4_sitl_default tests TESTFILTER=Rover
make tests
make px4_sitl_default
make px4_fmu-v6c_default
make px4_fmu-v6c_arm
make px4_fmu-v6x_default
```

没有生成 `.px4` 固件，因此固件路径、大小和 SHA-256 均记为 **N/A**。必须在完整 Linux PX4 v1.17 工具链中运行上述命令并保存日志，之后才可解除构建门。

## 后续硬件与动态验证门

静态/standalone 测试成功不等于真实飞控板或实车/实机已支持。至少还需完成：

1. 在 Linux 上完成全部原生单测、SITL 和四个目标构建，并记录固件 SHA-256、大小及构建产物路径。
2. 在 Gazebo SITL 中动态验证六路执行器、Flight/Ground 双向切换、超时、Fault、RC 丢失和 Commander 重启。
3. 在 Pixhawk 6X 上无桨、车轮离地验证 AUX1–4 DShot600 与 AUX5–6 50 Hz PWM 的定时器组兼容性；确认同组通道不会被错误协议连带配置。
4. 用示波器或协议分析仪确认 Motor1–4 只驱动旋翼、Motor5–6 只驱动车轮，轮电机中位严格为 1500 μs。
5. 完成急停、断链、陈旧 uORB、模块停止、上电模式和解锁门测试，保存参数快照及 ULog。
6. 通过无桨台架后，按分阶段地面/系留/低风险试飞程序验证实车，任何阶段均不得用本报告替代安全审查。
