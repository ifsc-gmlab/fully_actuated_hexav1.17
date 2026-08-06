# gz_hitl_bridge

让 **新版 Gazebo (Harmonic)** 仿真驱动**真实飞控**做硬件在环（HITL）的桥接程序。

PX4 上游 HITL 只支持 Gazebo Classic / jMAVSim / JSBSim，**不支持新版 Gz**。本项目填上这个空缺，并支持：

- ✅ **多机架**：四旋翼、六旋翼、八旋翼、Standard VTOL、固定翼、Rover（内置 preset，可自定义）
- ✅ **多机集群**：一个进程同时驱动 1-10 台飞控（serial + USB hub 或 UDP）
- ✅ **多种传输**：USB 串口、UDP（mavlink-router 分流场景）
- ✅ **YAML 配置文件**：所有参数外置（机型、位置、传输），**一行命令搞定**

---

## 🚀 30 秒快速入门（已编固件+bridge 的前提下）

**1. 改你想要的配置**（机型、位置、传输都在这一个文件里）：

```yaml
# configs/single_x500.yaml
world: default
vehicles:
  - name: drone_0
    sysid: 1
    transport: serial:///dev/ttyACM0
    model: x500              ← Gz spawn 时的名字
    sdf_model: x500          ← 用哪个 SDF 模型
    pose: {x: 0, y: 0, z: 0.3}
    preset: x500_quad        ← 机架预设（决定有多少电机/舵机）
```

**2. 一行命令启动一切**（Gz + spawn 模型 + bridge）：

```bash
./launch.py configs/single_x500.yaml
```

就这样。集群也一样：改成 `configs/swarm_3x_quad.yaml`，启动同一个命令，自动 spawn 3 台、连 3 个飞控。

---

## 1. 环境依赖

```bash
sudo apt install \
    libgz-transport13-dev libgz-msgs10-dev libgz-math7-dev \
    libyaml-cpp-dev
```

- Ubuntu 22.04 + **Gazebo Harmonic** (`gz-sim 8.x`)
- PX4-Autopilot 仓库（至少编过一次任意板子，会生成 MAVLink C 头文件）
- 真实飞控（雷迅 7-Nano / Pixhawk 6X / 6C-mini 等，跑 PX4 NuttX 固件）

---

## 2. ⚠️ 飞控固件必须改一处再编

**默认 PX4 板子配置不带 `pwm_out_sim`** — HITL 必需，它把飞控电机输出转成 MAVLink HIL_ACTUATOR_CONTROLS。
在板子的 `.px4board` 文件加：

```bash
echo 'CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y' >> boards/cuav/7-nano/default.px4board
make cuav_7-nano_default upload
```

(其他板子如 `boards/px4/fmu-v6x/default.px4board` 同理。)

忘了改的现象：飞控启动播错误音，串口 boot log 报 `nsh: pwm_out_sim: command not found`，bridge 永远 `act_rx=0`。

---

## 3. 编译 bridge

```bash
cd /home/ubuntu22/PX4-Autopilot/Tools/simulation/gz_hitl_bridge
cmake -B build -S .
cmake --build build -j$(nproc)
```

---

## 4. 飞控参数（每台飞控一次性）

QGC → MAVLink Console：

```sh
# 选 HITL 机架（决定 SYS_HITL/HIL_ACT_FUNC/机架几何）
# 四旋翼:
param set SYS_AUTOSTART 1001            # HIL Quadcopter X
# VTOL（如果做 VTOL）:
# param set SYS_AUTOSTART 1002          # 标准 VTOL HIL
# 通用要求:
param set MAV_0_RATE 10000              # set_hil_enabled 需要 datarate > 5000
# 集群每台飞控必须有不同 sysid:
param set MAV_SYS_ID 1                  # 第 1 台 = 1, 第 2 台 = 2, ...
param save
reboot
```

重启后验证：
```sh
pwm_out_sim status                       # 必须 running
listener vehicle_status -n 1             # hil_state 必须是 1
mavlink status                           # 找到对应 instance "Hil: Enabled"
```

---

## 5. 启动 — `launch.py` 一键模式（推荐）

只用编辑一个 YAML 文件，然后跑一条命令：

```bash
cd /home/ubuntu22/PX4-Autopilot/Tools/simulation/gz_hitl_bridge

# 单机
./launch.py configs/single_x500.yaml

# 多机集群
./launch.py configs/swarm_3x_quad.yaml

# 混合机型（2 quad + 1 VTOL + 1 rover）
./launch.py configs/swarm_mixed.yaml
```

`launch.py` 做四件事：
1. 设好 Gz 的环境变量（`GZ_SIM_RESOURCE_PATH`、`GZ_SIM_SERVER_CONFIG_PATH`）
2. 启 Gazebo，等世界就绪
3. 按 YAML 里每台 vehicle 的 `pose:` **依次 spawn 所有模型**（每台 yaw 也支持）
4. 启 bridge 进程，把 YAML 直接传给它

按一次 Ctrl-C 同时干掉 bridge 和 Gazebo。

可选参数：
```bash
./launch.py --headless configs/single_x500.yaml      # Gz 无 GUI
./launch.py --gz-only  configs/single_x500.yaml      # 只启 Gz 不启 bridge（调试用）
./launch.py --skip-gz  configs/single_x500.yaml      # 假设 Gz 已经在跑，只启 bridge
```

---

## 6. 启动 — 手动模式（高级 / 不喜欢 Python）

如果不想用 `launch.py`：

```bash
# 终端 1 — 启 Gz + spawn x500
./start_gz.sh                                   # 默认 default 世界 + x500 模型
# 多机：手动循环 spawn 更多
for i in 1 2; do
    gz service -s /world/default/create --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean --timeout 1000 \
        --req "sdf_filename: \"x500\", name: \"x500_${i}\", pose: {position: {x: ${i}.0, z: 0.3}}"
done

# 终端 2 — 启 bridge
./build/gz_hitl_bridge --device /dev/ttyACM0                       # 单机 CLI
./build/gz_hitl_bridge --config configs/swarm_3x_quad.yaml          # 多机 YAML
./build/gz_hitl_bridge --device /dev/ttyACM0 --preset x500_hex      # 切换机架
./build/gz_hitl_bridge --list-presets                                # 看所有 preset
```

输出（每秒一组）：
```
--- status (3 vehicles) ---
[drone_0] sysid=1 sent:hil_s=250 act_rx=200 armed=0 | cb: imu=250 mag=50 baro=20 gps=30 | az=-9.81 baro=1013.2hPa gps=OK lat=47.39797 lon=8.54616
[drone_1] sysid=2 sent:hil_s=250 act_rx=198 armed=0 | cb: imu=250 mag=50 baro=20 gps=30 | az=-9.81 baro=1013.2hPa gps=OK lat=47.39797 lon=8.54616
[drone_2] sysid=3 sent:hil_s=249 act_rx=195 armed=1 | cb: imu=249 mag=50 baro=20 gps=30 | az=-9.81 baro=1013.2hPa gps=OK lat=47.39797 lon=8.54616
```

---

## 7. 内置 Frame Presets

| Preset 名 | 机型 | actuator 配置 |
|----------|------|-------------|
| `x500_quad` | 默认四旋翼 (x500) | 4 motors @ controls[0..3], 150-1000 rad/s |
| `x500_hex` | 通用六旋翼 | 6 motors |
| `x500_octo` | 通用八旋翼 | 8 motors |
| `standard_vtol` | 标准 VTOL | 5 motors @ [0..4] + 4 servos @ [5..8] ±45° |
| `rc_cessna` | RC Cessna 固定翼 | 1 motor + 3 servos (aileron/elevator/rudder) |
| `r1_rover` | 差动驱动 rover | 2 wheels, ±10 rad/s |
| `rover_ackermann` | 阿克曼转向 rover | 1 drive wheel + 1 steering servo |

不在列表里的机型用 **YAML 内联 outputs** 自定义（见 `configs/custom_inline.yaml`）。

---

## 8. YAML 配置文件格式

完整 schema：

```yaml
world: default                     # 顶层世界名（也可放在 defaults 里）

defaults:                          # 可选，应用到每台 vehicle
  baud: 921600                     # 串口波特率
  sensor_link: base_link           # gz IMU/mag/baro 所在的 link 名

vehicles:
  # ----- 用 preset（推荐）-----
  - name: drone_0                  # 日志前缀
    sysid: 1                       # MAVLink 系统 ID（必须和飞控 MAV_SYS_ID 匹配）
    transport: serial:///dev/ttyACM0    # 或 udp://host:port
    model: x500_0                  # gz 模型名（spawn 时起的名字，必须唯一）
    sdf_model: x500                # 用哪个 SDF（默认 = model 去掉 _N 后缀）
    pose: {x: 0, y: 0, z: 0.3, yaw: 0}    # 仅 launch.py 用，bridge 忽略
    preset: x500_quad              # 内置 preset 名

  # ----- 自定义 outputs -----
  - name: my_hex
    sysid: 2
    transport: serial:///dev/ttyUSB0
    model: my_custom_hex
    outputs:                       # 不用 preset 时必填
      - type: motor                # motor | servo | wheel
        channels: [0, 1, 2, 3, 4, 5]   # HIL_ACTUATOR_CONTROLS 哪些 slot
        min: 100                   # controls=0 时输出值
        max: 1200                  # controls=1 时输出值
        bidirectional: false       # true: 把 [-1,1] 映射到 [min,max]
      - type: servo
        channels:
          - {hil: 6, gz: 0, min: -0.78, max: 0.78, bidirectional: true}
          - {hil: 7, gz: 1, min: -0.78, max: 0.78}
```

附带的示例：
- `configs/single_x500.yaml` — 单四旋翼
- `configs/swarm_3x_quad.yaml` — 3 台 x500，2 USB + 1 UDP
- `configs/swarm_mixed.yaml` — **混合机型集群**：2 quad + 1 VTOL + 1 rover
- `configs/vtol.yaml` — 标准 VTOL
- `configs/fixedwing.yaml` — RC Cessna
- `configs/rover.yaml` — 差动驱动 rover
- `configs/custom_inline.yaml` — 完全自定义六旋翼

### `pose` 字段细节（位置 + 朝向）

```yaml
pose: {x: 0, y: 0, z: 0.3, yaw: 0}
```

- `x, y, z`: 米（NED 反向 ENU，z 朝上，所以 z=0.3 表示离地 30cm）
- `yaw`: 弧度（绕 z 轴旋转）
- 不写 `pose:` 时全用 0（z 默认 0.3 防穿地）
- 集群必须每台位置错开 1-3 米，否则模型会重叠然后飞出去

### `sdf_model` 自动推断

如果不写 `sdf_model`，launcher 会从 `model` 名字推断：
- `model: x500_0` → 自动找 `models/x500/`（去掉 `_0` 后缀）
- `model: standard_vtol_2` → 自动找 `models/standard_vtol/`
- `model: my_custom_drone` → 找 `models/my_custom_drone/`

所以同样机型多机集群只用写 `model: x500_0`、`x500_1`、…，不用每台都写 `sdf_model`。

---

## 9. Transport URI 格式

| URI | 含义 |
|-----|------|
| `serial:///dev/ttyACM0:921600` | 串口，带波特率 |
| `serial:///dev/ttyACM0` | 串口，用 `--baud` 或 YAML `baud:` 配置 |
| `/dev/ttyACM0` | 串口简写（默认 921600） |
| `udp://127.0.0.1:14550` | UDP 发送+接收到此端口；本地端口由 OS 分配 |
| `udp://127.0.0.1:14550@14600` | UDP，本地绑定 14600 |
| `udp://0.0.0.0:14550` | UDP 服务器，等第一个 packet 学会对端地址 |

### USB 共享给 QGC：用 mavlink-routerd 分流

```bash
sudo apt install mavlink-router
mavlink-routerd /dev/ttyACM0:921600 -e 127.0.0.1:14550 -e 127.0.0.1:14551
# bridge:  --device udp://127.0.0.1:14550
# QGC:    UDP 链接到 127.0.0.1:14551
```

集群场景每台飞控一个端口：
```bash
mavlink-routerd \
    /dev/ttyACM0:921600 -e 127.0.0.1:14550 \
    /dev/ttyACM1:921600 -e 127.0.0.1:14551 \
    /dev/ttyACM2:921600 -e 127.0.0.1:14552
```

---

## 10. 多机注意事项

1. **每台 FC 的 `MAV_SYS_ID` 必须不同**（默认全是 1，启动两台会冲突）
2. **每台 Gz 模型名必须不同**（`x500_0`, `x500_1`, `x500_2` …）
3. **Spawn 模型时位置要错开**，否则会重叠飞起来
4. **USB 不够用就走 UDP**：1-3 台 USB 还行，超过 3 台建议 mavlink-routerd + UDP
5. **同一进程跑多机的资源开销**：每台多约 50KB 内存 + ~3% CPU @ 250Hz IMU

---

## 11. 文件结构

```
gz_hitl_bridge/
├── CMakeLists.txt
├── README.md
├── launch.py                        # 一键启动：启 Gz + spawn 模型 + 启 bridge
├── start_gz.sh                      # 仅启 Gz + spawn 单个模型（手动模式用）
├── configs/
│   ├── single_x500.yaml
│   ├── swarm_3x_quad.yaml
│   ├── swarm_mixed.yaml
│   ├── vtol.yaml
│   ├── fixedwing.yaml
│   ├── rover.yaml
│   └── custom_inline.yaml
└── src/
    ├── main.cpp                     # argparse + CLI/YAML 双模式
    │
    ├── transport.hpp                # 抽象 Transport 接口
    ├── transport_serial.{hpp,cpp}   # USB 串口实现
    ├── transport_udp.{hpp,cpp}      # UDP 实现
    ├── transport_factory.cpp        # URI 解析 & 工厂方法
    │
    ├── actuator.{hpp,cpp}           # MotorGroup / ServoGroup / WheelGroup
    ├── preset.{hpp,cpp}             # 内置机架 preset 注册
    │
    ├── vehicle.{hpp,cpp}            # 单机桥接逻辑（gz sub + MAVLink IO）
    ├── vehicle_manager.{hpp,cpp}    # 多机协调
    │
    └── config.{hpp,cpp}             # YAML 解析
```

---

## 12. 坐标系 / 单位约定

- **Gz IMU** 是 **FLU** (Forward-Left-Up)，HIL_SENSOR 要 **FRD**：accel/gyro 的 Y/Z 取反
- **Gz mag 插件** 在左手系发 **Gauss**（不是 Tesla，字段名 `field_tesla` 是历史遗留命名）：
  - `x_frd = -y_gz`, `y_frd = -x_gz`, `z_frd = z_gz`
- **HIL_GPS**：lat/lon int32 单位 1e-7°，alt int32 单位 mm，速度 int16 单位 cm/s
- **气压**：Gz 发 Pa，HIL_SENSOR 要 hPa（÷100）。高度用国际标准大气公式由 Pa 反算

---

## 13. 输出读法

```
[NAME] sysid=N sent:hil_s=A act_rx=B armed=0/1 | cb: imu=I mag=M baro=BA gps=G | az=... baro=... gps=OK/NO lat=... lon=...
```

| 字段 | 期望 | 异常含义 |
|------|------|---------|
| `hil_s` 增长 | ≈250/s | 跟 Gz IMU 速率绑定 |
| `act_rx` 增长 | ≈200/s | FC 在回传电机命令 |
| `armed` | 解锁前 0 | 用 SAFETY_ARMED 位 gate motor 输出 |
| `cb: imu/mag/baro/gps` | 都涨 | 任何一项 = 0 说明对应传感器订阅没生效 |
| `az` | ≈ -9.81 | 重力方向对（FRD z 朝下） |
| `gps=OK` lat/lon | 47.397, 8.546 | 苏黎世（Gz 默认 GPS 起点） |

---

## 14. 排查常见问题

| 现象 | 原因 | 修复 |
|------|------|------|
| `act_rx=0`，FC 启动错误音 | `pwm_out_sim` 没编进固件 | 见 §2 |
| `cb: imu/mag/baro/gps=0` | Gz 没加载 server.config，传感器系统插件没起 | 用 `start_gz.sh`（自动设 `GZ_SIM_SERVER_CONFIG_PATH`） |
| `Bad transport: udp://...` | URI 写错 | 见 §9 格式 |
| `unknown preset: xxx` | preset 名拼错 | `--list-presets` 查 |
| 一台 OK，第二台 `act_rx=0` | 两台 `MAV_SYS_ID` 相同 | 见 §10 |
| QGC 显示 `Flight termination` | 起飞后 GPS 失效触发 failsafe | 保证 `cb: gps` 持续增长 |
| `Device or resource busy` | QGC / 上次 bridge 没退 | `pkill -f gz_hitl`；QGC 走 TELEM 或 mavlink-routerd |
| 集群一台模型直接掉下来 | Gz 模型重名导致 spawn 失败 | `gz topic -l \| grep motor_speed` 确认每台都有 |

---

## 15. 命令速查

```bash
# 编固件（一次性）
make cuav_7-nano_default upload

# 编 bridge
cmake --build /home/ubuntu22/PX4-Autopilot/Tools/simulation/gz_hitl_bridge/build -j$(nproc)

# 🚀 一键启动（推荐）
./launch.py configs/single_x500.yaml
./launch.py configs/swarm_3x_quad.yaml
./launch.py configs/swarm_mixed.yaml

# 手动模式（不用 launch.py）
./start_gz.sh                          # 终端 1
./build/gz_hitl_bridge --device /dev/ttyACM0   # 终端 2

# 查所有 preset
./build/gz_hitl_bridge --list-presets

# 清理
pkill -f gz_hitl_bridge
pkill -f "gz sim"; pkill -f ruby
```

---

## 16. 局限性

- HIL_STATE_QUATERNION 没发（EKF2 用不到，调试需要可加 pose callback）
- 差压（airspeed）没桥接 — 固定翼/VTOL 用空速做 EKF 修正可能需要加
- 集群上限实测约 8-10 台（单进程，CPU 限制）— 再多建议拆分多个 bridge 进程
- UDP 没做包重发 — MAVLink 自己重传 HEARTBEAT/PARAM，普通 HIL 流够用

---

## 17. 改了什么外部仓库

- `boards/cuav/7-nano/default.px4board`: 加 `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y`

如果你用其他板子，照样在那个板子的 `.px4board` 加一行就行。
