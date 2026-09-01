/*
 * PX4 v1.17 arm_control, ported from the verified PX4 v1.13.3 implementation.
 *
 * Dual-platform HX-30HM arm driver:
 *   - PX4 SITL/posix: /tmp/hxarm_px4 -> Gazebo PTY emulator
 *   - PX4 NuttX/Pixhawk6C: configured serial port -> BusLinker -> HX-30HM bus
 *
 * Optional RC/AUX mode (-r):
 *   Subscribes to manual_control_setpoint and uses PX4 logical AUX functions:
 *     AUX3 -> Joint 1 absolute angle [-90, +90] deg
 *     AUX4 -> Joint 2 absolute angle [-120, +120] deg
 *     AUX5 -> gripper two-state angle OPEN 90 / CLOSED 0 deg
 *
 * RC_MAP_AUX3 / RC_MAP_AUX4 / RC_MAP_AUX5 select which physical receiver
 * channels feed those logical AUX functions. The arm actuator output itself is
 * NEVER PWM/AUX: target angles are encoded as HX-30HM SYNC_WRITE serial frames.
 *
 * The implementation intentionally avoids dynamic STL vector and other dynamic
 * containers so the same module can be compiled for PX4 NuttX FMUv6C.
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static inline bool would_block_errno(int err)
{
#if defined(EWOULDBLOCK) && (EWOULDBLOCK != EAGAIN)
    return err == EAGAIN || err == EWOULDBLOCK;
#else
    return err == EAGAIN;
#endif
}

static inline float constrainf_local(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

class ArmControl final : public ModuleBase<ArmControl>
{
public:
    ArmControl(const char *device, bool rc_enabled)
        : _rc_enabled(rc_enabled)
    {
        strncpy(_device, device, sizeof(_device) - 1);
        _device[sizeof(_device) - 1] = '\0';

        if (px4_sem_init(&_cmd_lock, 0, 1) == 0) {
            _cmd_lock_initialized = true;
            (void)px4_sem_setprotocol(&_cmd_lock, SEM_PRIO_INHERIT);
        }

        if (px4_sem_init(&_cmd_done, 0, 0) == 0) {
            _cmd_done_initialized = true;
        }

        _cmd_sync_ok = _cmd_lock_initialized && _cmd_done_initialized;

        if (!_cmd_sync_ok) {
            PX4_ERR("command synchronization init failed");
        }
    }

    ~ArmControl() override
    {
        if (_manual_control_sub >= 0) {
            orb_unsubscribe(_manual_control_sub);
            _manual_control_sub = -1;
        }

        close_serial();

        if (_cmd_done_initialized) {
            px4_sem_destroy(&_cmd_done);
        }

        if (_cmd_lock_initialized) {
            px4_sem_destroy(&_cmd_lock);
        }
    }

    static int task_spawn(int argc, char *argv[]);
    static ArmControl *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;
    int print_status() override;

private:
    static constexpr uint8_t BROADCAST_ID = 0xFE;

    static constexpr uint8_t INST_PING       = 0x01;
    static constexpr uint8_t INST_READ       = 0x02;
    static constexpr uint8_t INST_WRITE      = 0x03;
    static constexpr uint8_t INST_SYNC_WRITE = 0x83;

    static constexpr uint8_t REG_TORQUE_ENABLE = 40;
    static constexpr uint8_t REG_ACC           = 41;
    static constexpr uint8_t REG_PRESENT_POS_L = 56;

    static constexpr int SERVO_COUNT = 3;
    static constexpr size_t MAX_PARAMS = 40;
    static constexpr size_t MAX_PACKET = MAX_PARAMS + 6;
    static constexpr size_t RX_BUFFER_SIZE = 96;

    // Real-arm calibration retained from the verified Pixhawk 6C package.
    static constexpr int ZERO[SERVO_COUNT] = {2048, 2048, 2048};
    static constexpr int DIR[SERVO_COUNT]  = {-1, +1, +1};
    static constexpr uint8_t ID[SERVO_COUNT] = {1, 2, 3};

    static constexpr float Q_MIN_DEG[SERVO_COUNT] = {-90.0f, -90.0f, 0.0f};
    static constexpr float Q_MAX_DEG[SERVO_COUNT] = { 90.0f,  90.0f, 90.0f};

    // Gripper two-state angles. Swap these values if the linkage is reversed.
    static constexpr float GRIP_OPEN_DEG = 90.0f;
    static constexpr float GRIP_CLOSED_DEG = 0.0f;

    enum class CommandType : uint8_t {
        NONE = 0,
        PING,
        READ,
        ZERO,
        TORQUE,
        JOINT,
        RAW,
        RC
    };

    struct CommandRequest {
        CommandType type{CommandType::NONE};
        int servo_id{0};        // 0 = all configured servos
        bool flag{false};       // torque/rc on|off
        int raw[3]{0, 0, 0};
        float q[3]{0.f, 0.f, 0.f};
    };

    char _device[96]{"/tmp/hxarm_px4"};
    int _fd{-1};
    int _manual_control_sub{-1};

    bool _rc_enabled{false};
    bool _rc_seen{false};
    hrt_abstime _last_rc_sample{0};
    hrt_abstime _last_rc_command{0};

    float _target_deg[SERVO_COUNT]{0.f, 0.f, 0.f};
    float _last_rc_target_deg[SERVO_COUNT]{NAN, NAN, NAN};

    // CLI commands are submitted from the NSH/QGC task, but ALL UART I/O must
    // be performed by ArmControl::run(), which owns _fd on NuttX.
    px4_sem_t _cmd_lock{};
    px4_sem_t _cmd_done{};
    bool _cmd_lock_initialized{false};
    bool _cmd_done_initialized{false};
    bool _cmd_sync_ok{false};
    bool _cmd_pending{false};
    bool _cmd_busy{false};
    CommandRequest _cmd_request{};
    int _cmd_result{PX4_ERROR};

    bool open_serial();
    void close_serial();

    static uint8_t checksum(const uint8_t *packet_without_checksum, size_t len);
    bool write_packet(uint8_t id, uint8_t instruction, const uint8_t *params, size_t params_len);
    bool read_status(uint8_t expected_id, uint8_t *params, size_t params_capacity,
                     size_t &params_len, uint8_t &error, uint32_t timeout_us);

    int angle_deg_to_raw(int joint_index, float deg) const;
    float raw_to_angle_deg(int joint_index, int raw) const;

    int ping_one(uint8_t id);
    int ping_all();
    int read_one(int joint_index, int &raw);
    int read_all();
    int set_torque_one(int joint_index, bool on);
    int set_torque(bool on);
    int set_raw_targets(int p1, int p2, int p3, bool log_command = true);
    int set_joint_targets(float q1_deg, float q2_deg, float q3_deg, bool log_command = true);

    int joint_index_from_id(int servo_id) const;
    int submit_command(const CommandRequest &request);
    void process_pending_command();
    void process_aux_control();
};

constexpr uint8_t ArmControl::BROADCAST_ID;
constexpr uint8_t ArmControl::INST_PING;
constexpr uint8_t ArmControl::INST_READ;
constexpr uint8_t ArmControl::INST_WRITE;
constexpr uint8_t ArmControl::INST_SYNC_WRITE;
constexpr uint8_t ArmControl::REG_TORQUE_ENABLE;
constexpr uint8_t ArmControl::REG_ACC;
constexpr uint8_t ArmControl::REG_PRESENT_POS_L;
constexpr int ArmControl::SERVO_COUNT;
constexpr size_t ArmControl::MAX_PARAMS;
constexpr size_t ArmControl::MAX_PACKET;
constexpr size_t ArmControl::RX_BUFFER_SIZE;
constexpr int ArmControl::ZERO[SERVO_COUNT];
constexpr int ArmControl::DIR[SERVO_COUNT];
constexpr uint8_t ArmControl::ID[SERVO_COUNT];
constexpr float ArmControl::Q_MIN_DEG[SERVO_COUNT];
constexpr float ArmControl::Q_MAX_DEG[SERVO_COUNT];
constexpr float ArmControl::GRIP_OPEN_DEG;
constexpr float ArmControl::GRIP_CLOSED_DEG;

int ArmControl::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd(
        "arm_control",
        SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,
        3200,
        (px4_main_t)&run_trampoline,
        (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return PX4_OK;
}

ArmControl *ArmControl::instantiate(int argc, char *argv[])
{
    const char *device = "/tmp/hxarm_px4";
    bool rc_enabled = false;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-d") && i + 1 < argc) {
            device = argv[++i];

        } else if (!strcmp(argv[i], "-r")) {
            rc_enabled = true;
        }
    }

    return new ArmControl(device, rc_enabled);
}

bool ArmControl::open_serial()
{
    if (_fd >= 0) {
        return true;
    }

    _fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (_fd < 0) {
        const bool waiting_for_sim_pty =
            ((!strcmp(_device, "/tmp/hxarm_px4") || !strcmp(_device, "/tmp/hxarm_hitl")) && errno == ENOENT);

        if (!waiting_for_sim_pty) {
            PX4_ERR("open %s failed: %s", _device, strerror(errno));
        }

        return false;
    }

    termios tio{};

    if (tcgetattr(_fd, &tio) != 0) {
        PX4_ERR("tcgetattr(%s) failed: %s", _device, strerror(errno));
        close_serial();
        return false;
    }

    // Avoid cfmakeraw(): keep the code compatible with both glibc and PX4/NuttX termios.
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tio.c_cflag |= (CS8 | CLOCAL | CREAD);

#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
#ifdef IXON
    tio.c_iflag &= ~IXON;
#endif
#ifdef IXOFF
    tio.c_iflag &= ~IXOFF;
#endif
#ifdef IXANY
    tio.c_iflag &= ~IXANY;
#endif

#ifndef B1000000
    PX4_ERR("B1000000 is not supported by this PX4/NuttX build");
    close_serial();
    return false;
#else
    if (cfsetispeed(&tio, B1000000) != 0 || cfsetospeed(&tio, B1000000) != 0) {
        PX4_ERR("set 1000000 baud failed: %s", strerror(errno));
        close_serial();
        return false;
    }
#endif

    if (tcsetattr(_fd, TCSANOW, &tio) != 0) {
        PX4_ERR("tcsetattr(%s) failed: %s", _device, strerror(errno));
        close_serial();
        return false;
    }

#ifdef B1000000
    termios verify{};

    if (tcgetattr(_fd, &verify) != 0) {
        PX4_ERR("tcgetattr verify(%s) failed: %s", _device, strerror(errno));
        close_serial();
        return false;
    }

    const speed_t verify_ispeed = cfgetispeed(&verify);
    const speed_t verify_ospeed = cfgetospeed(&verify);

    // glibc generally reports the encoded B1000000 constant, while PX4/NuttX
    // may report the actual numerical baud rate. Accept either representation.
    const bool input_1mbps =
        (verify_ispeed == B1000000) || (verify_ispeed == static_cast<speed_t>(1000000));
    const bool output_1mbps =
        (verify_ospeed == B1000000) || (verify_ospeed == static_cast<speed_t>(1000000));

    if (!input_1mbps || !output_1mbps) {
        PX4_ERR("UART %s baud verify failed: input=%lu output=%lu",
                _device,
                static_cast<unsigned long>(verify_ispeed),
                static_cast<unsigned long>(verify_ospeed));
        close_serial();
        return false;
    }
#endif

    if (tcflush(_fd, TCIOFLUSH) != 0) {
        PX4_WARN("tcflush(%s) failed: %s", _device, strerror(errno));
    }

    PX4_INFO("HX-30HM UART ready: %s @ 1000000 8N1", _device);
    return true;
}

void ArmControl::close_serial()
{
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

uint8_t ArmControl::checksum(const uint8_t *packet_without_checksum, size_t len)
{
    uint8_t sum = 0;

    for (size_t i = 2; i < len; ++i) {
        sum = (uint8_t)(sum + packet_without_checksum[i]);
    }

    return (uint8_t)(~sum);
}

bool ArmControl::write_packet(uint8_t id, uint8_t instruction, const uint8_t *params, size_t params_len)
{
    if (params_len > MAX_PARAMS) {
        PX4_ERR("HX packet params too long: %u", (unsigned)params_len);
        return false;
    }

    if (!open_serial()) {
        return false;
    }

    uint8_t packet[MAX_PACKET]{};
    const size_t total = params_len + 6;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = (uint8_t)(params_len + 2);
    packet[4] = instruction;

    if (params_len > 0 && params != nullptr) {
        memcpy(&packet[5], params, params_len);
    }

    packet[total - 1] = checksum(packet, total - 1);

    const uint8_t *ptr = packet;
    size_t left = total;

    while (left > 0) {
        const ssize_t n = ::write(_fd, ptr, left);

        if (n > 0) {
            ptr += n;
            left -= (size_t)n;

        } else if (n < 0 && would_block_errno(errno)) {
            px4_usleep(200);

        } else {
            PX4_ERR("serial write failed: %s", strerror(errno));
            close_serial();
            return false;
        }
    }

    return true;
}

bool ArmControl::read_status(uint8_t expected_id, uint8_t *params, size_t params_capacity,
                             size_t &params_len, uint8_t &error, uint32_t timeout_us)
{
    params_len = 0;
    error = 0x80;

    if (_fd < 0) {
        return false;
    }

    uint8_t rx[RX_BUFFER_SIZE]{};
    size_t rx_len = 0;
    const hrt_abstime deadline = hrt_absolute_time() + timeout_us;

    while (hrt_absolute_time() < deadline) {
        if (rx_len < RX_BUFFER_SIZE) {
            const ssize_t n = ::read(_fd, &rx[rx_len], RX_BUFFER_SIZE - rx_len);

            if (n > 0) {
                rx_len += (size_t)n;

            } else if (n < 0 && !would_block_errno(errno)) {
                PX4_ERR("serial read failed: %s", strerror(errno));
                close_serial();
                return false;
            }
        }

        while (rx_len >= 2 && !(rx[0] == 0xFF && rx[1] == 0xFF)) {
            memmove(rx, rx + 1, --rx_len);
        }

        if (rx_len >= 4) {
            const uint8_t length = rx[3];

            if (length < 2 || length > 60) {
                memmove(rx, rx + 1, --rx_len);
                continue;
            }

            const size_t total = (size_t)length + 4;

            if (total > RX_BUFFER_SIZE) {
                rx_len = 0;
                continue;
            }

            if (rx_len >= total) {
                uint8_t sum = 0;

                for (size_t i = 2; i + 1 < total; ++i) {
                    sum = (uint8_t)(sum + rx[i]);
                }

                if (rx[total - 1] != (uint8_t)(~sum)) {
                    PX4_WARN("RX checksum error");
                    memmove(rx, rx + 1, --rx_len);
                    continue;
                }

                if (rx[2] != expected_id) {
                    if (rx_len > total) {
                        memmove(rx, rx + total, rx_len - total);
                    }

                    rx_len -= total;
                    continue;
                }

                error = rx[4];
                const size_t available_params = total > 6 ? total - 6 : 0;
                params_len = available_params < params_capacity ? available_params : params_capacity;

                if (params_len > 0 && params != nullptr) {
                    memcpy(params, &rx[5], params_len);
                }

                return true;
            }
        }

        if (rx_len == RX_BUFFER_SIZE) {
            // Corrupt/unsynchronised stream: keep one byte so FF FF can span reads.
            rx[0] = rx[RX_BUFFER_SIZE - 1];
            rx_len = 1;
        }

        px4_usleep(500);
    }

    return false;
}

int ArmControl::angle_deg_to_raw(int joint_index, float deg) const
{
    const float units_per_deg = 4096.0f / 360.0f;
    return (int)lroundf((float)ZERO[joint_index] + (float)DIR[joint_index] * deg * units_per_deg);
}

float ArmControl::raw_to_angle_deg(int joint_index, int raw) const
{
    const float deg_per_unit = 360.0f / 4096.0f;
    return (float)DIR[joint_index] * (float)(raw - ZERO[joint_index]) * deg_per_unit;
}

int ArmControl::ping_one(uint8_t id)
{
    if (_fd >= 0) {
        tcflush(_fd, TCIFLUSH);
    }

    if (!write_packet(id, INST_PING, nullptr, 0)) {
        return PX4_ERROR;
    }

    uint8_t params[8]{};
    size_t params_len = 0;
    uint8_t error = 0;

    if (!read_status(id, params, sizeof(params), params_len, error, 100000)) {
        PX4_ERR("PING ID=%u timeout", (unsigned)id);
        return PX4_ERROR;
    }

    if (error != 0) {
        PX4_ERR("PING ID=%u servo error=0x%02X", (unsigned)id, (unsigned)error);
        return PX4_ERROR;
    }

    PX4_INFO("PING ID=%u OK", (unsigned)id);
    return PX4_OK;
}

int ArmControl::ping_all()
{
    int ret = PX4_OK;

    for (int i = 0; i < SERVO_COUNT; ++i) {
        if (ping_one(ID[i]) != PX4_OK) {
            ret = PX4_ERROR;
        }
    }

    return ret;
}

int ArmControl::read_one(int joint_index, int &raw)
{
    const uint8_t id = ID[joint_index];

    if (_fd >= 0) {
        tcflush(_fd, TCIFLUSH);
    }

    const uint8_t request[2] = {REG_PRESENT_POS_L, 2};

    if (!write_packet(id, INST_READ, request, sizeof(request))) {
        return PX4_ERROR;
    }

    uint8_t params[8]{};
    size_t params_len = 0;
    uint8_t error = 0;

    if (!read_status(id, params, sizeof(params), params_len, error, 100000)) {
        PX4_ERR("READ ID=%u timeout", (unsigned)id);
        return PX4_ERROR;
    }

    if (error != 0 || params_len < 2) {
        PX4_ERR("READ ID=%u invalid status error=0x%02X len=%u",
                (unsigned)id, (unsigned)error, (unsigned)params_len);
        return PX4_ERROR;
    }

    const uint16_t raw_u16 = (uint16_t)params[0] | ((uint16_t)params[1] << 8);
    raw = (int16_t)raw_u16;
    return PX4_OK;
}

int ArmControl::read_all()
{
    int ret = PX4_OK;

    for (int i = 0; i < SERVO_COUNT; ++i) {
        int raw = 0;

        if (read_one(i, raw) == PX4_OK) {
            PX4_INFO("ID%u raw=%d angle=%.2f deg",
                     (unsigned)ID[i], raw, (double)raw_to_angle_deg(i, raw));

        } else {
            ret = PX4_ERROR;
        }
    }

    return ret;
}

int ArmControl::set_torque_one(int joint_index, bool on)
{
    if (joint_index < 0 || joint_index >= SERVO_COUNT) {
        return PX4_ERROR;
    }

    if (_fd >= 0) {
        tcflush(_fd, TCIFLUSH);
    }

    const uint8_t request[2] = {REG_TORQUE_ENABLE, (uint8_t)(on ? 1 : 0)};

    if (!write_packet(ID[joint_index], INST_WRITE, request, sizeof(request))) {
        return PX4_ERROR;
    }

    uint8_t params[8]{};
    size_t params_len = 0;
    uint8_t error = 0;

    if (!read_status(ID[joint_index], params, sizeof(params), params_len, error, 100000) || error != 0) {
        PX4_WARN("torque ID=%u no/invalid ACK", (unsigned)ID[joint_index]);
        return PX4_ERROR;
    }

    PX4_INFO("torque ID=%u %s", (unsigned)ID[joint_index], on ? "ON" : "OFF");
    return PX4_OK;
}

int ArmControl::set_torque(bool on)
{
    int ret = PX4_OK;

    for (int i = 0; i < SERVO_COUNT; ++i) {
        if (set_torque_one(i, on) != PX4_OK) {
            ret = PX4_ERROR;
        }
    }

    if (ret == PX4_OK) {
        PX4_INFO("torque all %s", on ? "ON" : "OFF");
    }

    return ret;
}

int ArmControl::set_raw_targets(int p1, int p2, int p3, bool log_command)
{
    const int raw[SERVO_COUNT] = {p1, p2, p3};

    for (int i = 0; i < SERVO_COUNT; ++i) {
        if (raw[i] < -30719 || raw[i] > 30719) {
            PX4_ERR("ID%u target raw=%d outside -30719..30719", (unsigned)ID[i], raw[i]);
            return PX4_ERROR;
        }
    }

    constexpr uint8_t acc = 50;
    constexpr uint16_t speed = 500;
    constexpr uint16_t time = 0;

    uint8_t params[2 + SERVO_COUNT * 8]{};
    size_t n = 0;
    params[n++] = REG_ACC;
    params[n++] = 7;

    for (int i = 0; i < SERVO_COUNT; ++i) {
        params[n++] = ID[i];
        params[n++] = acc;

        const uint16_t pos_u16 = (uint16_t)(int16_t)raw[i];
        params[n++] = (uint8_t)(pos_u16 & 0xFF);
        params[n++] = (uint8_t)((pos_u16 >> 8) & 0xFF);
        params[n++] = (uint8_t)(time & 0xFF);
        params[n++] = (uint8_t)((time >> 8) & 0xFF);
        params[n++] = (uint8_t)(speed & 0xFF);
        params[n++] = (uint8_t)((speed >> 8) & 0xFF);
    }

    if (!write_packet(BROADCAST_ID, INST_SYNC_WRITE, params, n)) {
        return PX4_ERROR;
    }

    for (int i = 0; i < SERVO_COUNT; ++i) {
        _target_deg[i] = raw_to_angle_deg(i, raw[i]);
    }

    if (log_command) {
        PX4_INFO("SYNC_WRITE raw=[%d,%d,%d]", p1, p2, p3);
    }

    return PX4_OK;
}

int ArmControl::set_joint_targets(float q1_deg, float q2_deg, float q3_deg, bool log_command)
{
    if (!isfinite(q1_deg) || !isfinite(q2_deg) || !isfinite(q3_deg)) {
        PX4_ERR("joint target must be finite");
        return PX4_ERROR;
    }

    const float q[SERVO_COUNT] = {q1_deg, q2_deg, q3_deg};
    int raw[SERVO_COUNT]{};

    for (int i = 0; i < SERVO_COUNT; ++i) {
        if (q[i] < Q_MIN_DEG[i] || q[i] > Q_MAX_DEG[i]) {
            PX4_ERR("q%d=%.2f deg outside [%.1f, %.1f] deg",
                    i + 1, (double)q[i], (double)Q_MIN_DEG[i], (double)Q_MAX_DEG[i]);
            return PX4_ERROR;
        }

        raw[i] = angle_deg_to_raw(i, q[i]);

        if (raw[i] < -30719 || raw[i] > 30719) {
            PX4_ERR("q%d maps to HX-30HM raw=%d outside -30719..30719", i + 1, raw[i]);
            return PX4_ERROR;
        }
    }

    const int ret = set_raw_targets(raw[0], raw[1], raw[2], log_command);

    if (ret == PX4_OK && log_command) {
        PX4_INFO("target q=[%.2f, %.2f, %.2f] deg -> raw=[%d,%d,%d]",
                 (double)q1_deg, (double)q2_deg, (double)q3_deg, raw[0], raw[1], raw[2]);
    }

    return ret;
}

int ArmControl::joint_index_from_id(int servo_id) const
{
    for (int i = 0; i < SERVO_COUNT; ++i) {
        if ((int)ID[i] == servo_id) {
            return i;
        }
    }

    return -1;
}

int ArmControl::submit_command(const CommandRequest &request)
{
    if (!_cmd_sync_ok) {
        PX4_ERR("command synchronization unavailable");
        return PX4_ERROR;
    }

    if (px4_sem_wait(&_cmd_lock) != 0) {
        PX4_ERR("command lock failed");
        return PX4_ERROR;
    }

    if (_cmd_pending || _cmd_busy) {
        px4_sem_post(&_cmd_lock);
        PX4_ERR("another arm_control command is still running");
        return PX4_ERROR;
    }

    // Remove a stale completion token from a previous timed-out caller.
    while (px4_sem_trywait(&_cmd_done) == 0) {}

    _cmd_request = request;
    _cmd_result = PX4_ERROR;
    _cmd_pending = true;
    px4_sem_post(&_cmd_lock);

    // The run() task executes UART work. CLI only waits for completion.
    // 3 seconds is comfortably above the normal 100 ms per-servo reply timeout.
    for (int i = 0; i < 300; ++i) {
        if (px4_sem_trywait(&_cmd_done) == 0) {
            return _cmd_result;
        }

        px4_usleep(10000);
    }

    // If run() has not started the request, cancel it. If already busy, it will
    // finish in the owner task; the completion token is drained next command.
    if (px4_sem_wait(&_cmd_lock) == 0) {
        if (_cmd_pending && !_cmd_busy) {
            _cmd_pending = false;
            _cmd_request.type = CommandType::NONE;
        }

        px4_sem_post(&_cmd_lock);
    }

    PX4_ERR("arm_control command timed out");
    return PX4_ERROR;
}

void ArmControl::process_pending_command()
{
    if (!_cmd_sync_ok) {
        return;
    }

    CommandRequest request{};
    bool have_request = false;

    if (px4_sem_wait(&_cmd_lock) == 0) {
        if (_cmd_pending && !_cmd_busy) {
            request = _cmd_request;
            _cmd_busy = true;
            have_request = true;
        }

        px4_sem_post(&_cmd_lock);
    }

    if (!have_request) {
        return;
    }

    int ret = PX4_ERROR;

    switch (request.type) {
    case CommandType::PING:
        if (request.servo_id == 0) {
            ret = ping_all();

        } else {
            const int joint_index = joint_index_from_id(request.servo_id);

            if (joint_index < 0) {
                PX4_ERR("servo ID=%d is not configured (configured IDs: 1,2,3)", request.servo_id);
                ret = PX4_ERROR;

            } else {
                ret = ping_one((uint8_t)request.servo_id);
            }
        }

        break;

    case CommandType::READ:
        if (request.servo_id == 0) {
            ret = read_all();

        } else {
            const int joint_index = joint_index_from_id(request.servo_id);

            if (joint_index < 0) {
                PX4_ERR("servo ID=%d is not configured (configured IDs: 1,2,3)", request.servo_id);
                ret = PX4_ERROR;

            } else {
                int raw = 0;
                ret = read_one(joint_index, raw);

                if (ret == PX4_OK) {
                    PX4_INFO("ID%u raw=%d angle=%.2f deg",
                             (unsigned)ID[joint_index], raw,
                             (double)raw_to_angle_deg(joint_index, raw));
                }
            }
        }

        break;

    case CommandType::ZERO:
        ret = set_joint_targets(0.f, 0.f, 0.f);
        break;

    case CommandType::TORQUE:
        if (request.servo_id == 0) {
            ret = set_torque(request.flag);

        } else {
            const int joint_index = joint_index_from_id(request.servo_id);

            if (joint_index < 0) {
                PX4_ERR("servo ID=%d is not configured (configured IDs: 1,2,3)", request.servo_id);
                ret = PX4_ERROR;

            } else {
                ret = set_torque_one(joint_index, request.flag);
            }
        }

        break;

    case CommandType::JOINT:
        ret = set_joint_targets(request.q[0], request.q[1], request.q[2]);
        break;

    case CommandType::RAW:
        ret = set_raw_targets(request.raw[0], request.raw[1], request.raw[2]);
        break;

    case CommandType::RC:
        _rc_enabled = request.flag;
        PX4_INFO("manual AUX3/4/5 control %s", _rc_enabled ? "ON" : "OFF");
        ret = PX4_OK;
        break;

    case CommandType::NONE:
    default:
        ret = PX4_ERROR;
        break;
    }

    if (px4_sem_wait(&_cmd_lock) == 0) {
        _cmd_result = ret;
        _cmd_pending = false;
        _cmd_busy = false;
        _cmd_request.type = CommandType::NONE;
        px4_sem_post(&_cmd_lock);
    }

    px4_sem_post(&_cmd_done);
}


void ArmControl::process_aux_control()
{
    if (!_rc_enabled || _manual_control_sub < 0 || _fd < 0) {
        return;
    }

    bool updated = false;

    if (orb_check(_manual_control_sub, &updated) != PX4_OK || !updated) {
        return;
    }

    manual_control_setpoint_s mc{};

    if (orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &mc) != PX4_OK) {
        return;
    }

    const hrt_abstime now = hrt_absolute_time();

    if (!mc.valid || mc.timestamp == 0 || now < mc.timestamp || now - mc.timestamp > 500000) {
        return;
    }

    if (!isfinite(mc.aux3) || !isfinite(mc.aux4) || !isfinite(mc.aux5)) {
        return;
    }

    _rc_seen = true;
    _last_rc_sample = mc.timestamp;

    // RC_MAP_AUX3/4/5 choose which physical receiver channels feed these
    // logical AUX values. manual_control_setpoint already provides normalized
    // values approximately in [-1,+1]. The actuator path remains serial.
    const float n1 = constrainf_local(mc.aux3, -1.0f, 1.0f);
    const float n2 = constrainf_local(mc.aux4, -1.0f, 1.0f);
    const float n3 = constrainf_local(mc.aux5, -1.0f, 1.0f);

    const float q1 = n1 * Q_MAX_DEG[0];
    const float q2 = n2 * Q_MAX_DEG[1];
    const float q3 = (n3 >= 0.0f) ? GRIP_CLOSED_DEG : GRIP_OPEN_DEG;

    const bool first = !isfinite(_last_rc_target_deg[0]);
    const bool changed = first
                      || fabsf(q1 - _last_rc_target_deg[0]) >= 0.5f
                      || fabsf(q2 - _last_rc_target_deg[1]) >= 0.5f
                      || fabsf(q3 - _last_rc_target_deg[2]) >= 1.0f;

    if (changed && (first || now - _last_rc_command >= 50000)) {
        if (set_joint_targets(q1, q2, q3, false) == PX4_OK) {
            _last_rc_target_deg[0] = q1;
            _last_rc_target_deg[1] = q2;
            _last_rc_target_deg[2] = q3;
            _last_rc_command = now;
        }
    }
}

void ArmControl::run()
{
    const bool simulated_pty = !strcmp(_device, "/tmp/hxarm_px4") || !strcmp(_device, "/tmp/hxarm_hitl");
    bool waiting_logged = false;
    bool connected_logged = false;

    _manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

    if (_manual_control_sub >= 0) {
        orb_set_interval(_manual_control_sub, 40);

    } else if (_rc_enabled) {
        PX4_WARN("manual_control_setpoint subscription failed; AUX3/4/5 arm control disabled");
    }

    while (!should_exit()) {
        if (_fd < 0) {
            if (!open_serial()) {
                if (simulated_pty) {
                    if (!waiting_logged) {
                        PX4_INFO("waiting for Gazebo HX-30HM PTY: %s", _device);
                        waiting_logged = true;
                    }

                    px4_usleep(200000);
                    continue;
                }

                PX4_ERR("arm_control stopped: serial unavailable");
                return;
            }

            waiting_logged = false;

            if (!connected_logged) {
                PX4_INFO("started on %s; HX-30HM serial active; AUX3/4/5 control=%s",
                         _device, _rc_enabled ? "ON" : "OFF");
                connected_logged = true;
            }
        }

        // All CLI-triggered UART operations are executed here in the same
        // task that opened _fd. This is required for NuttX-safe descriptor use.
        process_pending_command();
        process_aux_control();
        px4_usleep(20000);
    }
}

int ArmControl::print_status()
{
    PX4_INFO("device: %s", _device);
    PX4_INFO("serial: %s", _fd >= 0 ? "OPEN" : "CLOSED");
    PX4_INFO("manual AUX control: %s (AUX3=J1 AUX4=J2 AUX5=GRIP)", _rc_enabled ? "ON" : "OFF");
    PX4_INFO("target q=[%.2f, %.2f, %.2f] deg",
             (double)_target_deg[0], (double)_target_deg[1], (double)_target_deg[2]);

    if (_rc_seen) {
        PX4_INFO("last RC age: %llu ms",
                 (unsigned long long)((hrt_absolute_time() - _last_rc_sample) / 1000));
    }

    PX4_INFO("zero raw=[%d,%d,%d], dir=[%d,%d,%d]",
             ZERO[0], ZERO[1], ZERO[2], DIR[0], DIR[1], DIR[2]);
    return PX4_OK;
}

int ArmControl::custom_command(int argc, char *argv[])
{
    ArmControl *inst = get_instance();

    if (!inst) {
        PX4_ERR("not running; start arm_control first");
        return PX4_ERROR;
    }

    if (argc < 1) {
        return print_usage("missing command");
    }

    CommandRequest request{};

    if (!strcmp(argv[0], "ping")) {
        request.type = CommandType::PING;

        if (argc == 2) {
            char *endp = nullptr;
            const long id = strtol(argv[1], &endp, 0);

            if (endp == argv[1] || *endp != '\0' || id < 1 || id > 253) {
                return print_usage("ping optional ID must be 1..253");
            }

            request.servo_id = (int)id;

        } else if (argc != 1) {
            return print_usage("ping [id]");
        }

        return inst->submit_command(request);
    }

    if (!strcmp(argv[0], "read")) {
        request.type = CommandType::READ;

        if (argc == 2) {
            char *endp = nullptr;
            const long id = strtol(argv[1], &endp, 0);

            if (endp == argv[1] || *endp != '\0' || id < 1 || id > 253) {
                return print_usage("read optional ID must be 1..253");
            }

            request.servo_id = (int)id;

        } else if (argc != 1) {
            return print_usage("read [id]");
        }

        return inst->submit_command(request);
    }

    if (!strcmp(argv[0], "zero")) {
        if (argc != 1) {
            return print_usage("zero takes no arguments");
        }

        request.type = CommandType::ZERO;
        return inst->submit_command(request);
    }

    if (!strcmp(argv[0], "torque")) {
        if (argc != 2 && argc != 3) {
            return print_usage("torque requires on|off [id]");
        }

        request.type = CommandType::TORQUE;

        if (!strcmp(argv[1], "on")) {
            request.flag = true;

        } else if (!strcmp(argv[1], "off")) {
            request.flag = false;

        } else {
            return print_usage("torque requires on|off [id]");
        }

        if (argc == 3) {
            char *endp = nullptr;
            const long id = strtol(argv[2], &endp, 0);

            if (endp == argv[2] || *endp != '\0' || id < 1 || id > 253) {
                return print_usage("torque optional ID must be 1..253");
            }

            request.servo_id = (int)id;
        }

        return inst->submit_command(request);
    }

    if (!strcmp(argv[0], "rc")) {
        if (argc != 2) {
            return print_usage("rc requires on or off");
        }

        request.type = CommandType::RC;

        if (!strcmp(argv[1], "on")) {
            request.flag = true;

        } else if (!strcmp(argv[1], "off")) {
            request.flag = false;

        } else {
            return print_usage("rc requires on or off");
        }

        return inst->submit_command(request);
    }

    if (!strcmp(argv[0], "joint")) {
        if (argc != 4) {
            return print_usage("joint requires q1 q2 q3");
        }

        char *e1 = nullptr;
        char *e2 = nullptr;
        char *e3 = nullptr;
        request.q[0] = strtof(argv[1], &e1);
        request.q[1] = strtof(argv[2], &e2);
        request.q[2] = strtof(argv[3], &e3);

        if (e1 == argv[1] || *e1 != '\0' || e2 == argv[2] || *e2 != '\0' || e3 == argv[3] || *e3 != '\0') {
            PX4_ERR("invalid joint angle");
            return PX4_ERROR;
        }

        request.type = CommandType::JOINT;
        return inst->submit_command(request);
    }

    if (!strcmp(argv[0], "raw")) {
        if (argc != 4) {
            return print_usage("raw requires p1 p2 p3");
        }

        char *e1 = nullptr;
        char *e2 = nullptr;
        char *e3 = nullptr;
        const long p1 = strtol(argv[1], &e1, 0);
        const long p2 = strtol(argv[2], &e2, 0);
        const long p3 = strtol(argv[3], &e3, 0);

        if (e1 == argv[1] || *e1 != '\0' || e2 == argv[2] || *e2 != '\0' || e3 == argv[3] || *e3 != '\0') {
            PX4_ERR("invalid raw position");
            return PX4_ERROR;
        }

        request.type = CommandType::RAW;
        request.raw[0] = (int)p1;
        request.raw[1] = (int)p2;
        request.raw[2] = (int)p3;
        return inst->submit_command(request);
    }

    return print_usage("unknown command");
}

int ArmControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
3-DOF HX-30HM arm driver for PX4 v1.17, buildable for both SITL and NuttX/FMUV6C.

SITL serial path:
  arm_control -> /tmp/hxarm_px4 -> Gazebo HX-30HM PTY emulator

Direct Pixhawk hardware serial path:
  arm_control -> /dev/ttySx -> BusLinker -> HX-30HM IDs 1/2/3

RC/AUX mode (-r):
  Reads manual_control_setpoint AUX3/AUX4/AUX5.
  AUX3 -> J1 [-90,+90] deg
  AUX4 -> J2 [-120,+120] deg
  AUX5 -> gripper OPEN/CLOSED (90/0 deg)
  Map physical FS-i6S channels with RC_MAP_AUX3/RC_MAP_AUX4/RC_MAP_AUX5.

The arm actuator path is always serial HX-30HM, never AUX/PWM:
  target -> HX-30HM SYNC_WRITE -> /dev/ttySx -> BusLinker -> servos

Note: /dev/ttyUSB0 is a Linux-host device, not a Pixhawk/NuttX device. In HITL,
use the supplied host bridge for /dev/ttyUSB0, or wire BusLinker to a Pixhawk UART.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("arm_control", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/tmp/hxarm_px4", "<device>",
                                    "serial device (SITL /tmp/...; Pixhawk /dev/ttySx)", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('r', "enable manual_control_setpoint AUX3/AUX4/AUX5 control", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("ping", "PING all configured servos, or ping <id>");
    PRINT_MODULE_USAGE_ARG("[id]", "optional configured servo ID (1,2,3)", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("read", "read all configured servos, or read <id>");
    PRINT_MODULE_USAGE_ARG("[id]", "optional configured servo ID (1,2,3)", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("zero", "command q1=q2=q3=0 deg");
    PRINT_MODULE_USAGE_COMMAND_DESCR("joint", "command joint angles in degrees");
    PRINT_MODULE_USAGE_ARG("<q1> <q2> <q3>", "q1:-90..90, q2:-120..120, q3:0..90 deg", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("raw", "command signed HX-30HM absolute positions -30719..30719");
    PRINT_MODULE_USAGE_COMMAND_DESCR("torque", "torque on|off [id]");
    PRINT_MODULE_USAGE_COMMAND_DESCR("rc", "AUX3/AUX4/AUX5 control on|off");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return PX4_OK;
}

extern "C" __EXPORT int arm_control_main(int argc, char *argv[])
{
    return ArmControl::main(argc, argv);
}
