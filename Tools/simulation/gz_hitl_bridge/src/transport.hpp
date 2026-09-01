#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

extern "C" {
#include <mavlink.h>
}

// Abstract MAVLink transport. Concrete implementations are SerialTransport
// (CDC ACM / UART) and UdpTransport (mavlink-router compatible UDP endpoints).
//
// Each Vehicle owns one Transport. The Transport spawns a background RX thread
// that decodes incoming MAVLink and dispatches every message to the user-set
// handler. Send is synchronous, thread-safe.
class Transport {
public:
    using MessageHandler = std::function<void(const mavlink_message_t &)>;

    Transport(uint8_t system_id, uint8_t component_id)
        : _sys_id(system_id), _comp_id(component_id) {}
    virtual ~Transport() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool send(const mavlink_message_t &msg) = 0;
    virtual const char *description() const = 0;

    void start(MessageHandler handler) {
        _handler = std::move(handler);
        _running = true;
        _rx_thread = std::thread([this]{ rxLoop(); });
    }

    void stop() {
        _running = false;
        if (_rx_thread.joinable()) _rx_thread.join();
    }

    uint8_t system_id() const { return _sys_id; }
    uint8_t component_id() const { return _comp_id; }

    // Factory: parse a URI like "serial:///dev/ttyACM0:921600" or
    // "udp://127.0.0.1:14550" and return the right Transport.
    // sysid/compid are used as the MAVLink identity for outgoing messages.
    static std::unique_ptr<Transport> create(const std::string &uri,
                                             uint8_t sysid, uint8_t compid);

protected:
    // Concrete transports implement this — read bytes (blocking or polled)
    // and feed them to feedBytes(). Loop while _running.
    virtual void rxLoop() = 0;

    // Helper for rxLoop implementations: parse a byte buffer and dispatch
    // completed MAVLink messages.
    void feedBytes(const uint8_t *buf, size_t n, uint8_t channel = MAVLINK_COMM_0) {
        mavlink_message_t msg;
        mavlink_status_t status;
        for (size_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(channel, buf[i], &msg, &status)) {
                if (_handler) _handler(msg);
            }
        }
    }

    std::atomic<bool> _running{false};

private:
    uint8_t _sys_id;
    uint8_t _comp_id;
    std::thread _rx_thread;
    MessageHandler _handler;
};
