#pragma once

#include "transport.hpp"

#include <mutex>
#include <netinet/in.h>
#include <string>

// UDP transport for MAVLink. Two modes:
//   udp://0.0.0.0:14550               — bind only (server mode), learns remote
//                                       from first inbound packet
//   udp://192.168.0.5:14550           — bind ephemeral, send to that endpoint
//                                       (client mode)
//   udp://192.168.0.5:14550@14560     — bind locally to :14560, target the remote
//
// Typical use with mavlink-router on the bridge host:
//   mavlink-routerd /dev/ttyACM0:921600 -e 127.0.0.1:14550 -e 127.0.0.1:14551
// then bridge:    --device udp://127.0.0.1:14550
// and QGC:        UDP link to 127.0.0.1:14551
class UdpTransport : public Transport {
public:
    UdpTransport(const std::string &host, int remote_port, int local_port,
                 uint8_t sysid, uint8_t compid);
    ~UdpTransport() override;

    bool open() override;
    void close() override;
    bool send(const mavlink_message_t &msg) override;
    const char *description() const override { return _desc.c_str(); }

protected:
    void rxLoop() override;

private:
    std::string _host;
    int _remote_port;
    int _local_port;
    std::string _desc;
    int _fd{-1};
    sockaddr_in _remote{};
    bool _remote_known{false};
    std::mutex _tx_mutex;
};
