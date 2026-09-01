#include "transport.hpp"
#include "transport_serial.hpp"
#include "transport_udp.hpp"

#include <cstdio>
#include <stdexcept>
#include <string>

namespace {
// Parse "host:port" or "host:port@localport". Returns true on success.
bool parseHostPort(const std::string &s, std::string &host,
                   int &port, int &local_port) {
    local_port = 0;
    auto at = s.find('@');
    std::string main = (at == std::string::npos) ? s : s.substr(0, at);
    if (at != std::string::npos) {
        local_port = std::atoi(s.substr(at + 1).c_str());
    }
    auto colon = main.rfind(':');
    if (colon == std::string::npos) return false;
    host = main.substr(0, colon);
    port = std::atoi(main.substr(colon + 1).c_str());
    return port > 0 && !host.empty();
}
}

std::unique_ptr<Transport> Transport::create(const std::string &uri,
                                             uint8_t sysid, uint8_t compid) {
    // Accepted forms:
    //   serial:///dev/ttyACM0:921600
    //   serial:///dev/ttyUSB0:57600
    //   udp://127.0.0.1:14550            -- send + recv to/from this endpoint
    //   udp://127.0.0.1:14550@14551      -- bind locally to :14551
    //   udp://0.0.0.0:14550              -- server mode, learn peer on first packet
    //
    // For backwards compatibility: a path beginning with "/" is treated as a
    // serial device at default 921600. Anything else without scheme is an error.
    if (uri.rfind("serial://", 0) == 0) {
        std::string rest = uri.substr(std::string("serial://").size());
        auto colon = rest.rfind(':');
        std::string dev = (colon == std::string::npos) ? rest : rest.substr(0, colon);
        int baud = (colon == std::string::npos) ? 921600
                                                : std::atoi(rest.substr(colon+1).c_str());
        return std::make_unique<SerialTransport>(dev, baud, sysid, compid);
    }
    if (uri.rfind("udp://", 0) == 0) {
        std::string rest = uri.substr(std::string("udp://").size());
        std::string host;
        int port = 0, local_port = 0;
        if (!parseHostPort(rest, host, port, local_port)) {
            std::fprintf(stderr, "Bad UDP URI: %s\n", uri.c_str());
            return nullptr;
        }
        return std::make_unique<UdpTransport>(host, port, local_port, sysid, compid);
    }
    // Bare path "/dev/ttyACM0" → assume serial @ 921600
    if (!uri.empty() && uri[0] == '/') {
        return std::make_unique<SerialTransport>(uri, 921600, sysid, compid);
    }
    std::fprintf(stderr, "Unknown transport URI: %s\n", uri.c_str());
    return nullptr;
}
