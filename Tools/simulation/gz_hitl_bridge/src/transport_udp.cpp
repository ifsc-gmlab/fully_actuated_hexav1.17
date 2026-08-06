#include "transport_udp.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

UdpTransport::UdpTransport(const std::string &host, int remote_port, int local_port,
                           uint8_t sysid, uint8_t compid)
    : Transport(sysid, compid),
      _host(host), _remote_port(remote_port), _local_port(local_port) {
    _desc = "udp://" + host + ":" + std::to_string(remote_port);
    if (local_port > 0) _desc += "@" + std::to_string(local_port);
}

UdpTransport::~UdpTransport() {
    stop();
    close();
}

bool UdpTransport::open() {
    _fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_fd < 0) {
        std::fprintf(stderr, "[%s] socket: %s\n", _desc.c_str(), std::strerror(errno));
        return false;
    }

    int one = 1;
    setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    // Bind local. If local_port == 0, the OS picks one (client mode).
    sockaddr_in local{};
    local.sin_family      = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port        = htons(_local_port > 0 ? _local_port : _remote_port);

    // If host is "0.0.0.0" we're in pure server mode — bind to remote port,
    // wait for first inbound packet to learn the endpoint.
    bool server_mode = (_host == "0.0.0.0");

    if (server_mode || _local_port > 0) {
        if (bind(_fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
            std::fprintf(stderr, "[%s] bind %d: %s\n", _desc.c_str(),
                         ntohs(local.sin_port), std::strerror(errno));
            close();
            return false;
        }
    }

    if (!server_mode) {
        _remote.sin_family = AF_INET;
        _remote.sin_port   = htons(_remote_port);
        if (inet_pton(AF_INET, _host.c_str(), &_remote.sin_addr) <= 0) {
            std::fprintf(stderr, "[%s] bad host %s\n", _desc.c_str(), _host.c_str());
            close();
            return false;
        }
        _remote_known = true;
    } else {
        std::printf("[%s] server mode — waiting for first packet to learn endpoint\n",
                    _desc.c_str());
    }

    // Non-blocking for poll-based RX.
    int flags = fcntl(_fd, F_GETFL, 0);
    fcntl(_fd, F_SETFL, flags | O_NONBLOCK);

    std::printf("[%s] opened, fd=%d\n", _desc.c_str(), _fd);
    return true;
}

void UdpTransport::close() {
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

void UdpTransport::rxLoop() {
    uint8_t buf[2048];
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);

    while (_running) {
        struct pollfd pfd { _fd, POLLIN, 0 };
        int pret = poll(&pfd, 1, 100);
        if (pret <= 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        ssize_t n = recvfrom(_fd, buf, sizeof(buf), 0,
                             reinterpret_cast<sockaddr*>(&src), &srclen);
        if (n <= 0) continue;

        if (!_remote_known) {
            _remote = src;
            _remote_known = true;
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));
            std::printf("[%s] learned remote: %s:%d\n", _desc.c_str(),
                        ip, ntohs(src.sin_port));
        }

        feedBytes(buf, static_cast<size_t>(n));
    }
}

bool UdpTransport::send(const mavlink_message_t &msg) {
    if (_fd < 0 || !_remote_known) return false;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    std::lock_guard<std::mutex> lk(_tx_mutex);
    ssize_t w = sendto(_fd, buf, len, 0,
                       reinterpret_cast<const sockaddr*>(&_remote), sizeof(_remote));
    if (w < 0) {
        if (errno == EAGAIN || errno == EINTR) return true;
        std::fprintf(stderr, "[%s] sendto: %s\n", _desc.c_str(), std::strerror(errno));
        return false;
    }
    return true;
}
