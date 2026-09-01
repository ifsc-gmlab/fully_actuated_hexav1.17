#include "transport_serial.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

SerialTransport::SerialTransport(const std::string &device, int baud,
                                 uint8_t sysid, uint8_t compid)
    : Transport(sysid, compid), _device(device), _baud(baud),
      _desc("serial://" + device + ":" + std::to_string(baud)) {}

SerialTransport::~SerialTransport() {
    stop();
    close();
}

int SerialTransport::baudToTermios(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        default:      return -1;
    }
}

bool SerialTransport::open() {
    _fd = ::open(_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        std::fprintf(stderr, "[%s] open: %s\n", _desc.c_str(), std::strerror(errno));
        return false;
    }

    struct termios tio {};
    if (tcgetattr(_fd, &tio) != 0) {
        std::fprintf(stderr, "[%s] tcgetattr: %s\n", _desc.c_str(), std::strerror(errno));
        close();
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    int b = baudToTermios(_baud);
    if (b < 0) {
        std::fprintf(stderr, "[%s] unsupported baud %d\n", _desc.c_str(), _baud);
        close();
        return false;
    }
    cfsetispeed(&tio, b);
    cfsetospeed(&tio, b);

    if (tcsetattr(_fd, TCSANOW, &tio) != 0) {
        std::fprintf(stderr, "[%s] tcsetattr: %s\n", _desc.c_str(), std::strerror(errno));
        close();
        return false;
    }
    tcflush(_fd, TCIOFLUSH);

    std::printf("[%s] opened, fd=%d\n", _desc.c_str(), _fd);
    return true;
}

void SerialTransport::close() {
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

void SerialTransport::rxLoop() {
    uint8_t buf[512];
    while (_running) {
        struct pollfd pfd { _fd, POLLIN, 0 };
        int pret = poll(&pfd, 1, 100);
        if (pret <= 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        ssize_t n = ::read(_fd, buf, sizeof(buf));
        if (n <= 0) continue;
        feedBytes(buf, static_cast<size_t>(n));
    }
}

bool SerialTransport::send(const mavlink_message_t &msg) {
    if (_fd < 0) return false;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    std::lock_guard<std::mutex> lk(_tx_mutex);
    ssize_t written = 0;
    while (written < len) {
        ssize_t w = ::write(_fd, buf + written, len - written);
        if (w < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            std::fprintf(stderr, "[%s] write: %s\n", _desc.c_str(), std::strerror(errno));
            return false;
        }
        written += w;
    }
    return true;
}
