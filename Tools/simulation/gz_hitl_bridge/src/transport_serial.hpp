#pragma once

#include "transport.hpp"

#include <mutex>
#include <string>

class SerialTransport : public Transport {
public:
    SerialTransport(const std::string &device, int baud,
                    uint8_t sysid, uint8_t compid);
    ~SerialTransport() override;

    bool open() override;
    void close() override;
    bool send(const mavlink_message_t &msg) override;
    const char *description() const override { return _desc.c_str(); }

protected:
    void rxLoop() override;

private:
    static int baudToTermios(int baud);

    std::string _device;
    int _baud;
    std::string _desc;
    int _fd{-1};
    std::mutex _tx_mutex;
};
