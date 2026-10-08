// Forwards entity_state and detection samples from the bus to a UDP endpoint,
// so an autonomy process (in any language) can consume simulation output.
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "simbridge/message_bus.hpp"

namespace simbridge {

class UdpBridge {
public:
    UdpBridge(MessageBus& bus, const std::string& host, uint16_t port);
    ~UdpBridge();
    UdpBridge(const UdpBridge&) = delete;
    UdpBridge& operator=(const UdpBridge&) = delete;

    uint64_t packets_sent() const { return packets_; }
    uint64_t bytes_sent() const { return bytes_; }
    uint64_t send_errors() const { return errors_; }

private:
    void send(const std::vector<uint8_t>& packet);

    MessageBus& bus_;
    int fd_ = -1;
    std::vector<uint8_t> addr_;  // sockaddr_in storage, kept opaque to callers
    std::vector<SubscriptionId> subs_;
    std::atomic<uint64_t> packets_{0}, bytes_{0}, errors_{0};
};

}  // namespace simbridge
