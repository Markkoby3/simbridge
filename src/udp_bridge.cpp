#include "simbridge/udp_bridge.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

#include "simbridge/messages.hpp"
#include "simbridge/wire.hpp"

namespace simbridge {

UdpBridge::UdpBridge(MessageBus& bus, const std::string& host, uint16_t port) : bus_(bus) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        throw std::invalid_argument("UdpBridge: '" + host + "' is not an IPv4 address");
    }
    addr_.resize(sizeof addr);
    std::memcpy(addr_.data(), &addr, sizeof addr);

    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error(std::string("UdpBridge: socket failed: ") + std::strerror(errno));

    subs_.push_back(bus_.subscribe<EntityState>(topics::kEntityState,
                                                [this](const EntityState& s) { send(wire::encode(s)); }));
    subs_.push_back(
        bus_.subscribe<Detection>(topics::kDetection, [this](const Detection& d) { send(wire::encode(d)); }));
}

UdpBridge::~UdpBridge() {
    for (auto id : subs_) bus_.unsubscribe(id);
    if (fd_ >= 0) ::close(fd_);
}

void UdpBridge::send(const std::vector<uint8_t>& packet) {
    const auto n = ::sendto(fd_, packet.data(), packet.size(), 0, reinterpret_cast<const sockaddr*>(addr_.data()),
                            static_cast<socklen_t>(addr_.size()));
    if (n < 0) {
        ++errors_;  // UDP is best effort; a missing listener is not fatal to the simulation
        return;
    }
    ++packets_;
    bytes_ += static_cast<uint64_t>(n);
}

}  // namespace simbridge
