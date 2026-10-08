// Sends real UDP packets over loopback and decodes them on the receiving side.
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <vector>

#include "simbridge/udp_bridge.hpp"
#include "simbridge/wire.hpp"

using namespace simbridge;

namespace {

class Receiver {
public:
    Receiver() {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;  // let the OS pick a free port
        ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr);
        socklen_t len = sizeof addr;
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
        port_ = ntohs(addr.sin_port);
        timeval tv{2, 0};
        ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    }
    ~Receiver() { ::close(fd_); }
    uint16_t port() const { return port_; }
    std::vector<uint8_t> recv() {
        std::vector<uint8_t> buf(65535);
        const auto n = ::recv(fd_, buf.data(), buf.size(), 0);
        buf.resize(n > 0 ? static_cast<std::size_t>(n) : 0);
        return buf;
    }

private:
    int fd_ = -1;
    uint16_t port_ = 0;
};

}  // namespace

TEST(UdpBridge, ForwardsBusSamplesOverLoopback) {
    Receiver rx;
    MessageBus bus;
    UdpBridge bridge(bus, "127.0.0.1", rx.port());

    EntityState s;
    s.id = 9;
    s.name = "vessel";
    s.pos = Vec3{1, 2, 3};
    bus.publish(topics::kEntityState, s);

    Detection d;
    d.target_id = 9;
    d.range_m = 640.0;
    bus.publish(topics::kDetection, d);

    const auto p1 = rx.recv();
    ASSERT_FALSE(p1.empty()) << "no packet received";
    const EntityState got = wire::decode_entity_state(p1.data(), p1.size());
    EXPECT_EQ(got.id, 9u);
    EXPECT_EQ(got.name, "vessel");
    EXPECT_EQ(got.pos.z, 3.0);

    const auto p2 = rx.recv();
    ASSERT_FALSE(p2.empty());
    EXPECT_EQ(wire::decode_detection(p2.data(), p2.size()).range_m, 640.0);

    EXPECT_EQ(bridge.packets_sent(), 2u);
    EXPECT_EQ(bridge.bytes_sent(), p1.size() + p2.size());
}

TEST(UdpBridge, StopsForwardingWhenDestroyed) {
    MessageBus bus;
    {
        UdpBridge bridge(bus, "127.0.0.1", 9);
        EXPECT_EQ(bus.subscriber_count(topics::kEntityState), 1u);
    }
    EXPECT_EQ(bus.subscriber_count(topics::kEntityState), 0u);
    EXPECT_EQ(bus.subscriber_count(topics::kDetection), 0u);
}

TEST(UdpBridge, RejectsBadAddress) {
    MessageBus bus;
    EXPECT_THROW(UdpBridge(bus, "not-an-ip", 1234), std::invalid_argument);
}
