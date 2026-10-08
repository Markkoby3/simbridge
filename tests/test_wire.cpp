#include <gtest/gtest.h>

#include <limits>

#include "simbridge/wire.hpp"

using namespace simbridge;

TEST(Wire, EntityStateRoundTrip) {
    EntityState s;
    s.id = 4242;
    s.name = "uav_alpha";
    s.t = 12.5;
    s.pos = Vec3{1.25, -3.5, 120.0};
    s.vel = Vec3{24.9, -0.1, 0.0};
    s.heading_rad = -0.004;
    s.speed_mps = std::numeric_limits<double>::max();
    const auto pkt = wire::encode(s);
    EXPECT_EQ(pkt.size(), wire::kHeaderSize + 4 + 9 * 8 + 2 + s.name.size());
    EXPECT_EQ(wire::peek_type(pkt.data(), pkt.size()), wire::MsgType::EntityState);
    const EntityState d = wire::decode_entity_state(pkt.data(), pkt.size());
    EXPECT_EQ(d.id, s.id);
    EXPECT_EQ(d.name, s.name);
    EXPECT_EQ(d.t, s.t);
    EXPECT_EQ(d.pos.y, s.pos.y);
    EXPECT_EQ(d.vel.x, s.vel.x);
    EXPECT_EQ(d.heading_rad, s.heading_rad);
    EXPECT_EQ(d.speed_mps, s.speed_mps);
}

TEST(Wire, DetectionRoundTrip) {
    Detection m;
    m.sensor_id = 100;
    m.host_id = 1;
    m.target_id = 2;
    m.t = 3.25;
    m.range_m = 812.5;
    m.bearing_rad = 0.31;
    const auto pkt = wire::encode(m);
    const Detection d = wire::decode_detection(pkt.data(), pkt.size());
    EXPECT_EQ(d.sensor_id, 100u);
    EXPECT_EQ(d.target_id, 2u);
    EXPECT_EQ(d.range_m, 812.5);
    EXPECT_EQ(d.bearing_rad, 0.31);
}

TEST(Wire, HeaderIsLittleEndianSBRG) {
    const auto pkt = wire::encode(Detection{});
    EXPECT_EQ(pkt[0], 'S');
    EXPECT_EQ(pkt[1], 'B');
    EXPECT_EQ(pkt[2], 'R');
    EXPECT_EQ(pkt[3], 'G');
    EXPECT_EQ(pkt[4], wire::kVersion);
    EXPECT_EQ(pkt[6], 2);  // MsgType::Detection
}

TEST(Wire, RejectsCorruptPackets) {
    auto pkt = wire::encode(Detection{});
    EXPECT_THROW(wire::decode_detection(pkt.data(), 5), wire::WireError);                 // truncated header
    EXPECT_THROW(wire::decode_detection(pkt.data(), pkt.size() - 1), wire::WireError);    // truncated payload
    EXPECT_THROW(wire::decode_entity_state(pkt.data(), pkt.size()), wire::WireError);     // wrong type
    auto bad_magic = pkt;
    bad_magic[0] = 'X';
    EXPECT_THROW(wire::peek_type(bad_magic.data(), bad_magic.size()), wire::WireError);
    auto bad_version = pkt;
    bad_version[4] = 9;
    EXPECT_THROW(wire::peek_type(bad_version.data(), bad_version.size()), wire::WireError);
}
