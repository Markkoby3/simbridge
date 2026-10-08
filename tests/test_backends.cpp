#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "simbridge/backend.hpp"
#include "simbridge/legacy/block_sim.hpp"

using namespace simbridge;

namespace {

VehicleCommand cmd(uint32_t id, double heading, double speed) {
    VehicleCommand c;
    c.entity_id = id;
    c.heading_rad = heading;
    c.speed_mps = speed;
    return c;
}

class BothBackends : public ::testing::TestWithParam<std::string> {};

}  // namespace

TEST(Angles, WrapPiStaysInHalfOpenRange) {
    EXPECT_NEAR(wrap_pi(3 * kPi), kPi, 1e-12);
    EXPECT_NEAR(wrap_pi(-kPi), kPi, 1e-12);
    EXPECT_NEAR(wrap_pi(2 * kPi + 0.25), 0.25, 1e-12);
    EXPECT_NEAR(wrap_pi(-0.25), -0.25, 1e-12);
}

TEST_P(BothBackends, StraightLineMotion) {
    auto b = make_backend(GetParam());
    b->add_entity(1, "a", Vec3{0, 0, 50}, kPi / 2, 10.0);  // due north at 10 m/s
    for (int i = 0; i < 100; ++i) b->step(0.1);
    const auto s = b->states().at(0);
    EXPECT_NEAR(s.pos.x, 0.0, 1e-6);
    EXPECT_NEAR(s.pos.y, 100.0, 1e-6);
    EXPECT_NEAR(s.pos.z, 50.0, 1e-9);
    EXPECT_NEAR(b->time(), 10.0, 1e-9);
}

TEST_P(BothBackends, TurnRateIsLimited) {
    KinematicLimits lim;
    lim.max_turn_rate_rps = 0.2;
    auto b = make_backend(GetParam(), lim);
    b->add_entity(1, "a", Vec3{}, 0.0, 10.0);
    b->command(cmd(1, kPi / 2, 10.0));
    for (int i = 0; i < 10; ++i) b->step(0.1);  // 1 s at 0.2 rad/s
    EXPECT_NEAR(b->states()[0].heading_rad, 0.2, 1e-9);
}

TEST_P(BothBackends, AccelerationIsLimited) {
    KinematicLimits lim;
    lim.max_accel_mps2 = 2.0;
    auto b = make_backend(GetParam(), lim);
    b->add_entity(1, "a", Vec3{}, 0.0, 0.0);
    b->command(cmd(1, 0.0, 30.0));
    for (int i = 0; i < 20; ++i) b->step(0.1);  // 2 s at 2 m/s^2
    EXPECT_NEAR(b->states()[0].speed_mps, 4.0, 1e-9);
}

TEST_P(BothBackends, StatesSortedByEntityIdAndUnknownCommandsIgnored) {
    auto b = make_backend(GetParam());
    b->add_entity(30, "c", Vec3{}, 0, 0);
    b->add_entity(10, "a", Vec3{}, 0, 0);
    b->add_entity(20, "b", Vec3{}, 0, 0);
    EXPECT_NO_THROW(b->command(cmd(999, 1.0, 5.0)));
    const auto s = b->states();
    ASSERT_EQ(s.size(), 3u);
    EXPECT_EQ(s[0].id, 10u);
    EXPECT_EQ(s[1].id, 20u);
    EXPECT_EQ(s[2].id, 30u);
    EXPECT_EQ(s[0].name, "a");
    EXPECT_THROW(b->add_entity(10, "dup", Vec3{}, 0, 0), std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(Adapters, BothBackends, ::testing::Values("kinematic", "legacy_blocks"));

TEST(LegacyBlocksAdapter, ConvertsUnitsAndFrames) {
    // Heading north (ENU pi/2) at 10 m/s, from 100 m east.
    auto b = make_legacy_blocks_backend();
    b->add_entity(7, "x", Vec3{100, 0, 30.48}, kPi / 2, 10.0);
    const auto s = b->states().at(0);
    EXPECT_EQ(s.id, 7u);  // block id 1 mapped back to entity id 7
    EXPECT_NEAR(s.pos.x, 100.0, 1e-9);
    EXPECT_NEAR(s.pos.z, 30.48, 1e-9);
    EXPECT_NEAR(s.heading_rad, kPi / 2, 1e-12);
    EXPECT_NEAR(s.speed_mps, 10.0, 1e-12);
}

TEST(LegacyBlockSim, NativeConventions) {
    // Speak the legacy API directly: 3600 knots due east (090) for 1 s = 1 nm east.
    legacy::BlockSim sim(1000.0, 1000.0);
    const int id = sim.add_block("b", 0, 0, 0, 90.0, 3600.0);
    EXPECT_EQ(id, 1);
    sim.tick_ms(1000);
    EXPECT_NEAR(sim.blocks()[0].east_ft, 6076.11548556, 1e-6);
    EXPECT_NEAR(sim.blocks()[0].north_ft, 0.0, 1e-9);
    EXPECT_EQ(sim.clock_ms(), 1000);
}

TEST(LegacyBlocksAdapter, CarriesFractionalMillisecondsWithoutDrift) {
    auto b = make_legacy_blocks_backend();
    b->add_entity(1, "a", Vec3{}, 0, 0);
    // 2500 steps of 0.4 ms. The legacy clock only takes whole milliseconds, so a
    // naive adapter would round every step to 0 ms and never advance.
    for (int i = 0; i < 2500; ++i) b->step(0.0004);
    EXPECT_NEAR(b->time(), 1.0, 0.001);
}

// Migration check: drive both adapters with the same open loop command sequence
// and require identical trajectories. Any unit, frame or clock bug in the
// legacy adapter shows up here as a position error.
TEST(BackendParity, OpenLoopTrajectoriesMatch) {
    auto modern = make_kinematic_backend();
    auto legacy = make_legacy_blocks_backend();
    for (auto* b : {modern.get(), legacy.get()}) {
        b->add_entity(1, "uav", Vec3{0, 0, 120}, 0.0, 25.0);
        b->add_entity(2, "boat", Vec3{900, 1400, 0}, -kPi / 2, 6.0);
    }
    double worst = 0.0;
    for (int step = 0; step < 4000; ++step) {
        const double t = step * 0.05;
        // A scripted maneuver schedule: weaving turns and speed changes.
        const VehicleCommand c1 = cmd(1, std::sin(t / 7.0) * 2.5, 20.0 + 8.0 * std::cos(t / 11.0));
        const VehicleCommand c2 = cmd(2, -kPi / 2 + 0.6 * std::sin(t / 5.0), 6.0);
        modern->command(c1);
        modern->command(c2);
        legacy->command(c1);
        legacy->command(c2);
        modern->step(0.05);
        legacy->step(0.05);
        const auto a = modern->states();
        const auto b = legacy->states();
        for (std::size_t i = 0; i < a.size(); ++i) {
            worst = std::max(worst, std::hypot(a[i].pos.x - b[i].pos.x, a[i].pos.y - b[i].pos.y));
        }
    }
    EXPECT_LT(worst, 1e-6) << "adapters diverged by " << worst << " m";
}
