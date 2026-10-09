// Integration tests: scenario + real plugin libraries + backend + bus, end to end.
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "simbridge/engine.hpp"

using namespace simbridge;

namespace {

const std::string kPlugins = SIMBRIDGE_PLUGIN_DIR;
const std::string kCoastal = std::string(SIMBRIDGE_SCENARIO_DIR) + "/coastal_patrol.scn";

// Owns everything a run needs, in the correct destruction order.
struct Harness {
    PluginRegistry plugins;
    MessageBus bus;
    std::vector<Detection> detections;
    std::unique_ptr<SimEngine> engine;

    explicit Harness(Scenario sc) {
        declare_standard_topics(bus, sc.entities.size());
        plugins.load_directory(kPlugins);
        bus.subscribe<Detection>(topics::kDetection, [this](const Detection& d) { detections.push_back(d); });
        const std::string backend = sc.backend;
        engine = std::make_unique<SimEngine>(std::move(sc), make_backend(backend), plugins, bus);
    }
    RunStats run() { return engine->run(); }
    EntityState state(uint32_t id) const {
        for (const auto& s : engine->backend().states())
            if (s.id == id) return s;
        throw std::runtime_error("no entity " + std::to_string(id));
    }
};

Scenario coastal(const std::string& backend, uint64_t seed = 42) {
    Scenario sc = load_scenario(kCoastal);
    sc.backend = backend;
    sc.seed = seed;
    return sc;
}

}  // namespace

TEST(Engine, RunsShippedScenarioEndToEnd) {
    Harness h(coastal("kinematic"));
    const RunStats st = h.run();
    EXPECT_EQ(st.steps, 3600u);
    EXPECT_NEAR(st.sim_time_s, 180.0, 1e-6);
    EXPECT_EQ(st.commands, 2u * 3600u);  // two waypoint followers, one command per frame
    EXPECT_GT(st.detections, 0u);
    EXPECT_EQ(st.detections, h.detections.size());
    // The vessel holds course south at 6 m/s for the whole run: 1400 - 6 * 180 = 320.
    EXPECT_NEAR(h.state(2).pos.y, 320.0, 1e-6);
    EXPECT_NEAR(h.state(3).pos.x, 1200.0, 1e-9);  // the passive buoy never moves
}

TEST(Engine, WaypointFollowerReachesAndStops) {
    Harness h(parse_scenario_string(R"(
[scenario]
duration_s = 60
timestep_s = 0.05
[entity uav]
id = 1
model = waypoint_follower
position = 0, 0, 100
heading_deg = 90
speed_mps = 20
waypoints = 600, 0, 100
)"));
    h.run();
    const EntityState s = h.state(1);
    EXPECT_GT(s.pos.x, 575.0);   // got inside the 25 m arrival radius
    EXPECT_LT(s.pos.x, 700.0);   // then braked instead of flying on
    EXPECT_NEAR(s.speed_mps, 0.0, 1e-9);
}

TEST(Engine, RadarHonorsRangeAndFieldOfView) {
    Harness h(parse_scenario_string(R"(
[scenario]
duration_s = 1
timestep_s = 0.1
[entity host]
id = 1
model = none
position = 0, 0, 0
[entity ahead]
id = 2
model = none
position = 500, 0, 0
[entity behind]
id = 3
model = none
position = -500, 0, 0
[entity far]
id = 4
model = none
position = 2000, 0, 0
[entity off_axis]
id = 5
model = none
position = 300, 250, 0
[sensor radar]
id = 100
host = 1
model = radar_sensor
range_m = 1000
fov_deg = 90
update_hz = 10
)"));
    h.run();
    std::set<uint32_t> seen;
    for (const auto& d : h.detections) {
        seen.insert(d.target_id);
        if (d.target_id == 2) {
            EXPECT_NEAR(d.range_m, 500.0, 1e-9);
            EXPECT_NEAR(d.bearing_rad, 0.0, 1e-12);
        }
    }
    EXPECT_EQ(seen, (std::set<uint32_t>{2, 5}));
    EXPECT_EQ(h.detections.size(), 2u * 10u);  // two targets, ten scans
}

TEST(Engine, SameSeedIsBitForBitReproducible) {
    Harness a(coastal("kinematic", 7));
    Harness b(coastal("kinematic", 7));
    Harness c(coastal("kinematic", 8));
    a.run();
    b.run();
    c.run();
    ASSERT_EQ(a.detections.size(), b.detections.size());
    for (std::size_t i = 0; i < a.detections.size(); ++i) {
        EXPECT_EQ(a.detections[i].range_m, b.detections[i].range_m);
        EXPECT_EQ(a.detections[i].bearing_rad, b.detections[i].bearing_rad);
    }
    bool any_diff = false;
    for (std::size_t i = 0; i < std::min(a.detections.size(), c.detections.size()); ++i) {
        any_diff |= a.detections[i].range_m != c.detections[i].range_m;
    }
    EXPECT_TRUE(any_diff) << "a different seed should change sensor noise";
}

// Closed loop parity: the full scenario on both backends. Discrete events
// (a waypoint counting as reached) can land one frame apart from rounding at
// the 1e-13 level, so this allows one frame of travel (25 m/s x 0.05 s) per
// waypoint switch rather than demanding bit equality; the open loop test in
// test_backends.cpp is the strict one.
TEST(Engine, ClosedLoopParityAcrossBackends) {
    Harness modern(coastal("kinematic"));
    Harness legacy(coastal("legacy_blocks"));
    const RunStats a = modern.run();
    const RunStats b = legacy.run();
    EXPECT_EQ(a.steps, b.steps);
    for (uint32_t id : {1u, 2u, 3u}) {
        const auto p = modern.state(id).pos, q = legacy.state(id).pos;
        EXPECT_LT(std::hypot(p.x - q.x, p.y - q.y), 5.0) << "entity " << id;
    }
    const double ratio = static_cast<double>(b.detections) / static_cast<double>(a.detections);
    EXPECT_NEAR(ratio, 1.0, 0.02);
}

TEST(Engine, LateJoinerReceivesLatestStateOfEveryEntity) {
    Harness h(coastal("kinematic"));
    h.run();
    std::map<uint32_t, EntityState> latest;
    h.bus.subscribe<EntityState>(topics::kEntityState, [&](const EntityState& s) { latest[s.id] = s; });
    EXPECT_EQ(latest.size(), 3u);
}

TEST(Engine, ExternalEntityFollowsCommandsPublishedOnTheBus) {
    Harness h(parse_scenario_string(R"(
[scenario]
duration_s = 10
timestep_s = 0.1
[entity drone]
id = 4
model = external
position = 0, 0, 50
speed_mps = 0
)"));
    // Stand in for an outside controller: command due north at 10 m/s every frame.
    for (int i = 0; i < 100; ++i) {
        VehicleCommand c;
        c.entity_id = 4;
        c.heading_rad = kPi / 2;
        c.speed_mps = 10.0;
        h.bus.publish(topics::kVehicleCommand, c);
        h.engine->step();
    }
    const EntityState s = h.state(4);
    EXPECT_GT(s.pos.y, 50.0);  // turned north and accelerated (limits apply)
    EXPECT_NEAR(s.speed_mps, 10.0, 1e-9);
}

TEST(Engine, UnknownModelTypeFailsFast) {
    Scenario sc = parse_scenario_string(
        "[scenario]\nduration_s = 1\ntimestep_s = 0.1\n[entity a]\nid = 1\nmodel = warp_drive\nposition = 0,0,0\n");
    EXPECT_THROW(Harness h(std::move(sc)), PluginError);
}
