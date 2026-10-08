#include <gtest/gtest.h>

#include <string>

#include "simbridge/scenario.hpp"

using namespace simbridge;

namespace {

const char* kValid = R"(
# comment line
[scenario]
name = demo
duration_s = 10
timestep_s = 0.1
backend = legacy_blocks
seed = 9

[entity uav]
id = 1
model = waypoint_follower
position = 1, 2, 3   # trailing comment
waypoints = 10,0,0; 10,10,0

[sensor eo]
id = 50
host = 1
model = radar_sensor
)";

int error_line(const std::string& text) {
    try {
        parse_scenario_string(text);
    } catch (const ScenarioError& e) {
        return e.line();
    }
    return -1;
}

}  // namespace

TEST(Scenario, ParsesHeaderEntitiesAndSensors) {
    const Scenario sc = parse_scenario_string(kValid);
    EXPECT_EQ(sc.name, "demo");
    EXPECT_DOUBLE_EQ(sc.duration_s, 10.0);
    EXPECT_DOUBLE_EQ(sc.timestep_s, 0.1);
    EXPECT_EQ(sc.backend, "legacy_blocks");
    EXPECT_EQ(sc.seed, 9u);
    ASSERT_EQ(sc.entities.size(), 1u);
    ASSERT_EQ(sc.sensors.size(), 1u);

    const ObjectSpec& uav = sc.entities[0];
    EXPECT_EQ(uav.name, "uav");
    EXPECT_EQ(uav.get_id("id"), 1u);
    const Vec3 p = uav.get_vec3("position");
    EXPECT_DOUBLE_EQ(p.x, 1);
    EXPECT_DOUBLE_EQ(p.z, 3);
    const auto route = uav.get_vec3_list("waypoints");
    ASSERT_EQ(route.size(), 2u);
    EXPECT_DOUBLE_EQ(route[1].y, 10);
    EXPECT_EQ(sc.sensors[0].get_id("host"), 1u);
}

TEST(Scenario, ShippedScenarioLoads) {
    const Scenario sc = load_scenario(std::string(SIMBRIDGE_SCENARIO_DIR) + "/coastal_patrol.scn");
    EXPECT_EQ(sc.name, "coastal_patrol");
    EXPECT_EQ(sc.entities.size(), 3u);
    EXPECT_EQ(sc.sensors.size(), 1u);
}

TEST(Scenario, ReportsLineOfBadNumber) {
    EXPECT_EQ(error_line("[scenario]\nduration_s = ten\ntimestep_s = 1\n"), 2);
}

TEST(Scenario, ReportsMissingRequiredKey) {
    EXPECT_THROW(parse_scenario_string("[scenario]\nduration_s = 10\n"), ScenarioError);
    EXPECT_EQ(error_line("[scenario]\nduration_s = 10\ntimestep_s = 1\n[entity a]\nid = 1\nposition = 0,0,0\n"), 4);
}

TEST(Scenario, RejectsUnknownBackend) {
    EXPECT_EQ(error_line("[scenario]\nduration_s = 10\ntimestep_s = 1\nbackend = quantum\n"), 4);
}

TEST(Scenario, RejectsDuplicateIds) {
    const std::string text =
        "[scenario]\nduration_s = 10\ntimestep_s = 1\n"
        "[entity a]\nid = 1\nmodel = none\nposition = 0,0,0\n"
        "[entity b]\nid = 1\nmodel = none\nposition = 0,0,0\n";
    EXPECT_EQ(error_line(text), 9);
}

TEST(Scenario, RejectsSensorWithUnknownHost) {
    const std::string text =
        "[scenario]\nduration_s = 10\ntimestep_s = 1\n"
        "[sensor s]\nid = 5\nhost = 99\nmodel = radar_sensor\n";
    EXPECT_EQ(error_line(text), 6);
}

TEST(Scenario, RejectsMalformedStructure) {
    EXPECT_EQ(error_line("duration_s = 10\n"), 1);
    EXPECT_EQ(error_line("[scenario]\nduration_s 10\n"), 2);
    EXPECT_EQ(error_line("[scenario\n"), 1);
    EXPECT_EQ(error_line("[weather storm]\n"), 1);
    EXPECT_EQ(error_line("[scenario]\nduration_s = 1\nduration_s = 2\n"), 3);
    EXPECT_EQ(error_line("[scenario]\nduration_s = 10\ntimestep_s = 1\n[entity a]\nid = 1\nmodel = none\n"
                         "position = 1, 2\n"),
              7);
}

TEST(Scenario, RejectsNonPositiveTiming) {
    EXPECT_EQ(error_line("[scenario]\nduration_s = 0\ntimestep_s = 1\n"), 2);
    EXPECT_EQ(error_line("[scenario]\nduration_s = 5\ntimestep_s = 10\n"), 3);
}
