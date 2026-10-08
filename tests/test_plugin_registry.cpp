#include <gtest/gtest.h>

#include <string>

#include "simbridge/plugin_registry.hpp"

using namespace simbridge;

namespace {
const std::string kPlugins = SIMBRIDGE_PLUGIN_DIR;
const std::string kTestPlugins = SIMBRIDGE_TEST_PLUGIN_DIR;
}  // namespace

TEST(PluginRegistry, LoadsShippedPluginsFromDirectory) {
    PluginRegistry reg;
    EXPECT_EQ(reg.load_directory(kPlugins), 2u);
    EXPECT_TRUE(reg.has("waypoint_follower"));
    EXPECT_TRUE(reg.has("radar_sensor"));
    EXPECT_EQ(reg.types(), (std::vector<std::string>{"radar_sensor", "waypoint_follower"}));
}

TEST(PluginRegistry, CreatesIndependentInstances) {
    PluginRegistry reg;
    reg.load(kPlugins + "/radar_sensor.so");
    ModelPtr a = reg.create("radar_sensor");
    ModelPtr b = reg.create("radar_sensor");
    EXPECT_NE(a.get(), nullptr);
    EXPECT_NE(a.get(), b.get());
}

TEST(PluginRegistry, RejectsUnknownType) {
    PluginRegistry reg;
    EXPECT_THROW(reg.create("does_not_exist"), PluginError);
}

TEST(PluginRegistry, RejectsMissingFile) {
    PluginRegistry reg;
    EXPECT_THROW(reg.load(kPlugins + "/nope.so"), PluginError);
    EXPECT_THROW(reg.load_directory(kPlugins + "/no_such_dir"), PluginError);
}

TEST(PluginRegistry, RejectsAbiMismatch) {
    PluginRegistry reg;
    try {
        reg.load(kTestPlugins + "/bad_abi_plugin.so");
        FAIL() << "expected PluginError";
    } catch (const PluginError& e) {
        EXPECT_NE(std::string(e.what()).find("ABI version 999"), std::string::npos);
    }
    EXPECT_FALSE(reg.has("bad_abi"));
}

TEST(PluginRegistry, RejectsDuplicateType) {
    PluginRegistry reg;
    reg.load(kPlugins + "/radar_sensor.so");
    EXPECT_THROW(reg.load(kPlugins + "/radar_sensor.so"), PluginError);
}
