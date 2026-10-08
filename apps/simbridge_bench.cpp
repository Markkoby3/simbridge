// simbridge_bench: measures bus throughput and engine step rate.
//
//   simbridge_bench [--plugins DIR]
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "simbridge/backend.hpp"
#include "simbridge/engine.hpp"
#include "simbridge/message_bus.hpp"
#include "simbridge/plugin_registry.hpp"
#include "simbridge/scenario.hpp"

#ifndef SIMBRIDGE_DEFAULT_PLUGIN_DIR
#define SIMBRIDGE_DEFAULT_PLUGIN_DIR "plugins"
#endif

using namespace simbridge;
using Clock = std::chrono::steady_clock;

namespace {

void bench_bus() {
    MessageBus bus;
    bus.declare<EntityState>("bench");
    uint64_t received = 0;
    constexpr int kSubscribers = 4;
    constexpr int kMessages = 1'000'000;
    for (int i = 0; i < kSubscribers; ++i) {
        bus.subscribe<EntityState>("bench", [&received](const EntityState&) { ++received; });
    }
    EntityState s;
    s.name = "bench_entity";
    const auto t0 = Clock::now();
    for (int i = 0; i < kMessages; ++i) {
        s.id = static_cast<uint32_t>(i);
        bus.publish("bench", s);
    }
    const double secs = std::chrono::duration<double>(Clock::now() - t0).count();
    std::cout << std::fixed << std::setprecision(2) << "bus: " << kMessages << " publishes x " << kSubscribers
              << " subscribers in " << secs << " s = " << (kMessages / secs / 1e6) << " M publishes/s, "
              << (static_cast<double>(received) / secs / 1e6) << " M deliveries/s\n";
}

std::string make_swarm_scenario(int vehicles, int radars) {
    std::ostringstream o;
    o << "[scenario]\nname = swarm\nduration_s = 50\ntimestep_s = 0.05\nseed = 7\n";
    for (int i = 0; i < vehicles; ++i) {
        const double x = (i % 20) * 100.0, y = (i / 20) * 100.0;
        o << "[entity v" << i << "]\nid = " << (i + 1) << "\nmodel = waypoint_follower\nposition = " << x << ", "
          << y << ", 100\nspeed_mps = 20\nwaypoints = " << (x + 800) << "," << (y + 400) << ",100; " << x << ","
          << y << ",100\nloop = true\n";
    }
    for (int i = 0; i < radars; ++i) {
        o << "[sensor r" << i << "]\nid = " << (10000 + i) << "\nhost = " << (i * (vehicles / radars) + 1)
          << "\nmodel = radar_sensor\nrange_m = 1500\nfov_deg = 120\nupdate_hz = 5\n";
    }
    return o.str();
}

void bench_engine(const std::string& plugin_dir, const std::string& backend) {
    PluginRegistry plugins;
    MessageBus bus;
    const Scenario sc = parse_scenario_string(make_swarm_scenario(200, 20));
    declare_standard_topics(bus, sc.entities.size());
    plugins.load_directory(plugin_dir);
    SimEngine engine(sc, make_backend(backend), plugins, bus);
    const RunStats st = engine.run();
    std::cout << std::fixed << std::setprecision(1) << "engine (" << backend << "): " << sc.entities.size()
              << " entities, " << sc.sensors.size() << " radars, " << st.steps << " steps in " << st.wall_time_s
              << " s = " << (static_cast<double>(st.steps) / st.wall_time_s) << " steps/s (" << (st.sim_time_s / st.wall_time_s)
              << "x real time), " << st.messages << " messages, " << st.detections << " detections\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string plugin_dir = SIMBRIDGE_DEFAULT_PLUGIN_DIR;
    if (argc == 3 && std::string(argv[1]) == "--plugins") plugin_dir = argv[2];
    try {
        bench_bus();
        bench_engine(plugin_dir, "kinematic");
        bench_engine(plugin_dir, "legacy_blocks");
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
