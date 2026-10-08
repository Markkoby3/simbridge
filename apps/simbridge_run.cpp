// simbridge_run: run a scenario from the command line.
//
//   simbridge_run <scenario.scn> [--plugins DIR] [--backend kinematic|legacy_blocks]
//                 [--udp HOST:PORT] [--csv FILE] [--realtime]
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "simbridge/backend.hpp"
#include "simbridge/engine.hpp"
#include "simbridge/message_bus.hpp"
#include "simbridge/plugin_registry.hpp"
#include "simbridge/scenario.hpp"
#include "simbridge/udp_bridge.hpp"

#ifndef SIMBRIDGE_DEFAULT_PLUGIN_DIR
#define SIMBRIDGE_DEFAULT_PLUGIN_DIR "plugins"
#endif

namespace {

int usage() {
    std::cerr << "usage: simbridge_run <scenario.scn> [--plugins DIR] [--backend kinematic|legacy_blocks]\n"
                 "                     [--udp HOST:PORT] [--csv FILE] [--realtime]\n";
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace simbridge;
    if (argc < 2) return usage();

    std::string scenario_path = argv[1];
    std::string plugin_dir = SIMBRIDGE_DEFAULT_PLUGIN_DIR;
    std::string backend_override, udp_target, csv_path;
    bool realtime = false;

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument(arg + " needs a value");
            return argv[++i];
        };
        try {
            if (arg == "--plugins") plugin_dir = value();
            else if (arg == "--backend") backend_override = value();
            else if (arg == "--udp") udp_target = value();
            else if (arg == "--csv") csv_path = value();
            else if (arg == "--realtime") realtime = true;
            else return usage();
        } catch (const std::exception& e) {
            std::cerr << e.what() << "\n";
            return usage();
        }
    }

    try {
        // Order matters: the registry must outlive the bus and the engine.
        PluginRegistry plugins;
        Scenario scenario = load_scenario(scenario_path);
        if (!backend_override.empty()) scenario.backend = backend_override;

        MessageBus bus;
        declare_standard_topics(bus, scenario.entities.size());
        plugins.load_directory(plugin_dir);

        std::unique_ptr<UdpBridge> udp;
        if (!udp_target.empty()) {
            const auto colon = udp_target.rfind(':');
            if (colon == std::string::npos) throw std::invalid_argument("--udp expects HOST:PORT");
            udp = std::make_unique<UdpBridge>(bus, udp_target.substr(0, colon),
                                              static_cast<uint16_t>(std::stoi(udp_target.substr(colon + 1))));
        }

        std::ofstream csv;
        if (!csv_path.empty()) {
            csv.open(csv_path);
            if (!csv) throw std::runtime_error("cannot write " + csv_path);
            csv << "t,id,name,x,y,z,heading_rad,speed_mps\n" << std::setprecision(10);
            bus.subscribe<EntityState>(topics::kEntityState, [&csv](const EntityState& s) {
                csv << s.t << ',' << s.id << ',' << s.name << ',' << s.pos.x << ',' << s.pos.y << ',' << s.pos.z << ','
                    << s.heading_rad << ',' << s.speed_mps << '\n';
            });
        }

        SimEngine engine(scenario, make_backend(scenario.backend), plugins, bus);
        std::cout << "scenario  " << scenario.name << "  backend " << engine.backend().name() << "  entities "
                  << scenario.entities.size() << "  sensors " << scenario.sensors.size() << "\n";

        const auto start = std::chrono::steady_clock::now();
        const RunStats stats = engine.run([&](const RunStats& s) {
            if (realtime) {
                std::this_thread::sleep_until(start + std::chrono::duration<double>(s.sim_time_s));
            }
            return true;
        });

        std::cout << std::fixed << std::setprecision(3) << "steps " << stats.steps << "  sim " << stats.sim_time_s
                  << " s  wall " << stats.wall_time_s << " s  ("
                  << (stats.wall_time_s > 0 ? stats.sim_time_s / stats.wall_time_s : 0.0) << "x real time)\n"
                  << "messages " << stats.messages << "  commands " << stats.commands << "  detections "
                  << stats.detections << "\n";
        if (udp) {
            std::cout << "udp packets " << udp->packets_sent() << "  bytes " << udp->bytes_sent() << "  errors "
                      << udp->send_errors() << "\n";
        }
        for (const auto& s : engine.backend().states()) {
            std::cout << std::setprecision(1) << "  " << s.name << "  pos (" << s.pos.x << ", " << s.pos.y << ", "
                      << s.pos.z << ")  speed " << s.speed_mps << " m/s\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
