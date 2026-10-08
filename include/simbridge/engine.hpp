// SimEngine runs a scenario: it owns the backend, instantiates one plugin model
// per entity and sensor, and drives a fixed timestep loop.
//
// Each frame at time t:
//   1. read ground truth from the backend and publish it on entity_state
//   2. step every model (models publish vehicle_command and detection)
//   3. vehicle_command samples are forwarded to the backend
//   4. advance the backend by dt
#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "simbridge/backend.hpp"
#include "simbridge/message_bus.hpp"
#include "simbridge/plugin_registry.hpp"
#include "simbridge/scenario.hpp"

namespace simbridge {

struct RunStats {
    uint64_t steps = 0;
    double sim_time_s = 0.0;
    double wall_time_s = 0.0;
    uint64_t messages = 0;
    uint64_t commands = 0;
    uint64_t detections = 0;
};

// Declare the standard topics with their QoS. Call before loading plugins.
void declare_standard_topics(MessageBus& bus, std::size_t entity_count);

class SimEngine {
public:
    SimEngine(Scenario scenario, std::unique_ptr<ISimBackend> backend, const PluginRegistry& plugins,
              MessageBus& bus);
    ~SimEngine();
    SimEngine(const SimEngine&) = delete;
    SimEngine& operator=(const SimEngine&) = delete;

    void step();

    // Run to the scenario's duration. on_step may return false to stop early.
    RunStats run(const std::function<bool(const RunStats&)>& on_step = {});

    const ISimBackend& backend() const { return *backend_; }
    const Scenario& scenario() const { return scenario_; }
    const RunStats& stats() const { return stats_; }

private:
    struct Instance {
        ModelPtr model;
        uint32_t host_id;
    };
    Scenario scenario_;
    std::unique_ptr<ISimBackend> backend_;
    MessageBus& bus_;
    std::vector<Instance> models_;
    std::vector<SubscriptionId> subs_;
    RunStats stats_;
    uint64_t bus_baseline_ = 0;
};

}  // namespace simbridge
