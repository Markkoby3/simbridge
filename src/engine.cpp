#include "simbridge/engine.hpp"

#include <chrono>
#include <cmath>
#include <unordered_map>

namespace simbridge {

void declare_standard_topics(MessageBus& bus, std::size_t entity_count) {
    // Latest state of every entity is kept for late joiners (e.g. a tool that attaches mid run).
    bus.declare<EntityState>(topics::kEntityState, TopicQos{entity_count});
    bus.declare<VehicleCommand>(topics::kVehicleCommand, TopicQos{0});
    bus.declare<Detection>(topics::kDetection, TopicQos{0});
}

SimEngine::SimEngine(Scenario scenario, std::unique_ptr<ISimBackend> backend, const PluginRegistry& plugins,
                     MessageBus& bus)
    : scenario_(std::move(scenario)), backend_(std::move(backend)), bus_(bus) {
    declare_standard_topics(bus_, scenario_.entities.size());

    for (const auto& e : scenario_.entities) {
        const uint32_t id = e.get_id("id");
        const double heading = e.get_double_or("heading_deg", 0.0) * kPi / 180.0;
        backend_->add_entity(id, e.name, e.get_vec3("position"), heading, e.get_double_or("speed_mps", 0.0));
        const std::string type = e.get("model");
        if (type == "none") continue;  // passive entity, moved only by its initial state
        Instance inst{plugins.create(type), id};
        inst.model->configure(ModelInit{e, id, id, scenario_.seed});
        models_.push_back(std::move(inst));
    }
    for (const auto& s : scenario_.sensors) {
        const uint32_t id = s.get_id("id");
        const uint32_t host = s.get_id("host");
        Instance inst{plugins.create(s.get("model")), host};
        inst.model->configure(ModelInit{s, id, host, scenario_.seed});
        models_.push_back(std::move(inst));
    }

    subs_.push_back(bus_.subscribe<VehicleCommand>(topics::kVehicleCommand, [this](const VehicleCommand& c) {
        backend_->command(c);
        ++stats_.commands;
    }));
    subs_.push_back(
        bus_.subscribe<Detection>(topics::kDetection, [this](const Detection&) { ++stats_.detections; }));
    bus_baseline_ = bus_.published_count();
}

SimEngine::~SimEngine() {
    for (auto id : subs_) bus_.unsubscribe(id);
}

void SimEngine::step() {
    const double t = backend_->time();
    const double dt = scenario_.timestep_s;
    const std::vector<EntityState> world = backend_->states();

    std::unordered_map<uint32_t, const EntityState*> by_id;
    by_id.reserve(world.size());
    for (const auto& s : world) {
        by_id[s.id] = &s;
        bus_.publish(topics::kEntityState, s);
    }
    for (auto& inst : models_) {
        auto it = by_id.find(inst.host_id);
        ModelContext ctx{t, dt, it == by_id.end() ? nullptr : it->second, world, bus_};
        inst.model->step(ctx);
    }
    backend_->step(dt);

    ++stats_.steps;
    stats_.sim_time_s = backend_->time();
    stats_.messages = bus_.published_count() - bus_baseline_;
}

RunStats SimEngine::run(const std::function<bool(const RunStats&)>& on_step) {
    const auto total = static_cast<uint64_t>(std::llround(scenario_.duration_s / scenario_.timestep_s));
    const auto start = std::chrono::steady_clock::now();
    while (stats_.steps < total) {
        step();
        if (on_step && !on_step(stats_)) break;
    }
    stats_.wall_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return stats_;
}

}  // namespace simbridge
