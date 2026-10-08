#include <algorithm>
#include <map>
#include <stdexcept>

#include "simbridge/backend.hpp"

namespace simbridge {

namespace {

class KinematicBackend final : public ISimBackend {
public:
    explicit KinematicBackend(KinematicLimits limits) : limits_(limits) {}

    std::string name() const override { return "kinematic"; }

    void add_entity(uint32_t id, const std::string& name, const Vec3& pos, double heading_rad,
                    double speed_mps) override {
        if (bodies_.count(id)) throw std::invalid_argument("entity id already exists: " + std::to_string(id));
        Body b;
        b.name = name;
        b.pos = pos;
        b.heading = wrap_pi(heading_rad);
        b.speed = speed_mps;
        b.cmd_heading = b.heading;
        b.cmd_speed = speed_mps;
        bodies_.emplace(id, b);
    }

    void command(const VehicleCommand& cmd) override {
        auto it = bodies_.find(cmd.entity_id);
        if (it == bodies_.end()) return;  // commands for unknown entities are dropped
        it->second.cmd_heading = wrap_pi(cmd.heading_rad);
        it->second.cmd_speed = std::max(0.0, cmd.speed_mps);
    }

    void step(double dt) override {
        const double max_turn = limits_.max_turn_rate_rps * dt;
        const double max_dv = limits_.max_accel_mps2 * dt;
        for (auto& entry : bodies_) {
            Body& b = entry.second;
            const double err = wrap_pi(b.cmd_heading - b.heading);
            b.heading = wrap_pi(b.heading + std::clamp(err, -max_turn, max_turn));
            b.speed += std::clamp(b.cmd_speed - b.speed, -max_dv, max_dv);
            b.pos.x += b.speed * std::cos(b.heading) * dt;
            b.pos.y += b.speed * std::sin(b.heading) * dt;
        }
        t_ += dt;
    }

    double time() const override { return t_; }

    std::vector<EntityState> states() const override {
        std::vector<EntityState> out;
        out.reserve(bodies_.size());
        for (const auto& entry : bodies_) {
            const Body& b = entry.second;
            EntityState s;
            s.id = entry.first;
            s.name = b.name;
            s.t = t_;
            s.pos = b.pos;
            s.vel = Vec3{b.speed * std::cos(b.heading), b.speed * std::sin(b.heading), 0.0};
            s.heading_rad = b.heading;
            s.speed_mps = b.speed;
            out.push_back(s);
        }
        return out;
    }

private:
    struct Body {
        std::string name;
        Vec3 pos;
        double heading = 0, speed = 0, cmd_heading = 0, cmd_speed = 0;
    };
    KinematicLimits limits_;
    std::map<uint32_t, Body> bodies_;  // ordered, so states() is sorted by id
    double t_ = 0.0;
};

}  // namespace

std::unique_ptr<ISimBackend> make_kinematic_backend(KinematicLimits limits) {
    return std::make_unique<KinematicBackend>(limits);
}

std::unique_ptr<ISimBackend> make_backend(const std::string& name, KinematicLimits limits) {
    if (name == "kinematic") return make_kinematic_backend(limits);
    if (name == "legacy_blocks") return make_legacy_blocks_backend(limits);
    throw std::invalid_argument("unknown backend '" + name + "'");
}

}  // namespace simbridge
