// Adapter: exposes legacy::BlockSim through the ISimBackend interface.
//
// Every conversion between the legacy conventions and SimBridge's lives here:
//   position  feet (north, east, alt)  <->  meters ENU (x = east, y = north, z = up)
//   speed     knots                    <->  meters per second
//   heading   compass degrees (cw from north)  <->  radians (ccw from east)
//   time      integer milliseconds     <->  seconds, with the fractional
//             remainder carried forward so odd timesteps do not drift
//   ids       sequential block ids     <->  scenario entity ids
#include <cmath>
#include <map>
#include <stdexcept>

#include "simbridge/backend.hpp"
#include "simbridge/legacy/block_sim.hpp"

namespace simbridge {

namespace {

constexpr double kMetersPerFoot = 0.3048;
constexpr double kMpsPerKnot = 1852.0 / 3600.0;

double enu_to_compass_deg(double heading_rad) {
    double d = 90.0 - heading_rad * 180.0 / kPi;
    d = std::fmod(d, 360.0);
    return d < 0 ? d + 360.0 : d;
}

double compass_deg_to_enu(double compass_deg) { return wrap_pi((90.0 - compass_deg) * kPi / 180.0); }

class LegacyBlocksAdapter final : public ISimBackend {
public:
    explicit LegacyBlocksAdapter(KinematicLimits limits)
        : sim_(limits.max_turn_rate_rps * 180.0 / kPi, limits.max_accel_mps2 / kMpsPerKnot) {}

    std::string name() const override { return "legacy_blocks"; }

    void add_entity(uint32_t id, const std::string& name, const Vec3& pos, double heading_rad,
                    double speed_mps) override {
        if (block_of_.count(id)) throw std::invalid_argument("entity id already exists: " + std::to_string(id));
        const int block = sim_.add_block(name, pos.y / kMetersPerFoot, pos.x / kMetersPerFoot, pos.z / kMetersPerFoot,
                                         enu_to_compass_deg(heading_rad), speed_mps / kMpsPerKnot);
        block_of_[id] = block;
        entity_of_[block] = id;
    }

    void command(const VehicleCommand& cmd) override {
        auto it = block_of_.find(cmd.entity_id);
        if (it == block_of_.end()) return;
        sim_.set_heading_deg(it->second, enu_to_compass_deg(cmd.heading_rad));
        sim_.set_speed_kts(it->second, cmd.speed_mps / kMpsPerKnot);
    }

    void step(double dt) override {
        remainder_ms_ += dt * 1000.0;
        const auto whole = static_cast<int64_t>(std::llround(remainder_ms_));
        remainder_ms_ -= static_cast<double>(whole);
        sim_.tick_ms(whole);
    }

    double time() const override { return static_cast<double>(sim_.clock_ms()) / 1000.0; }

    std::vector<EntityState> states() const override {
        std::map<uint32_t, EntityState> sorted;
        for (const auto& b : sim_.blocks()) {
            EntityState s;
            s.id = entity_of_.at(b.block_id);
            s.name = b.label;
            s.t = time();
            s.pos = Vec3{b.east_ft * kMetersPerFoot, b.north_ft * kMetersPerFoot, b.alt_ft * kMetersPerFoot};
            s.heading_rad = compass_deg_to_enu(b.heading_deg);
            s.speed_mps = b.speed_kts * kMpsPerKnot;
            s.vel = Vec3{s.speed_mps * std::cos(s.heading_rad), s.speed_mps * std::sin(s.heading_rad), 0.0};
            sorted[s.id] = s;
        }
        std::vector<EntityState> out;
        out.reserve(sorted.size());
        for (auto& entry : sorted) out.push_back(entry.second);
        return out;
    }

private:
    legacy::BlockSim sim_;
    std::map<uint32_t, int> block_of_;
    std::map<int, uint32_t> entity_of_;
    double remainder_ms_ = 0.0;
};

}  // namespace

std::unique_ptr<ISimBackend> make_legacy_blocks_backend(KinematicLimits limits) {
    return std::make_unique<LegacyBlocksAdapter>(limits);
}

}  // namespace simbridge
