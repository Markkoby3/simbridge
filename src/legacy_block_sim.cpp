#include "simbridge/legacy/block_sim.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legacy {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kFtPerNm = 6076.11548556;  // feet per nautical mile

double wrap360(double d) {
    d = std::fmod(d, 360.0);
    return d < 0 ? d + 360.0 : d;
}
}  // namespace

BlockSim::BlockSim(double max_turn_deg_per_s, double max_accel_kts_per_s)
    : max_turn_dps_(max_turn_deg_per_s), max_accel_ktps_(max_accel_kts_per_s) {}

int BlockSim::add_block(const std::string& label, double north_ft, double east_ft, double alt_ft, double heading_deg,
                        double speed_kts) {
    BlockState b;
    b.block_id = static_cast<int>(blocks_.size()) + 1;
    b.label = label;
    b.north_ft = north_ft;
    b.east_ft = east_ft;
    b.alt_ft = alt_ft;
    b.heading_deg = wrap360(heading_deg);
    b.speed_kts = speed_kts;
    blocks_.push_back(b);
    commands_.push_back(Command{b.heading_deg, speed_kts});
    return b.block_id;
}

void BlockSim::set_heading_deg(int block_id, double heading_deg) {
    commands_.at(static_cast<std::size_t>(block_id - 1)).heading_deg = wrap360(heading_deg);
}

void BlockSim::set_speed_kts(int block_id, double speed_kts) {
    commands_.at(static_cast<std::size_t>(block_id - 1)).speed_kts = std::max(0.0, speed_kts);
}

void BlockSim::tick_ms(int64_t ms) {
    if (ms < 0) throw std::invalid_argument("tick_ms requires a non-negative duration");
    const double dt = static_cast<double>(ms) / 1000.0;
    const double max_turn = max_turn_dps_ * dt;
    const double max_dv = max_accel_ktps_ * dt;
    for (std::size_t i = 0; i < blocks_.size(); ++i) {
        BlockState& b = blocks_[i];
        const Command& c = commands_[i];
        double err = std::fmod(c.heading_deg - b.heading_deg + 540.0, 360.0) - 180.0;  // [-180, 180)
        b.heading_deg = wrap360(b.heading_deg + std::clamp(err, -max_turn, max_turn));
        b.speed_kts += std::clamp(c.speed_kts - b.speed_kts, -max_dv, max_dv);
        const double ft_per_s = b.speed_kts * kFtPerNm / 3600.0;
        const double h = b.heading_deg * kPi / 180.0;
        b.north_ft += ft_per_s * std::cos(h) * dt;
        b.east_ft += ft_per_s * std::sin(h) * dt;
    }
    clock_ms_ += ms;
}

}  // namespace legacy
