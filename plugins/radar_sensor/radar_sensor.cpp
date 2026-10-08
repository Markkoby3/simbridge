// Plugin: a forward looking radar mounted on a host entity.
//
// Scenario parameters
//   range_m             maximum detection range (default 1000)
//   fov_deg             field of view centered on the host's heading (default 90)
//   update_hz           scan rate (default 10)
//   range_noise_m       1 sigma Gaussian range error (default 0)
//   bearing_noise_deg   1 sigma Gaussian bearing error (default 0)
//
// Publishes a Detection for every other entity inside range and field of view.
// Noise comes from a generator seeded by the scenario seed and the sensor id,
// so runs are exactly reproducible.
#include <cmath>
#include <random>

#include "simbridge/backend.hpp"
#include "simbridge/model.hpp"

namespace {

using namespace simbridge;

class RadarSensor final : public IModel {
public:
    void configure(const ModelInit& init) override {
        id_ = init.self_id;
        host_ = init.host_id;
        range_ = init.spec.get_double_or("range_m", 1000.0);
        half_fov_ = init.spec.get_double_or("fov_deg", 90.0) * kPi / 360.0;
        const double hz = init.spec.get_double_or("update_hz", 10.0);
        period_ = hz > 0 ? 1.0 / hz : 0.0;
        range_noise_ = init.spec.get_double_or("range_noise_m", 0.0);
        bearing_noise_ = init.spec.get_double_or("bearing_noise_deg", 0.0) * kPi / 180.0;
        rng_.seed(init.seed * 1000003ULL + id_);
    }

    void step(const ModelContext& ctx) override {
        if (!ctx.host) return;
        if (ctx.t + 1e-9 < next_scan_) return;
        next_scan_ = ctx.t + period_;

        for (const auto& target : ctx.world) {
            if (target.id == host_) continue;
            const double dx = target.pos.x - ctx.host->pos.x;
            const double dy = target.pos.y - ctx.host->pos.y;
            const double dz = target.pos.z - ctx.host->pos.z;
            const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
            const double bearing = wrap_pi(std::atan2(dy, dx) - ctx.host->heading_rad);
            if (range > range_ || std::abs(bearing) > half_fov_) continue;

            Detection d;
            d.sensor_id = id_;
            d.host_id = host_;
            d.target_id = target.id;
            d.t = ctx.t;
            d.range_m = range + (range_noise_ > 0 ? gauss_(rng_) * range_noise_ : 0.0);
            d.bearing_rad = wrap_pi(bearing + (bearing_noise_ > 0 ? gauss_(rng_) * bearing_noise_ : 0.0));
            ctx.bus.publish(topics::kDetection, d);
        }
    }

private:
    uint32_t id_ = 0, host_ = 0;
    double range_ = 1000.0, half_fov_ = kPi / 4, period_ = 0.1;
    double range_noise_ = 0.0, bearing_noise_ = 0.0;
    double next_scan_ = 0.0;
    std::mt19937_64 rng_;
    std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace

SIMBRIDGE_DECLARE_PLUGIN("radar_sensor", RadarSensor)
