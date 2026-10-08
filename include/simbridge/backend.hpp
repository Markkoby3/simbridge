// The backend adapter interface. Each simulation framework SimBridge can drive
// sits behind one ISimBackend implementation, which translates the
// framework's native API and conventions into SimBridge messages.
//
// Shipped adapters:
//   kinematic      native SI/ENU kinematics (the "new" framework)
//   legacy_blocks  wraps legacy::BlockSim, which uses feet, knots, compass
//                  degrees and a millisecond clock (the "legacy" framework)
//
// Both must produce the same trajectories for the same scenario; the parity
// tests enforce that, which is what makes migrating between them safe.
#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "simbridge/messages.hpp"

namespace simbridge {

inline constexpr double kPi = 3.14159265358979323846;

inline double wrap_pi(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0) a += 2.0 * kPi;
    return a - kPi;  // (-pi, pi]
}

struct KinematicLimits {
    double max_turn_rate_rps = 0.35;  // ~20 deg/s
    double max_accel_mps2 = 4.0;
};

class ISimBackend {
public:
    virtual ~ISimBackend() = default;
    virtual std::string name() const = 0;
    virtual void add_entity(uint32_t id, const std::string& name, const Vec3& pos, double heading_rad,
                            double speed_mps) = 0;
    virtual void command(const VehicleCommand& cmd) = 0;
    virtual void step(double dt_s) = 0;
    virtual double time() const = 0;
    virtual std::vector<EntityState> states() const = 0;  // sorted by id
};

std::unique_ptr<ISimBackend> make_kinematic_backend(KinematicLimits limits = {});
std::unique_ptr<ISimBackend> make_legacy_blocks_backend(KinematicLimits limits = {});
std::unique_ptr<ISimBackend> make_backend(const std::string& name, KinematicLimits limits = {});

}  // namespace simbridge
