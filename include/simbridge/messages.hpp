// Message types exchanged on the SimBridge bus.
//
// Every backend and plugin speaks these types, in one convention:
//   * SI units (meters, meters per second, seconds)
//   * ENU frame: x = east, y = north, z = up
//   * heading in radians, counterclockwise from +x (east), wrapped to (-pi, pi]
//
// Backend adapters are responsible for converting their native conventions
// into this one, so autonomy code never sees simulator specific units.
#pragma once

#include <cstdint>
#include <string>

namespace simbridge {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Ground truth state of one simulated entity at time t.
struct EntityState {
    uint32_t id = 0;
    std::string name;
    double t = 0.0;
    Vec3 pos;
    Vec3 vel;
    double heading_rad = 0.0;
    double speed_mps = 0.0;
};

// A steering request from a behavior model to the simulation backend.
struct VehicleCommand {
    uint32_t entity_id = 0;
    double t = 0.0;
    double heading_rad = 0.0;
    double speed_mps = 0.0;
};

// A sensor report: target seen at a range and a bearing relative to the host's heading.
struct Detection {
    uint32_t sensor_id = 0;
    uint32_t host_id = 0;
    uint32_t target_id = 0;
    double t = 0.0;
    double range_m = 0.0;
    double bearing_rad = 0.0;
};

namespace topics {
inline constexpr const char* kEntityState = "entity_state";
inline constexpr const char* kVehicleCommand = "vehicle_command";
inline constexpr const char* kDetection = "detection";
}  // namespace topics

}  // namespace simbridge
