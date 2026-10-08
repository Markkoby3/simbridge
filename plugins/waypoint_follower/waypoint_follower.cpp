// Plugin: steers its host entity through a list of waypoints.
//
// Scenario parameters
//   speed_mps         cruise speed (default: the entity's speed_mps, else 10)
//   waypoints         "x,y,z; x,y,z; ..." in ENU meters
//   arrival_radius_m  distance at which a waypoint counts as reached (default 25)
//   loop              "true" to cycle the route forever (default false)
//
// Publishes one VehicleCommand per frame. Holds position (speed 0) when the route ends.
#include <cmath>
#include <vector>

#include "simbridge/model.hpp"

namespace {

using namespace simbridge;

class WaypointFollower final : public IModel {
public:
    void configure(const ModelInit& init) override {
        id_ = init.self_id;
        speed_ = init.spec.get_double_or("speed_mps", 10.0);
        route_ = init.spec.get_vec3_list("waypoints");
        radius_ = init.spec.get_double_or("arrival_radius_m", 25.0);
        loop_ = init.spec.get_or("loop", "false") == "true";
    }

    void step(const ModelContext& ctx) override {
        if (!ctx.host) return;
        VehicleCommand cmd;
        cmd.entity_id = id_;
        cmd.t = ctx.t;
        cmd.heading_rad = ctx.host->heading_rad;

        // Advance past every waypoint already inside the arrival radius.
        // Bounded so a looping route whose points all sit inside the radius cannot spin forever.
        for (std::size_t guard = 0;
             guard < route_.size() && next_ < route_.size() && distance(*ctx.host, route_[next_]) <= radius_;
             ++guard) {
            ++next_;
            if (loop_ && next_ == route_.size()) next_ = 0;
        }
        if (next_ >= route_.size()) {
            cmd.speed_mps = 0.0;  // route complete
        } else {
            const Vec3& wp = route_[next_];
            cmd.heading_rad = std::atan2(wp.y - ctx.host->pos.y, wp.x - ctx.host->pos.x);
            cmd.speed_mps = speed_;
        }
        ctx.bus.publish(topics::kVehicleCommand, cmd);
    }

private:
    static double distance(const EntityState& s, const Vec3& p) {
        return std::hypot(p.x - s.pos.x, p.y - s.pos.y);
    }

    uint32_t id_ = 0;
    double speed_ = 10.0;
    double radius_ = 25.0;
    bool loop_ = false;
    std::vector<Vec3> route_;
    std::size_t next_ = 0;
};

}  // namespace

SIMBRIDGE_DECLARE_PLUGIN("waypoint_follower", WaypointFollower)
