// simbridge_autonomy: an example autonomy process that controls a simulated
// vehicle purely over DDS. It never links the simulation; it only shares the
// IDL types and topic names.
//
// It reads SimBridge_EntityState, steers the interceptor toward where the
// target will be (lead pursuit), and writes SimBridge_VehicleCommand until the
// target is inside the capture radius.
//
//   simbridge_autonomy [--domain N] [--interceptor ID] [--target ID]
//                      [--speed M_PER_S] [--capture M] [--timeout S]
//
// Exit code 0 on capture, 1 on timeout, 2 on bad arguments.
#include <dds/dds.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <thread>

#include "simbridge/dds_bridge.hpp"
#include "simbridge_types.h"

namespace {

struct Options {
    uint32_t domain = 0;
    uint32_t interceptor = 10;
    uint32_t target = 1;
    double speed = 45.0;
    double capture_m = 30.0;
    double timeout_s = 120.0;
};

struct Track {
    double t, x, y, vx, vy;
};

int usage() {
    std::fprintf(stderr,
                 "usage: simbridge_autonomy [--domain N] [--interceptor ID] [--target ID]\n"
                 "                          [--speed M_PER_S] [--capture M] [--timeout S]\n");
    return 2;
}

// Aim point for lead pursuit: where the target will be after the time it takes
// to close the current distance, capped so a noisy velocity cannot send us far off.
void aim_point(const Track& me, const Track& tgt, double speed, double& ax, double& ay) {
    const double dist = std::hypot(tgt.x - me.x, tgt.y - me.y);
    const double t_go = std::min(dist / std::max(speed, 1.0), 8.0);
    ax = tgt.x + tgt.vx * t_go;
    ay = tgt.y + tgt.vy * t_go;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (i + 1 >= argc) return usage();
        const char* v = argv[++i];
        if (a == "--domain") o.domain = static_cast<uint32_t>(std::strtoul(v, nullptr, 10));
        else if (a == "--interceptor") o.interceptor = static_cast<uint32_t>(std::strtoul(v, nullptr, 10));
        else if (a == "--target") o.target = static_cast<uint32_t>(std::strtoul(v, nullptr, 10));
        else if (a == "--speed") o.speed = std::strtod(v, nullptr);
        else if (a == "--capture") o.capture_m = std::strtod(v, nullptr);
        else if (a == "--timeout") o.timeout_s = std::strtod(v, nullptr);
        else return usage();
    }

    const dds_entity_t pp = dds_create_participant(o.domain, nullptr, nullptr);
    if (pp < 0) {
        std::fprintf(stderr, "error: create_participant: %s\n", dds_strretcode(-pp));
        return 1;
    }

    dds_qos_t* state_qos = dds_create_qos();
    dds_qset_reliability(state_qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    dds_qset_durability(state_qos, DDS_DURABILITY_TRANSIENT_LOCAL);
    dds_qset_history(state_qos, DDS_HISTORY_KEEP_LAST, 1);
    const dds_entity_t state_topic = dds_create_topic(pp, &simbridge_dds_EntityState_desc,
                                                      simbridge::dds_topics::kEntityState, nullptr, nullptr);
    const dds_entity_t reader = dds_create_reader(pp, state_topic, state_qos, nullptr);
    dds_delete_qos(state_qos);

    dds_qos_t* cmd_qos = dds_create_qos();
    dds_qset_reliability(cmd_qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    const dds_entity_t cmd_topic = dds_create_topic(pp, &simbridge_dds_VehicleCommand_desc,
                                                    simbridge::dds_topics::kVehicleCommand, nullptr, nullptr);
    const dds_entity_t writer = dds_create_writer(pp, cmd_topic, cmd_qos, nullptr);
    dds_delete_qos(cmd_qos);

    // Block on new data instead of spinning.
    const dds_entity_t waitset = dds_create_waitset(pp);
    const dds_entity_t ready = dds_create_readcondition(reader, DDS_NOT_READ_SAMPLE_STATE);
    dds_waitset_attach(waitset, ready, 0);

    std::printf("autonomy: domain %u, interceptor %u, target %u, speed %.0f m/s, capture %.0f m\n", o.domain,
                o.interceptor, o.target, o.speed, o.capture_m);
    std::fflush(stdout);

    std::map<uint32_t, Track> tracks;
    const auto start = std::chrono::steady_clock::now();
    bool announced = false;
    int code = 1;
    uint64_t commands = 0;

    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < o.timeout_s) {
        dds_waitset_wait(waitset, nullptr, 0, DDS_MSECS(200));

        void* samples[16] = {};
        dds_sample_info_t infos[16];
        const int n = dds_take(reader, samples, infos, 16, 16);
        for (int i = 0; i < n; ++i) {
            if (!infos[i].valid_data) continue;
            const auto* s = static_cast<const simbridge_dds_EntityState*>(samples[i]);
            tracks[s->id] = Track{s->t, s->pos.x, s->pos.y, s->vel.x, s->vel.y};
        }
        if (n > 0) dds_return_loan(reader, samples, n);

        auto me = tracks.find(o.interceptor), tgt = tracks.find(o.target);
        if (me == tracks.end() || tgt == tracks.end()) continue;
        if (!announced) {
            std::printf("autonomy: tracking target at t=%.2f s, range %.0f m\n", tgt->second.t,
                        std::hypot(tgt->second.x - me->second.x, tgt->second.y - me->second.y));
            std::fflush(stdout);
            announced = true;
        }

        const double range = std::hypot(tgt->second.x - me->second.x, tgt->second.y - me->second.y);
        if (range <= o.capture_m) {
            std::printf("autonomy: CAPTURE at t=%.2f s, range %.1f m, %llu commands sent\n", me->second.t, range,
                        static_cast<unsigned long long>(commands));
            code = 0;
            break;
        }

        double ax = 0, ay = 0;
        aim_point(me->second, tgt->second, o.speed, ax, ay);
        simbridge_dds_VehicleCommand cmd{};
        cmd.entity_id = o.interceptor;
        cmd.t = me->second.t;
        cmd.heading_rad = std::atan2(ay - me->second.y, ax - me->second.x);
        cmd.speed_mps = o.speed;
        if (dds_write(writer, &cmd) == DDS_RETCODE_OK) ++commands;
    }

    if (code != 0) std::printf("autonomy: timed out without capture (%llu commands sent)\n",
                               static_cast<unsigned long long>(commands));
    dds_delete(pp);
    return code;
}
