// DdsBridge tests. Each test plays the role of an outside autonomy process by
// opening its own DDS participant and talking to the bridge only through the
// SimBridge DDS topics, never through the bus.
#include <gtest/gtest.h>

#include <dds/dds.h>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "simbridge/dds_bridge.hpp"
#include "simbridge/engine.hpp"
#include "simbridge_types.h"

using namespace simbridge;

namespace {

// A fresh domain per test keeps tests independent even if they share a host.
uint32_t next_domain() {
    static uint32_t d = 120;
    return d++;
}

// The "external" side: a participant with readers or writers on SimBridge topics.
class Peer {
public:
    explicit Peer(uint32_t domain) {
        pp_ = dds_create_participant(domain, nullptr, nullptr);
        EXPECT_GT(pp_, 0);
    }
    ~Peer() { dds_delete(pp_); }

    dds_entity_t state_reader() {
        dds_qos_t* q = dds_create_qos();
        dds_qset_reliability(q, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
        dds_qset_durability(q, DDS_DURABILITY_TRANSIENT_LOCAL);
        dds_qset_history(q, DDS_HISTORY_KEEP_LAST, 1);
        const dds_entity_t t =
            dds_create_topic(pp_, &simbridge_dds_EntityState_desc, dds_topics::kEntityState, nullptr, nullptr);
        const dds_entity_t r = dds_create_reader(pp_, t, q, nullptr);
        dds_delete_qos(q);
        return r;
    }

    dds_entity_t detection_reader() {
        dds_qos_t* q = dds_create_qos();
        dds_qset_reliability(q, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
        dds_qset_history(q, DDS_HISTORY_KEEP_ALL, 0);
        const dds_entity_t t =
            dds_create_topic(pp_, &simbridge_dds_Detection_desc, dds_topics::kDetection, nullptr, nullptr);
        const dds_entity_t r = dds_create_reader(pp_, t, q, nullptr);
        dds_delete_qos(q);
        return r;
    }

    dds_entity_t command_writer() {
        dds_qos_t* q = dds_create_qos();
        dds_qset_reliability(q, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
        const dds_entity_t t = dds_create_topic(pp_, &simbridge_dds_VehicleCommand_desc,
                                                dds_topics::kVehicleCommand, nullptr, nullptr);
        const dds_entity_t w = dds_create_writer(pp_, t, q, nullptr);
        dds_delete_qos(q);
        return w;
    }

private:
    dds_entity_t pp_ = 0;
};

// Read (not take) every valid sample a reader holds, waiting up to 2 s for `want`.
// `copy` must deep copy: sample memory is loaned and goes back to DDS before returning.
template <typename Sample, typename Out, typename Copy>
std::vector<Out> read_all(dds_entity_t reader, std::size_t want, Copy copy) {
    std::vector<Out> out;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        out.clear();
        void* samples[64] = {};
        dds_sample_info_t infos[64];
        const int n = dds_read(reader, samples, infos, 64, 64);
        for (int i = 0; i < n; ++i) {
            if (infos[i].valid_data) out.push_back(copy(*static_cast<const Sample*>(samples[i])));
        }
        if (n > 0) dds_return_loan(reader, samples, n);
        if (out.size() >= want) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return out;
}

std::vector<EntityState> read_states(dds_entity_t reader, std::size_t want) {
    return read_all<simbridge_dds_EntityState, EntityState>(reader, want, [](const simbridge_dds_EntityState& m) {
        EntityState s;
        s.id = m.id;
        s.name = m.name ? m.name : "";
        s.t = m.t;
        s.pos = Vec3{m.pos.x, m.pos.y, m.pos.z};
        s.vel = Vec3{m.vel.x, m.vel.y, m.vel.z};
        s.heading_rad = m.heading_rad;
        s.speed_mps = m.speed_mps;
        return s;
    });
}

std::vector<Detection> read_detections(dds_entity_t reader, std::size_t want) {
    return read_all<simbridge_dds_Detection, Detection>(reader, want, [](const simbridge_dds_Detection& m) {
        Detection d;
        d.sensor_id = m.sensor_id;
        d.host_id = m.host_id;
        d.target_id = m.target_id;
        d.t = m.t;
        d.range_m = m.range_m;
        d.bearing_rad = m.bearing_rad;
        return d;
    });
}

void send(dds_entity_t writer, uint32_t id, double heading, double speed) {
    simbridge_dds_VehicleCommand c{};
    c.entity_id = id;
    c.heading_rad = heading;
    c.speed_mps = speed;
    ASSERT_EQ(dds_write(writer, &c), DDS_RETCODE_OK);
}

std::size_t poll_until(DdsBridge& bridge, std::size_t want) {
    std::size_t got = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (got < want && std::chrono::steady_clock::now() < deadline) {
        got += bridge.poll_commands();
        if (got < want) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return got;
}

}  // namespace

TEST(DdsBridge, PublishesEntityStateWithAllFields) {
    const uint32_t domain = next_domain();
    MessageBus bus;
    DdsBridgeOptions opts;
    opts.domain_id = domain;
    DdsBridge bridge(bus, opts);
    Peer peer(domain);
    const dds_entity_t reader = peer.state_reader();

    EntityState s;
    s.id = 7;
    s.name = "uav_alpha";
    s.t = 3.5;
    s.pos = Vec3{10.0, -20.0, 120.0};
    s.vel = Vec3{25.0, 0.5, 0.0};
    s.heading_rad = 0.02;
    s.speed_mps = 25.005;
    bus.publish(topics::kEntityState, s);

    const auto got = read_states(reader, 1);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].id, 7u);
    EXPECT_EQ(got[0].name, "uav_alpha");
    EXPECT_EQ(got[0].t, 3.5);
    EXPECT_EQ(got[0].pos.y, -20.0);
    EXPECT_EQ(got[0].vel.x, 25.0);
    EXPECT_EQ(got[0].heading_rad, 0.02);
    EXPECT_EQ(got[0].speed_mps, 25.005);
    EXPECT_EQ(bridge.stats().states_written, 1u);
}

TEST(DdsBridge, LateJoinerGetsLatestStateOfEveryEntity) {
    const uint32_t domain = next_domain();
    MessageBus bus;
    DdsBridgeOptions opts;
    opts.domain_id = domain;
    DdsBridge bridge(bus, opts);

    // Three entities, several updates each, all written before any reader exists.
    for (int step = 1; step <= 5; ++step) {
        for (uint32_t id = 1; id <= 3; ++id) {
            EntityState s;
            s.id = id;
            s.name = "e" + std::to_string(id);
            s.t = step;
            bus.publish(topics::kEntityState, s);
        }
    }

    Peer late(domain);
    const auto got = read_states(late.state_reader(), 3);
    ASSERT_EQ(got.size(), 3u);  // one instance per entity id, not 15 samples
    std::map<uint32_t, double> latest;
    for (const auto& s : got) latest[s.id] = s.t;
    EXPECT_EQ(latest, (std::map<uint32_t, double>{{1, 5.0}, {2, 5.0}, {3, 5.0}}));
}

TEST(DdsBridge, PublishesDetections) {
    const uint32_t domain = next_domain();
    MessageBus bus;
    DdsBridgeOptions opts;
    opts.domain_id = domain;
    DdsBridge bridge(bus, opts);
    Peer peer(domain);
    const dds_entity_t reader = peer.detection_reader();

    for (uint32_t target = 1; target <= 4; ++target) {
        Detection d;
        d.sensor_id = 100;
        d.host_id = 9;
        d.target_id = target;
        d.range_m = 100.0 * target;
        bus.publish(topics::kDetection, d);
    }
    const auto got = read_detections(reader, 4);
    ASSERT_EQ(got.size(), 4u);
    EXPECT_EQ(got[3].target_id, 4u);
    EXPECT_EQ(got[3].range_m, 400.0);
    EXPECT_EQ(bridge.stats().detections_written, 4u);
}

TEST(DdsBridge, DeliversExternalCommandsOnlyWhenPolled) {
    const uint32_t domain = next_domain();
    MessageBus bus;
    std::vector<VehicleCommand> received;
    bus.subscribe<VehicleCommand>(topics::kVehicleCommand, [&](const VehicleCommand& c) { received.push_back(c); });
    DdsBridgeOptions opts;
    opts.domain_id = domain;
    DdsBridge bridge(bus, opts);
    Peer autonomy(domain);
    const dds_entity_t writer = autonomy.command_writer();

    send(writer, 11, 1.25, 8.0);
    send(writer, 12, -0.5, 3.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(received.empty()) << "commands must wait for poll_commands()";

    EXPECT_EQ(poll_until(bridge, 2), 2u);
    ASSERT_EQ(received.size(), 2u);
    std::map<uint32_t, VehicleCommand> by_id;
    for (const auto& c : received) by_id[c.entity_id] = c;
    EXPECT_EQ(by_id.at(11).heading_rad, 1.25);
    EXPECT_EQ(by_id.at(12).speed_mps, 3.0);
    EXPECT_EQ(bridge.stats().commands_received, 2u);
    EXPECT_EQ(bridge.poll_commands(), 0u);  // nothing is delivered twice
}

TEST(DdsBridge, OptionsDisableDirections) {
    const uint32_t domain = next_domain();
    MessageBus bus;
    DdsBridgeOptions opts;
    opts.domain_id = domain;
    opts.publish_state = false;
    opts.accept_commands = false;
    DdsBridge bridge(bus, opts);
    EXPECT_EQ(bus.subscriber_count(topics::kEntityState), 0u);
    EXPECT_EQ(bus.subscriber_count(topics::kDetection), 1u);
    EXPECT_EQ(bridge.poll_commands(), 0u);
}

TEST(DdsBridge, UnsubscribesFromBusOnDestruction) {
    MessageBus bus;
    {
        DdsBridgeOptions opts;
        opts.domain_id = next_domain();
        DdsBridge bridge(bus, opts);
        EXPECT_EQ(bus.subscriber_count(topics::kEntityState), 1u);
    }
    EXPECT_EQ(bus.subscriber_count(topics::kEntityState), 0u);
    EXPECT_EQ(bus.subscriber_count(topics::kDetection), 0u);
}

// Full loop: an outside controller reads simulation state over DDS and steers
// an "external" entity over DDS, with the simulation running in this process.
TEST(DdsBridge, ClosedLoopControlOfExternalEntityOverDds) {
    const uint32_t domain = next_domain();
    PluginRegistry plugins;
    MessageBus bus;
    Scenario sc = parse_scenario_string(R"(
[scenario]
duration_s = 60
timestep_s = 0.1
[entity target]
id = 1
model = none
position = 0, 500, 0
[entity chaser]
id = 2
model = external
position = 0, 0, 0
heading_deg = 0
speed_mps = 0
)");
    declare_standard_topics(bus, sc.entities.size());
    plugins.load_directory(SIMBRIDGE_PLUGIN_DIR);
    DdsBridgeOptions opts;
    opts.domain_id = domain;
    DdsBridge bridge(bus, opts);
    SimEngine engine(sc, make_backend("kinematic"), plugins, bus);

    Peer autonomy(domain);
    const dds_entity_t states = autonomy.state_reader();
    const dds_entity_t commands = autonomy.command_writer();

    for (int frame = 0; frame < 600; ++frame) {
        engine.step();
        // Controller: steer the chaser straight at the target, slowing as it closes in.
        const auto world = read_states(states, 2);
        ASSERT_EQ(world.size(), 2u) << "frame " << frame;
        const EntityState* target = nullptr;
        const EntityState* chaser = nullptr;
        for (const auto& s : world) (s.id == 1 ? target : chaser) = &s;
        const double dx = target->pos.x - chaser->pos.x, dy = target->pos.y - chaser->pos.y;
        send(commands, 2, std::atan2(dy, dx), std::min(15.0, 0.25 * std::hypot(dx, dy)));
        poll_until(bridge, 1);
    }

    EntityState chaser;
    for (const auto& s : engine.backend().states())
        if (s.id == 2) chaser = s;
    EXPECT_LT(std::hypot(chaser.pos.x - 0.0, chaser.pos.y - 500.0), 25.0)
        << "chaser ended at (" << chaser.pos.x << ", " << chaser.pos.y << ")";
    EXPECT_EQ(bridge.stats().commands_received, 600u);
}
