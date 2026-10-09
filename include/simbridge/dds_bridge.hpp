// DdsBridge connects the in-process MessageBus to a DDS domain (Eclipse Cyclone DDS),
// so autonomy software in other processes or on other machines can run
// against the simulation through standard DDS topics.
//
//   outbound  entity_state  -> DDS topic "SimBridge_EntityState"
//             detection     -> DDS topic "SimBridge_Detection"
//   inbound   DDS topic "SimBridge_VehicleCommand" -> vehicle_command
//
// QoS mirrors the bus: entity state is RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)
// per entity (key = id), so a reader that joins mid run immediately receives
// the latest state of every entity. Detections and commands are RELIABLE and
// VOLATILE.
//
// Inbound commands are not delivered on a DDS thread. They queue inside DDS
// until poll_commands() is called, which the simulation loop does once per
// frame. That keeps every backend call on the simulation thread and keeps
// runs deterministic for a given command arrival order.
//
// Cyclone DDS headers stay out of this file (pimpl), so code that includes it
// does not need DDS on its include path.
#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "simbridge/message_bus.hpp"

namespace simbridge {

struct DdsBridgeOptions {
    uint32_t domain_id = 0;
    bool publish_state = true;
    bool publish_detections = true;
    bool accept_commands = true;
};

struct DdsBridgeStats {
    uint64_t states_written = 0;
    uint64_t detections_written = 0;
    uint64_t commands_received = 0;
    uint64_t write_errors = 0;
};

class DdsError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

namespace dds_topics {
inline constexpr const char* kEntityState = "SimBridge_EntityState";
inline constexpr const char* kDetection = "SimBridge_Detection";
inline constexpr const char* kVehicleCommand = "SimBridge_VehicleCommand";
}  // namespace dds_topics

class DdsBridge {
public:
    DdsBridge(MessageBus& bus, DdsBridgeOptions options = {});
    ~DdsBridge();
    DdsBridge(const DdsBridge&) = delete;
    DdsBridge& operator=(const DdsBridge&) = delete;

    // Take every vehicle command that has arrived over DDS and publish it on
    // the bus. Returns the number of commands delivered.
    std::size_t poll_commands();

    DdsBridgeStats stats() const;
    uint32_t domain_id() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace simbridge
