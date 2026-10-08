// The plugin interface. A model is a behavior (an autopilot, a sensor, an
// effect) compiled into its own shared library and loaded at runtime.
//
// Models never talk to a simulation backend directly. They read ground truth
// from the ModelContext and publish requests and reports on the bus. That keeps
// one plugin binary working against every backend adapter.
#pragma once

#include <cstdint>
#include <vector>

#include "simbridge/message_bus.hpp"
#include "simbridge/messages.hpp"
#include "simbridge/scenario.hpp"

namespace simbridge {

// Bump when IModel, ModelInit or ModelContext change layout. The loader
// refuses plugins built against a different version.
inline constexpr int kPluginAbiVersion = 1;

struct ModelInit {
    const ObjectSpec& spec;
    uint32_t self_id;
    uint32_t host_id;  // entity this model is attached to (itself, for entity models)
    uint64_t seed;
};

struct ModelContext {
    double t;
    double dt;
    const EntityState* host;  // nullptr if the host is not present this frame
    const std::vector<EntityState>& world;
    MessageBus& bus;
};

class IModel {
public:
    virtual ~IModel() = default;
    virtual void configure(const ModelInit& init) = 0;
    virtual void step(const ModelContext& ctx) = 0;
};

}  // namespace simbridge

#define SIMBRIDGE_EXPORT extern "C" __attribute__((visibility("default")))

// Place once in a plugin's .cpp file to export the factory symbols the loader expects.
#define SIMBRIDGE_DECLARE_PLUGIN(TYPE_NAME, CLASS_NAME)                                                \
    SIMBRIDGE_EXPORT int simbridge_plugin_abi_version() { return ::simbridge::kPluginAbiVersion; }    \
    SIMBRIDGE_EXPORT const char* simbridge_plugin_type() { return TYPE_NAME; }                        \
    SIMBRIDGE_EXPORT ::simbridge::IModel* simbridge_create_model() { return new CLASS_NAME(); }        \
    SIMBRIDGE_EXPORT void simbridge_destroy_model(::simbridge::IModel* m) { delete m; }
