// Test fixture: a plugin that claims an ABI version the host does not support.
// The registry must refuse to load it.
#include "simbridge/model.hpp"

namespace {
class Dummy final : public simbridge::IModel {
public:
    void configure(const simbridge::ModelInit&) override {}
    void step(const simbridge::ModelContext&) override {}
};
}  // namespace

SIMBRIDGE_EXPORT int simbridge_plugin_abi_version() { return 999; }
SIMBRIDGE_EXPORT const char* simbridge_plugin_type() { return "bad_abi"; }
SIMBRIDGE_EXPORT simbridge::IModel* simbridge_create_model() { return new Dummy(); }
SIMBRIDGE_EXPORT void simbridge_destroy_model(simbridge::IModel* m) { delete m; }
