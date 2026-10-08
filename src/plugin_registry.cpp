#include "simbridge/plugin_registry.hpp"

#include <dlfcn.h>

#include <algorithm>
#include <filesystem>

namespace simbridge {

namespace {

template <typename Fn>
Fn symbol(void* handle, const char* name, const std::string& path) {
    dlerror();
    void* sym = dlsym(handle, name);
    if (const char* err = dlerror()) {
        throw PluginError("plugin '" + path + "' is missing symbol " + name + ": " + err);
    }
    return reinterpret_cast<Fn>(sym);
}

}  // namespace

PluginRegistry::~PluginRegistry() {
    for (auto& entry : plugins_) dlclose(entry.second.handle);
}

std::string PluginRegistry::load(const std::string& path) {
    void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle) throw PluginError("cannot load plugin '" + path + "': " + dlerror());

    try {
        auto abi = symbol<int (*)()>(handle, "simbridge_plugin_abi_version", path);
        if (abi() != kPluginAbiVersion) {
            throw PluginError("plugin '" + path + "' was built for ABI version " + std::to_string(abi()) +
                              ", host expects " + std::to_string(kPluginAbiVersion));
        }
        auto type_fn = symbol<const char* (*)()>(handle, "simbridge_plugin_type", path);
        Entry entry;
        entry.handle = handle;
        entry.create = symbol<IModel* (*)()>(handle, "simbridge_create_model", path);
        entry.destroy = symbol<void (*)(IModel*)>(handle, "simbridge_destroy_model", path);
        entry.path = path;

        const std::string type = type_fn();
        if (plugins_.count(type)) {
            throw PluginError("model type '" + type + "' from '" + path + "' is already provided by '" +
                              plugins_.at(type).path + "'");
        }
        plugins_.emplace(type, entry);
        return type;
    } catch (...) {
        dlclose(handle);
        throw;
    }
}

std::size_t PluginRegistry::load_directory(const std::string& dir) {
    namespace fs = std::filesystem;
    if (!fs::is_directory(dir)) throw PluginError("plugin directory '" + dir + "' does not exist");
    std::vector<std::string> files;
    for (const auto& item : fs::directory_iterator(dir)) {
        if (item.is_regular_file() && item.path().extension() == ".so") files.push_back(item.path().string());
    }
    std::sort(files.begin(), files.end());  // deterministic load order
    for (const auto& f : files) load(f);
    return files.size();
}

std::vector<std::string> PluginRegistry::types() const {
    std::vector<std::string> out;
    for (const auto& entry : plugins_) out.push_back(entry.first);
    return out;
}

ModelPtr PluginRegistry::create(const std::string& type) const {
    auto it = plugins_.find(type);
    if (it == plugins_.end()) throw PluginError("no plugin provides model type '" + type + "'");
    IModel* raw = it->second.create();
    if (!raw) throw PluginError("plugin for '" + type + "' returned a null model");
    return ModelPtr(raw, it->second.destroy);
}

}  // namespace simbridge
