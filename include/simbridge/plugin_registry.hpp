// Loads model plugins from shared libraries and creates instances by type name.
// The registry must outlive every model it creates and the bus they publish on.
#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "simbridge/model.hpp"

namespace simbridge {

class PluginError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

using ModelPtr = std::unique_ptr<IModel, void (*)(IModel*)>;

class PluginRegistry {
public:
    PluginRegistry() = default;
    ~PluginRegistry();
    PluginRegistry(const PluginRegistry&) = delete;
    PluginRegistry& operator=(const PluginRegistry&) = delete;

    // Load one plugin library. Returns the model type it provides.
    std::string load(const std::string& path);

    // Load every .so file in a directory. Returns how many were loaded.
    std::size_t load_directory(const std::string& dir);

    bool has(const std::string& type) const { return plugins_.count(type) > 0; }
    std::vector<std::string> types() const;
    ModelPtr create(const std::string& type) const;

private:
    struct Entry {
        void* handle = nullptr;
        IModel* (*create)() = nullptr;
        void (*destroy)(IModel*) = nullptr;
        std::string path;
    };
    std::map<std::string, Entry> plugins_;
};

}  // namespace simbridge
