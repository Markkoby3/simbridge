// Scenario files: a small, line oriented format inspired by common scenario
// interchange formats. One [scenario] header, then [entity <name>] and
// [sensor <name>] sections with key = value pairs. '#' starts a comment.
//
//   [scenario]
//   name = coastal_patrol
//   duration_s = 120
//   timestep_s = 0.05
//   backend = kinematic          # or legacy_blocks
//   seed = 42
//
//   [entity uav1]
//   id = 1
//   model = waypoint_follower
//   position = 0, 0, 120
//   waypoints = 600,0,120; 600,600,120
//
// Every parse and validation error reports the line it came from.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <istream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "simbridge/messages.hpp"

namespace simbridge {

class ScenarioError : public std::runtime_error {
public:
    ScenarioError(const std::string& msg, int line)
        : std::runtime_error(line > 0 ? "line " + std::to_string(line) + ": " + msg : msg), line_(line) {}
    int line() const { return line_; }

private:
    int line_;
};

inline std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

inline double parse_number(const std::string& text, int line, const std::string& key) {
    const std::string t = trim(text);
    char* end = nullptr;
    const double v = std::strtod(t.c_str(), &end);
    if (t.empty() || end != t.c_str() + t.size()) {
        throw ScenarioError("'" + key + "' expects a number, got '" + t + "'", line);
    }
    return v;
}

inline Vec3 parse_vec3(const std::string& text, int line, const std::string& key) {
    std::vector<double> parts;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) parts.push_back(parse_number(item, line, key));
    if (parts.size() != 3) throw ScenarioError("'" + key + "' expects x, y, z", line);
    return Vec3{parts[0], parts[1], parts[2]};
}

using ParamMap = std::map<std::string, std::pair<std::string, int>>;  // key -> (value, line)

// One [entity ...] or [sensor ...] section, with typed accessors for plugins.
struct ObjectSpec {
    std::string kind;
    std::string name;
    int line = 0;
    ParamMap params;

    bool has(const std::string& key) const { return params.count(key) > 0; }

    std::string get(const std::string& key) const {
        auto it = params.find(key);
        if (it == params.end()) throw ScenarioError(kind + " '" + name + "' is missing '" + key + "'", line);
        return it->second.first;
    }
    std::string get_or(const std::string& key, const std::string& fallback) const {
        return has(key) ? get(key) : fallback;
    }
    // Line of a key's value. get() must run first: it throws a ScenarioError
    // for a missing key, where params.at() would throw std::out_of_range.
    int line_of(const std::string& key) const {
        auto it = params.find(key);
        return it == params.end() ? line : it->second.second;
    }

    double get_double(const std::string& key) const {
        const std::string value = get(key);
        return parse_number(value, line_of(key), key);
    }
    double get_double_or(const std::string& key, double fallback) const {
        return has(key) ? get_double(key) : fallback;
    }
    uint32_t get_id(const std::string& key) const {
        const double v = get_double(key);
        if (v < 1 || v > 4294967295.0 || v != static_cast<double>(static_cast<uint64_t>(v))) {
            throw ScenarioError("'" + key + "' must be a positive integer", line_of(key));
        }
        return static_cast<uint32_t>(v);
    }
    Vec3 get_vec3(const std::string& key) const {
        const std::string value = get(key);
        return parse_vec3(value, line_of(key), key);
    }
    // "x,y,z; x,y,z; ..."
    std::vector<Vec3> get_vec3_list(const std::string& key) const {
        std::vector<Vec3> out;
        if (!has(key)) return out;
        std::stringstream ss(get(key));
        std::string item;
        while (std::getline(ss, item, ';')) {
            if (!trim(item).empty()) out.push_back(parse_vec3(item, line_of(key), key));
        }
        return out;
    }
};

struct Scenario {
    std::string name = "unnamed";
    double duration_s = 0.0;
    double timestep_s = 0.0;
    std::string backend = "kinematic";
    uint64_t seed = 1;
    std::vector<ObjectSpec> entities;
    std::vector<ObjectSpec> sensors;
};

Scenario parse_scenario(std::istream& in);
Scenario parse_scenario_string(const std::string& text);
Scenario load_scenario(const std::string& path);

}  // namespace simbridge
