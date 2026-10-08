#include "simbridge/scenario.hpp"

#include <fstream>
#include <set>

namespace simbridge {

namespace {

const std::set<std::string> kBackends = {"kinematic", "legacy_blocks"};

void validate(Scenario& sc, const ObjectSpec& header) {
    sc.name = header.get_or("name", sc.name);
    sc.duration_s = header.get_double("duration_s");
    sc.timestep_s = header.get_double("timestep_s");
    sc.backend = header.get_or("backend", sc.backend);
    sc.seed = static_cast<uint64_t>(header.get_double_or("seed", 1));

    if (sc.duration_s <= 0) throw ScenarioError("duration_s must be positive", header.line_of("duration_s"));
    if (sc.timestep_s <= 0 || sc.timestep_s > sc.duration_s) {
        throw ScenarioError("timestep_s must be positive and no larger than duration_s",
                            header.line_of("timestep_s"));
    }
    if (!kBackends.count(sc.backend)) {
        throw ScenarioError("unknown backend '" + sc.backend + "' (expected kinematic or legacy_blocks)",
                            header.line_of("backend"));
    }

    std::set<uint32_t> ids;
    std::set<uint32_t> entity_ids;
    auto claim = [&](const ObjectSpec& spec) {
        const uint32_t id = spec.get_id("id");
        if (!ids.insert(id).second) {
            throw ScenarioError("duplicate id " + std::to_string(id), spec.line_of("id"));
        }
        return id;
    };
    for (const auto& e : sc.entities) {
        entity_ids.insert(claim(e));
        e.get("model");
        e.get_vec3("position");
        e.get_vec3_list("waypoints");
    }
    for (const auto& s : sc.sensors) {
        claim(s);
        s.get("model");
        const uint32_t host = s.get_id("host");
        if (!entity_ids.count(host)) {
            throw ScenarioError("sensor '" + s.name + "' references unknown host " + std::to_string(host),
                                s.line_of("host"));
        }
    }
}

}  // namespace

Scenario parse_scenario(std::istream& in) {
    Scenario sc;
    ObjectSpec header;
    bool have_header = false;
    ObjectSpec* current = nullptr;

    std::string raw;
    int line_no = 0;
    while (std::getline(in, raw)) {
        ++line_no;
        const auto hash = raw.find('#');
        const std::string line = trim(hash == std::string::npos ? raw : raw.substr(0, hash));
        if (line.empty()) continue;

        if (line.front() == '[') {
            if (line.back() != ']') throw ScenarioError("unterminated section header", line_no);
            std::stringstream ss(line.substr(1, line.size() - 2));
            std::string kind, name, extra;
            ss >> kind >> name >> extra;
            if (!extra.empty()) throw ScenarioError("section names cannot contain spaces", line_no);
            if (kind == "scenario") {
                if (have_header) throw ScenarioError("duplicate [scenario] section", line_no);
                have_header = true;
                header.kind = "scenario";
                header.name = "scenario";
                header.line = line_no;
                current = &header;
            } else if (kind == "entity" || kind == "sensor") {
                if (name.empty()) throw ScenarioError("[" + kind + "] needs a name", line_no);
                auto& list = kind == "entity" ? sc.entities : sc.sensors;
                list.push_back(ObjectSpec{kind, name, line_no, {}});
                current = &list.back();
            } else {
                throw ScenarioError("unknown section [" + kind + "]", line_no);
            }
            continue;
        }

        if (!current) throw ScenarioError("key/value outside of any section", line_no);
        const auto eq = line.find('=');
        if (eq == std::string::npos) throw ScenarioError("expected key = value", line_no);
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        if (key.empty()) throw ScenarioError("empty key", line_no);
        if (current->params.count(key)) throw ScenarioError("duplicate key '" + key + "'", line_no);
        current->params[key] = {value, line_no};
    }

    if (!have_header) throw ScenarioError("missing [scenario] section", 0);
    validate(sc, header);
    return sc;
}

Scenario parse_scenario_string(const std::string& text) {
    std::istringstream in(text);
    return parse_scenario(in);
}

Scenario load_scenario(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw ScenarioError("cannot open scenario file '" + path + "'", 0);
    return parse_scenario(in);
}

}  // namespace simbridge
