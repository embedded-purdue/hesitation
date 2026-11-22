#include "config_loader.h"
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

// This file is the code that parses the static system parameters (things like efficiency, # of thrusters, etc.) from the JSON file into CPP.

using json = nlohmann::json; // Makes it so you can just do json::
namespace {
void validateConfig(const LoadedNEPSetup& setup) {
    const auto& f = setup.fluid;
    const auto& c = setup.config;

    if (f.m_flow <= 0.0 || f.cp <= 0.0) {
        throw std::runtime_error("Config error: Working fluid m_flow and cp must be > 0.");
    }
    if (c.reactor_efficiency <= 0.0 || c.reactor_efficiency > 1.0 || c.p_th_max <= 0.0) {
        throw std::runtime_error("Config error: Invalid CTE-1 reactor parameters.");
    }
    if (c.eta_b <= 0.0 || c.eta_b >= 1.0 || c.recuperator_eff < 0.0 || c.recuperator_eff > 1.0) {
        throw std::runtime_error("Config error: Invalid CTE-2 Brayton efficiencies.");
    }
    if (c.eta_pmad <= 0.0 || c.eta_pmad > 1.0 ||
        c.thruster_power_frac < 0.0 || c.thruster_power_frac > 1.0) {
        throw std::runtime_error("Config error: Invalid CTE-3 PMAD parameters.");
    }
    if (c.num_thrusters == 0 || c.num_thrusters > MAX_THRUSTERS) {
        throw std::runtime_error("Config error: num_thrusters must be between 1 and MAX_THRUSTERS.");
    };
}
} // namespace

LoadedNEPSetup loadNEPConfig(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open NEP config file: " + filepath);
    }

    json j;
    try {
        file >> j;
    } catch (const json::parse_error& e) {
        throw std::runtime_error(std::string("JSON parse error: ") + e.what());
    }

    LoadedNEPSetup setup{};

    // Use .value() with defaults so minor schema additions don't break older test configs
    const auto& jf = j.at("working_liquid");
    setup.fluid.m_flow = jf.value("m_flow_kgs", 100.0);
    setup.fluid.cp     = jf.value("cp_j_kgk", 520.0);

    const auto& j1 = j.at("cte1_reactor");
    setup.config.reactor_efficiency = j1.value("reactor_efficiency", 1.0);
    setup.config.p_th_max           = j1.value("p_th_max_w", 16.0e6);
    setup.config.t_out_1_min        = j1.value("t_out_1_min", 1200.0);
    setup.config.t_out_1_max        = j1.value("t_out_1_max", 1400.0);

    const auto& j2 = j.at("cte2_brayton");
    setup.config.eta_b           = j2.value("eta_b", 0.25);
    setup.config.recuperator_eff = j2.value("recuperator_eff", 0.85);

    const auto& j3 = j.at("cte3_pmad");
    setup.config.eta_pmad            = j3.value("eta_pmad", 0.99);
    setup.config.thruster_power_frac = j3.value("thruster_power_frac", 0.75);

    const auto& j4 = j.at("cte4_propulsion");
    setup.config.num_thrusters = j4.value("num_thrusters", 12);
    setup.config.eta_thruster  = j4.value("eta_thruster", 0.60);

    validateConfig(setup);
    return setup;
}