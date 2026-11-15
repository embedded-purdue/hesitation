#include <iostream>
#include <exception>
#include "config_loader.h"

int main(int argc, char* argv[]) {
    std::string cfg_path = (argc > 1) ? argv[1] : "configs/nep_config.json";

    try {
        LoadedNEPSetup sim = loadNEPConfig(cfg_path);

        std::cout << "Successfully loaded config from: " << cfg_path << "\n"
                  << "  Fluid m_flow:       " << sim.fluid.m_flow << " kg/s\n"
                  << "  Fluid cp:           " << sim.fluid.cp << " J/(kg*K)\n"
                  << "  Reactor Max Power:  " << sim.config.p_th_max / 1e6 << " MW\n"
                  << "  Brayton Efficiency: " << sim.config.eta_b * 100.0 << " %\n"
                  << "  PMAD Efficiency:    " << sim.config.eta_pmad * 100.0 << " %\n"
                  << "  Active Thrusters:   " << sim.config.num_thrusters << "\n"
                  << "  Thruster Eff:       " << sim.config.eta_thruster * 100.0 << " %\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "CONFIG TEST FAILED: " << e.what() << "\n";
        return 1;
    }
}