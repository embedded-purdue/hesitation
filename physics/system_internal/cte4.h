#pragma once
#include <vector>

struct ThrusterInput {
    double i_sp = 0.0;
    const double eta_t = 0.60;
    double f_cmd = 0.0;
};

struct PropulsionInput {
    std::vector<ThrusterInput> thrusters;
    double p_propulsion = 0.0;
};

struct PropulsionOutput {
    std::vector<double> thrusters;
    double p_waste = 0.0;
};

PropulsionOutput simulatePropulsion(const PropulsionInput& input);
