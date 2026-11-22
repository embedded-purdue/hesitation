
#pragma once
#include "working_fluid.h"

struct BraytonInput {
    double p_th; // Thermal input (W)
    double t_out_1; // Reactor outlet (K)
    double t_out_5; // Radiator outlet (K)

    const double eta_b = 0.25; // Brayton energy conversion efficiency
};

struct BraytonOutput {
    double p_gen; // Electrical power (W)
    double p_waste_1; // Waste heat (W)
    double t_out_2; // Cooler inlet (K)

    bool in_design_range = false;
};

BraytonOutput simulateBrayton(const BraytonInput& input,const WorkingFluid& fluid);
