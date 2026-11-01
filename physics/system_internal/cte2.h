
#pragma once
#include "../working_fluid.h"

struct BraytonInput {
    double p_th = 0.0; // Thermal input (W)
    double t_out_1 = 0.0; // Reactor outlet (K)
    double t_out_5 = 0.0; // Radiator outlet (K)

    double eta_b = 0.25; // Brayton energy conversion efficiency
};

struct BraytonOutput {
    double p_gen = 0.0; // Electrical power (W)
    double p_waste_1 = 0.0; // Waste heat (W)
    double t_out_2 = 0.0; // Cooler inlet (K)

    bool in_design_range = false;
};

BraytonOutput simulateBrayton(const BraytonInput& input,const WorkingFluid& fluid);
