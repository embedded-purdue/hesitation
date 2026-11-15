#pragma once
#include "working_fluid.h"

struct ReactorInput {
    double p_th_cmd; // Requested thermal power (W)
    double t_in; // Reactor inlet temperature (K)
    const double reactor_efficiency = 1.0; // Assumed command-to-thermal efficiency
};

struct ReactorOutput {
    double p_th; // Actual thermal output (W)
    double t_out_1; // Reactor outlet temperature (K)
};

ReactorOutput simulateReactor(
    ReactorInput& input,
    const WorkingFluid& fluid
);