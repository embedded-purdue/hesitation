#pragma once
#include "../working_fluid.h"

struct ReactorInput {
    double p_th_cmd = 0.0; // Requested thermal power (W)
    double t_in = 0.0; // Reactor inlet temperature (K)
    double reactor_efficiency = 1.0; // Assumed command-to-thermal efficiency

};

struct ReactorOutput {
    double p_th = 0.0; // Actual thermal output (W)
    double t_out_1 = 0.0; // Reactor outlet temperature (K)
};

ReactorOutput simulateReactor(
    const ReactorInput& input,
    const WorkingFluid& fluid
);