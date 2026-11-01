
#include "cte2.h"
#include <stdexcept>

BraytonOutput simulateBrayton(
    const BraytonInput& input,
    const WorkingFluid& fluid
) {
    if (input.p_th <= 0 ||
        input.t_out_1 <= input.t_out_5 ||
        input.t_out_5 <= 0 ||
        input.eta_b <= 0 ||
        input.eta_b >= 1 ||
        fluid.m_flow <= 0 ||
        fluid.cp <= 0) {
        throw std::invalid_argument("Invalid Brayton input");
    }

    BraytonOutput output;

    output.p_gen = input.eta_b * input.p_th;

    output.p_waste_1 = input.p_th - output.p_gen;

    output.t_out_2 = input.t_out_5 +
        output.p_waste_1 / (fluid.m_flow * fluid.cp);

    if (output.t_out_2 >= input.t_out_1) {
        throw std::invalid_argument("Infeasible temperatures");
    }

    output.in_design_range =
        (input.t_out_1 >= 1200 && input.t_out_1 <= 1400) &&
        (input.t_out_5 >= 325 && input.t_out_5 <= 500) &&
        (output.t_out_2 >= 550 && output.t_out_2 <= 750);

    return output;
}