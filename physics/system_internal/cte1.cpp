#include "cte1.h"
#include <stdexcept>

ReactorOutput simulateReactor(const ReactorInput & input, const WorkingFluid & fluid) {

    if (input.p_th_cmd < 0 || input.t_in <= 0 || input.reactor_efficiency <= 0 || fluid.m_flow <= 0 || fluid.cp <= 0) {
        throw std::invalid_argument("Invalid reactor input");
    }

    ReactorOutput output;

    output.p_th = input.p_th_cmd * input.reactor_efficiency;

    output.t_out_1 =
        input.t_in + output.p_th / (fluid.m_flow * fluid.cp); // Output coolant temp from CTE-1

    return output;
}