#include "cte1.h"
#include <stdexcept>

ReactorOutput simulateReactor(const ReactorInput& input) {

    if (input.p_cmd < 0 || input.t_in <= 0 || input.m_flow <= 0) {
        throw std::invalid_argument("Invalid reactor input");
    }

    const double efficiency_b = 0.25;
    const double specificHeat = 520;

    ReactorOutput output;

    output.p_th = input.p_cmd / efficiency_b;

    output.t_out_1 =
        input.t_in + output.p_th / (input.m_flow * specificHeat);

    return output;
}