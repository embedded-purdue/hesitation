#include "cte3.h"
#include <stdexcept>

PMAD_Output simulatePMAD(const PMAD_Input& input) {
    if (input.p_gen <= 0 || input.eta_p <= 0 || input.eta_p >= 1.0) {
        throw std::invalid_argument("Invalid PMAD input");
    }

    PMAD_Output output;

    output.p_total = input.eta_p * input.p_gen;
    output.p_waste_2 = (1.0 - input.eta_p) * input.p_gen;
    output.p_thruster = 0.75 * output.p_total;
    output.p_aux = 0.25 * output.p_total;

    return output;
};