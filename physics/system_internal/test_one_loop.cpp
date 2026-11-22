#include <iostream>
#include "cte1.h"
#include "cte2.h"
#include "cte3.h"
#include "cte4.h"

int main() {
    WorkingFluid fluid = {
        100, // Mass flow rate of coolant
        1000 // Inlet temperatue (K)
    };

    ReactorInput input_r = {
        16e6, // Requested Power (W)
        fluid.temp
    };

    ReactorOutput output_r = simulateReactor(input_r, fluid);

    BraytonInput input_b = {
        output_r.p_th,
        fluid.temp,
        fluid.temp
    };

    BraytonOutput output_b = simulateBrayton(input_b, fluid);
}
