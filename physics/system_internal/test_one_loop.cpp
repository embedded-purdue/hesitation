
#include <iostream>
#include "cte1.h"
#include "cte2.h"

int main() {
    // Shared working fluid
    WorkingFluid fluid;
    fluid.m_flow = 100.0;
    fluid.cp = 520.0;

    // CTE1 inputs
    ReactorInput reactorInput;
    reactorInput.p_th_cmd = 16e6;
    reactorInput.t_in = 1000.0;

    // Run reactor
    ReactorOutput reactor =
        simulateReactor(reactorInput, fluid);

    // CTE2 inputs
    BraytonInput braytonInput;
    braytonInput.p_th = reactor.p_th;
    braytonInput.t_out_1 = reactor.t_out_1;
    braytonInput.t_out_5 = 400.0;

    // Run Brayton cycle
    BraytonOutput brayton =
        simulateBrayton(braytonInput, fluid);

    std::cout << "Thermal Power: "
              << reactor.p_th << " W\n";

    std::cout << "Reactor Outlet: "
              << reactor.t_out_1 << " K\n";

    std::cout << "Electrical Power: "
              << brayton.p_gen << " W\n";

    std::cout << "Waste Heat: "
              << brayton.p_waste_1 << " W\n";

    std::cout << "Cooler Inlet: "
              << brayton.t_out_2 << " K\n";

    return 0;
}
