#include <iostream>

struct ReactorInput{
    double p_cmd;
    double t_in;
    double m_flow;
};

struct ReactorOutput{
    double p_th;
    double t_out_1;
};

ReactorOutput simulateReactor(ReactorInput input){
    const double efficiency_b = 0.25;
    const double specificHeat = 520;

    ReactorOutput output;

    output.p_th = input.p_cmd/efficiency_b;

    output.t_out_1 = input.t_in + output.p_th/(input.m_flow * specificHeat);

    return output;
}
