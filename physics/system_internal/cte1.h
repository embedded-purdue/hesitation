#pragma once

struct ReactorInput {
    double p_cmd;
    double t_in;
    double m_flow;
};

struct ReactorOutput {
    double p_th;
    double t_out_1;
};

ReactorOutput simulateReactor(const ReactorInput& input);