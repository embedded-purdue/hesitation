#include "cte4.h"
#include <stdexcept>
#include <vector>

PropulsionOutput simulatePropulsion(const PropulsionInput& input){
    double g_0 = 9.800655;
    double total_power = 0.0;
    std::vector<double> p_need = {};
    std::vector<double> p_sup = {}; // Power supplied to each thruster (Proportional model)
    
    for(ThrusterInput thruster : input.thrusters){
        double power = thruster.f_cmd * thruster.i_sp * g_0/(2 * thruster.eta_t);
        p_need.push_back(power);
        total_power += power;
    }

    for (double p : p_need){
        p /= total_power;
        p_sup.push_back(p * input.p_propulsion);
    }

    PropulsionOutput thrust_out;
    double f_out_i;

    for (int i = 0; i <= input.thrusters.size(); i++){
        if (total_power > input.p_propulsion)
        {
            f_out_i = 2 * input.thrusters[i].eta_t * p_sup[i]/(g_0 * input.thrusters[i].i_sp);
        }
        else
        {
            f_out_i = p_need[i];
            thrust_out.p_waste += p_sup[i] - f_out_i;
        }
        
        thrust_out.thrusters.push_back(f_out_i);
    }

    return thrust_out;
}
