
#include "cte4.h"
#include <stdexcept>
#include <vector>

PropulsionOutput simulatePropulsion(const PropulsionInput& input) {
    const double g_0 = 9.80665;
    double total_power = 0.0;
    std::vector<double> p_need;

    if (input.p_propulsion < 0.0) {
        throw std::invalid_argument("Invalid propulsion power");
    }

    for (const ThrusterInput& thruster : input.thrusters) {
        if (thruster.i_sp <= 0.0 || thruster.eta_t <= 0.0 ||
            thruster.eta_t > 1.0 || thruster.f_cmd < 0.0) {
            throw std::invalid_argument("Invalid thruster input");
        }

        double power = thruster.f_cmd * thruster.i_sp * g_0 / (2.0 * thruster.eta_t);

        p_need.push_back(power);
        total_power += power;
    }

    double scale = 1.0;

    if (total_power > input.p_propulsion) {
        scale = input.p_propulsion / total_power;
    }

    PropulsionOutput thrust_out;

    for (size_t i = 0; i < input.thrusters.size(); i++) {
        const ThrusterInput& thruster = input.thrusters[i];

        double p_sup = p_need[i] * scale;

        double f_out_i = 2.0 * thruster.eta_t * p_sup / (g_0 * thruster.i_sp);

        thrust_out.thrusters.push_back(f_out_i);
        thrust_out.p_waste += (1.0 - thruster.eta_t) * p_sup;
    }

    return thrust_out;
}
