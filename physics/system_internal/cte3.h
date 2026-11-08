#pragma once
#include "../working_fluid.h"


struct PMAD_Input{
    double p_gen = 0.00; // Power Generated (W)
    double eta_p = 0.99; // Power Management and Delivery Efficiency (%)
};

struct PMAD_Output{
    double p_total = 0.00; // Total Power Available for Remaining Subsystems (W)
    double p_waste_2 = 0.00; // Waste Power from PMAD (W)
    double p_thruster = 0.00; // Power Delivered to Thruster (W)
    double p_aux = 0.00; // Power delivered to Auxilary Systems (W)
};

PMAD_Output simulatePMAD(const PMAD_Input& input);