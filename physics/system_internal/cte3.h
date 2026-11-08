#pragma once
#include "../working_fluid.h"


struct PMAD_Input{
double P_gen = 0.00; // Power Generated (W)
double eta_PMAD = 0.99; // Power Management and Delivery Efficiency (%)
}

struct PMAD_Output{
double P_total = 0.00; // Total Power Available for Remaining Subsystems (W)
double P_waste_2 = 0.00; // Waste Power from PMAD (W)
double P_thruster = 0.00; // Power Delivered to Thruster (W)
double P_aux = 0.00; // Power delivered to Auxilary Systems (W)
}

PMAD_OUTPUT simulatePMAD(const PMAD_Input& input,const WorkingFluid& fluid);