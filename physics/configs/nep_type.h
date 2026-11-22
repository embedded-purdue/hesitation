#pragma once
#include <array>
#include <cstdint>

// This file is where our static parameters will live in CPP. They are initalized here to give them a place in memory obviously
// and will take values from the JSON file.

constexpr size_t MAX_THRUSTERS;

enum class FaultFlag : uint32_t {
    NONE                = 0,
    INVALID_INPUT       = 1 << 0,
    OVER_TEMPERATURE    = 1 << 1,
    POWER_SATURATED     = 1 << 2,
    ISP_OUT_OF_RANGE    = 1 << 3
};

inline FaultFlag operator|(FaultFlag a, FaultFlag b) {
    return static_cast<FaultFlag>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

struct WorkingFluid {
    double m_flow; // Coolant mass flow rate (kg/s)
    double cp; // Specific heat capacity (J/(kg*K))
};

struct NEPSystemConfig {
    // CTE-1
    double reactor_efficiency;
    double p_th_max;
    double t_out_1_min;
    double t_out_1_max; 

    // CTE-2
    double eta_b;
    double recuperator_eff;
    double brayton_tau_;
    double t_out_2_min;
    double t_out_2_max;

    // CTE-3
    double eta_pmad;
    double thruster_power_frac;

    // CTE-4
    size_t num_thrusters;
    double eta_thruster;

    // CTE-5
};