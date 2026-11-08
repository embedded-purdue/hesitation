#pragma once
#include <random>

std::mt19937 gen(42);
double eta_range = std::uniform_real_distribution<double>(0.0, 1.0)(gen);

struct ThrusterInput{
    double p_thruster = 0.0;
    double eta_t = eta_range * 0.2 + 0.5;
    double f_cmd = 0.0;
};

struct ThrusterOutput{
    double i_sp = 0.0;
    double p_waste = 0.0;
    double f_out = 0.0;
};

ThrusterOutput simulateThruster(const ThrusterInput& input);
