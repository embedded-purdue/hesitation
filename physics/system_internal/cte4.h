#pragma once
#include <random>

std::mt19937 gen(std::random_device{}());
double eta_range = std::uniform_real_distribution<double>(0.0, 1.0)(gen);

struct ThrusterInput{
    double p_thruster = 0.0;
    double eta_t = 0.0;
    double f_cmd = 0.0;
};

struct ThrusterOutput{
    
};