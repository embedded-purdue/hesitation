#pragma once

struct WorkingFluid {
    double m_flow = 0.0; // In kg/s
    double temp;
    const double cp = 520; // Assumed specific heat of ideal He-Xe mixture
};