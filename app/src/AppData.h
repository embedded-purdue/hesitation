#pragma once

#include <string>

struct AppData {

    struct SimulationConfiguration {
        std::string runType = "Regular";

        double epoch = 1700000000.0;

        double position[3] = {};
        double velocity[3] = {};

        double sigmaBN[3]  = {};
        double omegaBNB[3] = {};

        // Orbital elements
        double semiMajorAxis = 6978000.0;
        double eccentricity  = 0.01;
        double inclination   = 63.3;
        double raan          = 88.2;
        double argPeriapsis  = 347.8;
        double trueAnomaly   = 135.3;

        int stateMode = 0;
    };

    struct SimulationStatus {
        bool running          = false;
        double simulationTime = 0.0;
    };

    SimulationConfiguration simulationConfig;
    SimulationStatus simulationStatus;
};

AppData& GetApplicationData();