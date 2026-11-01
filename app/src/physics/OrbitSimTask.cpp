#include "OrbitSimTask.h"

#include "messaging/channels/environment/EnvironmentChannel.h"

#include <iostream>
#include <chrono>
#include <thread>

using Hesitation::Environment::EnvironmentChannel;

bool OrbitSimTask::Initialize() {

    // Simulation Start
    if (!EnvironmentChannel::Subscribe("environment.simulation_start",
                                       [](const hesitation::messaging::Message& message) {
                                           std::cout << "[OrbitSim] Simulation START received\n";
                                       })) {
        return false;
    }

    // Simulation Stop
    if (!EnvironmentChannel::Subscribe("environment.simulation_stop",
                                       [](const hesitation::messaging::Message& message) {
                                           std::cout << "[OrbitSim] Simulation STOP received\n";
                                       })) {
        return false;
    }

    // Simulation Reset
    if (!EnvironmentChannel::Subscribe("environment.simulation_reset",
                                       [](const hesitation::messaging::Message& message) {
                                           std::cout << "[OrbitSim] Simulation RESET received\n";
                                       })) {
        return false;
    }

    std::cout << "[OrbitSim] Message listeners registered\n";

    return true;
}

void OrbitSimTask::Run() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}