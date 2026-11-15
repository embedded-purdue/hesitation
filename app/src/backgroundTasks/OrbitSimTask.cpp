#include "OrbitSimTask.h"

#include "messaging/channels/environment/EnvironmentChannel.h"
#include "environment.pb.h"

#include <iostream>
#include <chrono>
#include <thread>

using Hesitation::Environment::EnvironmentChannel;

bool OrbitSimTask::Initialize() {

    // Simulation Start
    if (!EnvironmentChannel::Subscribe("environment.simulation_start",
                                       [](const hesitation::messaging::Message&) {
                                           std::cout << "[OrbitSim] Simulation START received\n";
                                       })) {
        return false;
    }

    // Simulation Stop
    if (!EnvironmentChannel::Subscribe("environment.simulation_stop",
                                       [](const hesitation::messaging::Message&) {
                                           std::cout << "[OrbitSim] Simulation STOP received\n";
                                       })) {
        return false;
    }

    // Simulation Reset
    if (!EnvironmentChannel::Subscribe(
            "environment.simulation_reset", [](const hesitation::messaging::Message& message) {
                hesitation::environment::SimulationReset reset;

                // Deserialize protobuf payload.
                if (!reset.ParseFromString(message.payload())) {
                    std::cerr << "[OrbitSim] Failed to deserialize reset message\n";
                    return;
                }

                std::cout << "\n========== SIMULATION RESET ==========\n";

                std::cout << "Message Type: " << message.type() << '\n';
                std::cout << "Payload Size: " << message.payload().size() << '\n';

                std::cout << "Run Type: " << reset.run_type() << '\n';

                std::cout << "\nEpoch (UTC): " << reset.year() << '-' << reset.month() << '-'
                          << reset.day() << ' ' << reset.hour() << ':' << reset.minute() << ':'
                          << reset.second() << '\n';

                std::cout << "\nPosition (m):\n"
                          << "  X: " << reset.position_x() << '\n'
                          << "  Y: " << reset.position_y() << '\n'
                          << "  Z: " << reset.position_z() << '\n';

                std::cout << "\nVelocity (m/s):\n"
                          << "  X: " << reset.velocity_x() << '\n'
                          << "  Y: " << reset.velocity_y() << '\n'
                          << "  Z: " << reset.velocity_z() << '\n';

                std::cout << "\nAttitude (MRP):\n"
                          << "  X: " << reset.mrp_x() << '\n'
                          << "  Y: " << reset.mrp_y() << '\n'
                          << "  Z: " << reset.mrp_z() << '\n';

                std::cout << "\nAngular Velocity (rad/s):\n"
                          << "  X: " << reset.omega_x() << '\n'
                          << "  Y: " << reset.omega_y() << '\n'
                          << "  Z: " << reset.omega_z() << '\n';

                std::cout << "======================================\n";
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
