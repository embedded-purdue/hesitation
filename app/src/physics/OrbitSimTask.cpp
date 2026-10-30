#include "OrbitSimTask.h"

void OrbitSimTask::Run() {
    while (running) {
        std::cout << "Orbit Sim Running..." << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}