#pragma once

#include "messaging.pb.h"
#include "environment.pb.h"

namespace Hesitation::Environment {

class EnvironmentMessages {
  public:
    static void RegisterAll();

    static hesitation::messaging::Message SimulationStart();
    static hesitation::messaging::Message SimulationStop();
    static hesitation::messaging::Message SimulationReset();

    static hesitation::messaging::Message SimulationReset(
        const hesitation::environment::SimulationReset& config);
};

} // namespace Hesitation::Environment