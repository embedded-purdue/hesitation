#pragma once

#include "messaging.pb.h"

namespace Hesitation::Environment {

class EnvironmentMessages {
  public:
    static void RegisterAll();

    static hesitation::messaging::Message SimulationStart();
    static hesitation::messaging::Message SimulationStop();
    static hesitation::messaging::Message SimulationReset();
};

} // namespace Hesitation::Environment