#include "EnvironmentMessages.h"

#include "../../core/MessageFactory.h"
#include "environment.pb.h"

namespace Hesitation::Environment {

using hesitation::messaging::Message;
using Hesitation::Messaging::MessageFactory;

void EnvironmentMessages::RegisterAll() {
    MessageFactory::Register<hesitation::environment::SimulationStart>(
        "environment.simulation_start");

    MessageFactory::Register<hesitation::environment::SimulationStop>(
        "environment.simulation_stop");

    MessageFactory::Register<hesitation::environment::SimulationReset>(
        "environment.simulation_reset");

    MessageFactory::Register<hesitation::environment::SimulationStatus>(
        "environment.simulation_status");

    MessageFactory::Register<hesitation::environment::PositionTelemetry>(
        "environment.position_telemetry");

    MessageFactory::Register<hesitation::environment::VelocityTelemetry>(
        "environment.velocity_telemetry");
}

Message EnvironmentMessages::SimulationStart() {
    return MessageFactory::Build("environment.simulation_start",
                                 hesitation::environment::SimulationStart{});
}

Message EnvironmentMessages::SimulationStop() {
    return MessageFactory::Build("environment.simulation_stop",
                                 hesitation::environment::SimulationStop{});
}

Message EnvironmentMessages::SimulationReset(
    const hesitation::environment::SimulationReset& config) {

    return MessageFactory::Build("environment.simulation_reset", config);
}

} // namespace Hesitation::Environment