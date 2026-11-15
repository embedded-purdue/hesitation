
#include "EnvironmentChannel.h"

#include <utility>

namespace Hesitation::Environment {

namespace {

struct EnvironmentChannelState {
    zmq::context_t context{1};

    Hesitation::Messaging::Channel hub{context};
    Hesitation::Messaging::Channel channel{context};
};

EnvironmentChannelState& GetState() {
    static EnvironmentChannelState state;
    return state;
}

} // namespace

bool EnvironmentChannel::StartHub(const std::string& publishEndpoint,
                                  const std::string& subscribeEndpoint) {
    return GetState().hub.StartHub(publishEndpoint, subscribeEndpoint);
}

bool EnvironmentChannel::Start(const std::string& publishEndpoint,
                               const std::string& subscribeEndpoint) {
    return GetState().channel.Start(publishEndpoint, subscribeEndpoint);
}

void EnvironmentChannel::Stop() {
    auto& state = GetState();

    state.channel.Stop();
    state.hub.Stop();
}

bool EnvironmentChannel::IsStarted() {
    return GetState().channel.IsStarted();
}

bool EnvironmentChannel::Send(const hesitation::messaging::Message& message) {
    return GetState().channel.Send(message);
}

bool EnvironmentChannel::Subscribe(const std::string& type,
                                   Hesitation::Messaging::Receiver::Callback callback) {
    return GetState().channel.Subscribe(type, std::move(callback));
}

} // namespace Hesitation::Environment