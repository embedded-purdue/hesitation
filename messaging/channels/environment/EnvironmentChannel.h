#pragma once

#include <string>

#include "../../core/Channel.h"

namespace Hesitation::Environment {

class EnvironmentChannel {
  public:
    // Start the central network.
    static bool StartHub(const std::string& publishEndpoint, const std::string& subscribeEndpoint);

    // Connect this process to the network.
    static bool Start(const std::string& publishEndpoint, const std::string& subscribeEndpoint);

    static void Stop();

    static bool IsStarted();

    static bool Send(const hesitation::messaging::Message& message);

    static bool Subscribe(const std::string& type,
                          Hesitation::Messaging::Receiver::Callback callback);
};

} // namespace Hesitation::Environment