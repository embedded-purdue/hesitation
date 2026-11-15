#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <zmq.hpp>
#include "messaging.pb.h"

namespace Hesitation::Messaging {

class Receiver {
  public:
    using Callback = std::function<void(const hesitation::messaging::Message&)>;

    explicit Receiver(zmq::context_t& context);
    ~Receiver();

    bool Subscribe(const std::string& type, Callback callback);

    bool Start(const std::string& endpoint);
    void Stop();

    bool IsStarted() const;

  private:
    void Run(std::string endpoint);

    zmq::context_t& context_;

    std::thread workerThread_;
    std::atomic<bool> running_{false};

    std::mutex callbacksMutex_;

    std::unordered_map<std::string, std::vector<Callback>> callbacks_;
};

} // namespace Hesitation::Messaging