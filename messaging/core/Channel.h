#pragma once

#include <atomic>
#include <future>
#include <string>
#include <thread>
#include <utility>

#include "Sender.h"
#include "Receiver.h"

namespace Hesitation::Messaging {

class Channel {
  public:
    explicit Channel(zmq::context_t& context);
    ~Channel();

    Channel(const Channel&)            = delete;
    Channel& operator=(const Channel&) = delete;

    // Connect to an existing hub.
    bool Start(const std::string& publishEndpoint, const std::string& subscribeEndpoint);

    // Host the central network.
    bool StartHub(const std::string& publishEndpoint, const std::string& subscribeEndpoint);

    bool Send(const hesitation::messaging::Message& message);

    bool Subscribe(const std::string& type, Receiver::Callback callback);

    void Stop();
    bool IsStarted() const;

  private:
    void RunHub(std::string publishEndpoint, std::string subscribeEndpoint,
                std::promise<bool> startup);

    zmq::context_t& context_;

    Sender sender_;
    Receiver receiver_;

    std::thread hubThread_;
    std::atomic<bool> hubRunning_{false};

    bool started_ = false;
    bool hubMode_ = false;
};

} // namespace Hesitation::Messaging
