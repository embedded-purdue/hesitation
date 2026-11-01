#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <future>

#include <zmq.hpp>
#include "messaging.pb.h"

namespace Hesitation::Messaging {

class Sender {
  public:
    explicit Sender(zmq::context_t& context);
    ~Sender();

    bool Start(const std::string& endpoint);
    bool Send(const hesitation::messaging::Message& message);
    void Stop();

    bool IsStarted() const;

  private:
    struct OutgoingMessage {
        std::string topic;
        std::string payload;
    };

    void Run(std::string endpoint, std::promise<bool> startup);

    zmq::context_t& context_;

    std::thread workerThread_;
    std::atomic<bool> running_{false};

    std::mutex queueMutex_;
    std::condition_variable queueCondition_;
    std::queue<OutgoingMessage> queue_;

    static constexpr std::size_t MAX_QUEUE_SIZE = 1024;
};

} // namespace Hesitation::Messaging