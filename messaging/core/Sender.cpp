
#include "Sender.h"

#include <chrono>
#include <future>
#include <iostream>
#include <utility>

namespace Hesitation::Messaging {

Sender::Sender(zmq::context_t& context) : context_(context) {}

Sender::~Sender() {
    Stop();
}

bool Sender::Start(const std::string& endpoint) {
    if (running_)
        return true;

    std::promise<bool> startup;
    auto future = startup.get_future();

    running_ = true;

    workerThread_ = std::thread([this, endpoint, startup = std::move(startup)]() mutable {
        Run(endpoint, std::move(startup));
    });

    const bool success = future.get();

    if (!success && workerThread_.joinable())
        workerThread_.join();

    return success;
}

void Sender::Run(std::string endpoint, std::promise<bool> startup) {
    try {
        zmq::socket_t publisher(context_, zmq::socket_type::pub);

        publisher.set(zmq::sockopt::linger, 0);
        publisher.connect(endpoint);

        std::cout << "[Sender] Connected to " << endpoint << '\n';

        startup.set_value(true);

        while (true) {
            OutgoingMessage outgoing;

            {
                std::unique_lock<std::mutex> lock(queueMutex_);

                queueCondition_.wait_for(lock, std::chrono::milliseconds(100),
                                         [this] { return !queue_.empty() || !running_; });

                if (queue_.empty()) {
                    if (!running_)
                        break;

                    continue;
                }

                outgoing = std::move(queue_.front());
                queue_.pop();
            }

            // Send topic frame.
            zmq::message_t topicFrame(outgoing.topic.data(), outgoing.topic.size());

            auto topicResult = publisher.send(topicFrame, zmq::send_flags::sndmore);

            if (!topicResult.has_value()) {
                std::cerr << "[Sender] Failed to send topic\n";
                continue;
            }

            // Send serialized envelope.
            zmq::message_t payloadFrame(outgoing.payload.data(), outgoing.payload.size());

            auto payloadResult = publisher.send(payloadFrame, zmq::send_flags::none);

            if (!payloadResult.has_value()) {
                std::cerr << "[Sender] Failed to send payload\n";
            }
        }

    } catch (const zmq::error_t& e) {
        std::cerr << "[Sender] Error: " << e.what() << '\n';

        running_ = false;

        try {
            startup.set_value(false);
        } catch (const std::future_error&) {
            // Startup was already signalled.
        }
    }
}

bool Sender::Send(const hesitation::messaging::Message& message) {
    if (!running_)
        return false;

    if (message.type().empty())
        return false;

    // Serialize the complete envelope.
    std::string serialized;

    if (!message.SerializeToString(&serialized)) {
        std::cerr << "[Sender] Failed to serialize message\n";
        return false;
    }

    OutgoingMessage outgoing;

    outgoing.topic   = message.type() + "|";
    outgoing.payload = std::move(serialized);

    {
        std::lock_guard<std::mutex> lock(queueMutex_);

        if (!running_ || queue_.size() >= MAX_QUEUE_SIZE)
            return false;

        queue_.push(std::move(outgoing));
    }

    queueCondition_.notify_one();

    return true;
}

void Sender::Stop() {
    if (!running_ && !workerThread_.joinable())
        return;

    running_ = false;
    queueCondition_.notify_all();

    if (workerThread_.joinable())
        workerThread_.join();
}

bool Sender::IsStarted() const {
    return running_;
}

} // namespace Hesitation::Messaging
