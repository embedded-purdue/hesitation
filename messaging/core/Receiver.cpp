#include "Receiver.h"

#include <chrono>
#include <iostream>

namespace Hesitation::Messaging {

Receiver::Receiver(zmq::context_t& context) : context_(context) {}

Receiver::~Receiver() {
    Stop();
}

bool Receiver::Subscribe(const std::string& type, Callback callback) {
    if (running_ || type.empty() || !callback)
        return false;

    std::lock_guard<std::mutex> lock(callbacksMutex_);

    callbacks_[type].push_back(std::move(callback));

    return true;
}

bool Receiver::Start(const std::string& endpoint) {
    if (running_)
        return true;

    running_ = true;

    workerThread_ = std::thread(&Receiver::Run, this, endpoint);

    return true;
}

void Receiver::Stop() {
    running_ = false;

    if (workerThread_.joinable())
        workerThread_.join();
}

bool Receiver::IsStarted() const {
    return running_;
}

void Receiver::Run(std::string endpoint) {
    try {
        zmq::socket_t subscriber(context_, zmq::socket_type::sub);

        subscriber.set(zmq::sockopt::linger, 0);
        subscriber.set(zmq::sockopt::rcvtimeo, 100);

        // Subscribe to all registered message types.
        {
            std::lock_guard<std::mutex> lock(callbacksMutex_);

            for (const auto& [type, callbacks] : callbacks_) {
                (void)callbacks;

                subscriber.set(zmq::sockopt::subscribe, type + "|");
            }
        }

        subscriber.connect(endpoint);

        std::cout << "[Receiver] Connected to " << endpoint << '\n';

        while (running_) {
            zmq::message_t topicFrame;

            auto topicResult = subscriber.recv(topicFrame, zmq::recv_flags::none);

            if (!topicResult.has_value())
                continue;

            if (!topicFrame.more())
                continue;

            zmq::message_t payloadFrame;

            auto payloadResult = subscriber.recv(payloadFrame, zmq::recv_flags::none);

            if (!payloadResult.has_value())
                continue;

            // Reject unexpected additional frames.
            if (payloadFrame.more())
                continue;

            std::string topic(static_cast<const char*>(topicFrame.data()), topicFrame.size());

            // Remove the topic delimiter.
            if (topic.empty() || topic.back() != '|')
                continue;

            topic.pop_back();

            hesitation::messaging::Message message;

            if (!message.ParseFromArray(payloadFrame.data(),
                                        static_cast<int>(payloadFrame.size()))) {
                std::cerr << "[Receiver] Invalid Protobuf message\n";
                continue;
            }

            // Ensure the envelope matches the transport topic.
            if (message.type() != topic)
                continue;

            std::vector<Callback> callbacks;

            {
                std::lock_guard<std::mutex> lock(callbacksMutex_);

                const auto it = callbacks_.find(topic);

                if (it != callbacks_.end())
                    callbacks = it->second;
            }

            for (const auto& callback : callbacks) {
                try {
                    callback(message);
                } catch (const std::exception& e) {
                    std::cerr << "[Receiver] Callback error: " << e.what() << '\n';
                } catch (...) {
                    std::cerr << "[Receiver] Unknown callback error\n";
                }
            }
        }

    } catch (const zmq::error_t& e) {
        std::cerr << "[Receiver] Error: " << e.what() << '\n';
        running_ = false;
    }
}

} // namespace Hesitation::Messaging