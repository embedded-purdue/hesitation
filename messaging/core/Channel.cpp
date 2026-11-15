
#include "Channel.h"

#include <chrono>
#include <iostream>
#include <future>

#include <zmq.hpp>

namespace Hesitation::Messaging {

namespace {

bool ForwardMessage(zmq::socket_t& source, zmq::socket_t& destination) {
    zmq::message_t frame;

    auto received = source.recv(frame, zmq::recv_flags::none);

    if (!received.has_value())
        return false;

    while (true) {
        const bool more = frame.more();

        auto sent =
            destination.send(frame, more ? zmq::send_flags::sndmore : zmq::send_flags::none);

        if (!sent.has_value())
            return false;

        if (!more)
            break;

        frame = zmq::message_t{};

        if (!source.recv(frame, zmq::recv_flags::none).has_value())
            return false;
    }

    return true;
}

} // namespace

Channel::Channel(zmq::context_t& context)
    : context_(context), sender_(context), receiver_(context) {}

Channel::~Channel() {
    Stop();
}

// Client lifecycle

bool Channel::Start(const std::string& publishEndpoint, const std::string& subscribeEndpoint) {
    if (started_)
        return !hubMode_;

    if (!receiver_.Start(subscribeEndpoint))
        return false;

    if (!sender_.Start(publishEndpoint)) {
        receiver_.Stop();
        return false;
    }

    started_ = true;
    hubMode_ = false;

    std::cout << "[Channel] Client started\n";

    return true;
}

// Hub lifecycle

bool Channel::StartHub(const std::string& publishEndpoint, const std::string& subscribeEndpoint) {
    if (started_)
        return hubMode_;

    std::promise<bool> startup;
    auto future = startup.get_future();

    hubRunning_ = true;

    hubThread_ =
        std::thread(&Channel::RunHub, this, publishEndpoint, subscribeEndpoint, std::move(startup));

    const bool success = future.get();

    if (!success) {
        hubRunning_ = false;

        if (hubThread_.joinable())
            hubThread_.join();

        return false;
    }

    started_ = true;
    hubMode_ = true;

    std::cout << "[Channel] Hub started\n";

    return true;
}

void Channel::RunHub(std::string publishEndpoint, std::string subscribeEndpoint,
                     std::promise<bool> startup) {
    try {
        zmq::socket_t publisherIngress(context_, zmq::socket_type::xsub);

        zmq::socket_t subscriberDistribution(context_, zmq::socket_type::xpub);

        publisherIngress.set(zmq::sockopt::linger, 0);
        subscriberDistribution.set(zmq::sockopt::linger, 0);

        publisherIngress.bind(publishEndpoint);
        subscriberDistribution.bind(subscribeEndpoint);

        std::cout << "[Channel] XSUB bound to " << publishEndpoint << '\n';

        std::cout << "[Channel] XPUB bound to " << subscribeEndpoint << '\n';

        startup.set_value(true);

        while (hubRunning_) {
            zmq::pollitem_t items[] = {{publisherIngress, 0, ZMQ_POLLIN, 0},
                                       {subscriberDistribution, 0, ZMQ_POLLIN, 0}};

            zmq::poll(items, 2, std::chrono::milliseconds(100));

            // Publications: XSUB -> XPUB
            if (items[0].revents & ZMQ_POLLIN) {
                ForwardMessage(publisherIngress, subscriberDistribution);
            }

            // Subscriptions: XPUB -> XSUB
            if (items[1].revents & ZMQ_POLLIN) {
                ForwardMessage(subscriberDistribution, publisherIngress);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[Channel] Hub error: " << e.what() << '\n';

        hubRunning_ = false;

        try {
            startup.set_value(false);
        } catch (const std::future_error&) {
        }
    }
}

// Messaging

bool Channel::Send(const hesitation::messaging::Message& message) {
    if (!started_ || hubMode_)
        return false;

    return sender_.Send(message);
}

bool Channel::Subscribe(const std::string& type, Receiver::Callback callback) {
    if (started_ || hubMode_)
        return false;

    return receiver_.Subscribe(type, std::move(callback));
}

// Shutdown

void Channel::Stop() {
    if (!started_ && !hubThread_.joinable())
        return;

    if (hubMode_) {
        hubRunning_ = false;

        if (hubThread_.joinable())
            hubThread_.join();
    } else {
        sender_.Stop();
        receiver_.Stop();
    }

    started_ = false;
    hubMode_ = false;

    std::cout << "[Channel] Stopped\n";
}

bool Channel::IsStarted() const {
    return started_;
}

} // namespace Hesitation::Messaging