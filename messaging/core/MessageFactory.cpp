#include "MessageFactory.h"

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace Hesitation::Messaging {

std::unordered_map<std::string, const google::protobuf::Descriptor*> MessageFactory::registry_;

namespace {
std::atomic<uint64_t> nextMessageId{1};
}

void MessageFactory::RegisterInternal(const std::string& type,
                                      const google::protobuf::Descriptor* descriptor) {
    if (type.empty() || descriptor == nullptr) {
        throw std::invalid_argument("Invalid message registration");
    }

    auto [it, inserted] = registry_.emplace(type, descriptor);

    if (!inserted && it->second != descriptor) {
        throw std::runtime_error("Message type already registered with a different payload: " +
                                 type);
    }
}

bool MessageFactory::IsRegisteredInternal(const std::string& type,
                                          const google::protobuf::Descriptor* descriptor) {
    const auto it = registry_.find(type);

    return it != registry_.end() && it->second == descriptor;
}

bool MessageFactory::IsRegistered(const std::string& type) {
    return registry_.find(type) != registry_.end();
}

uint64_t MessageFactory::GenerateMessageId() {
    return nextMessageId.fetch_add(1, std::memory_order_relaxed);
}

uint64_t MessageFactory::GenerateTimestamp() {
    const auto now = std::chrono::system_clock::now();

    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
}

} // namespace Hesitation::Messaging
