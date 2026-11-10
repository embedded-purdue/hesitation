
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "messaging.pb.h"
#include <google/protobuf/message.h>

namespace Hesitation::Messaging {

class MessageFactory {
  public:
    // Register a logical message identifier and its Protobuf type.
    template <typename T> static void Register(const std::string& type);

    // Build an envelope from a registered Protobuf payload.
    template <typename T>
    static hesitation::messaging::Message Build(const std::string& type, const T& payload);

    // Build an envelope for an empty Protobuf message.
    template <typename T> static hesitation::messaging::Message Build(const std::string& type);

    static bool IsRegistered(const std::string& type);

  private:
    static void RegisterInternal(const std::string& type,
                                 const google::protobuf::Descriptor* descriptor);

    static bool IsRegisteredInternal(const std::string& type,
                                     const google::protobuf::Descriptor* descriptor);

    static uint64_t GenerateMessageId();
    static uint64_t GenerateTimestamp();

    static std::unordered_map<std::string, const google::protobuf::Descriptor*> registry_;
};

// Template implementations

template <typename T> void MessageFactory::Register(const std::string& type) {
    static_assert(std::is_base_of_v<google::protobuf::Message, T>, "T must be a Protobuf message");

    RegisterInternal(type, T::descriptor());
}

template <typename T>
hesitation::messaging::Message MessageFactory::Build(const std::string& type, const T& payload) {

    static_assert(std::is_base_of_v<google::protobuf::Message, T>, "T must be a Protobuf message");

    if (!IsRegisteredInternal(type, T::descriptor())) {
        throw std::runtime_error("Unregistered or mismatched message type: " + type);
    }

    hesitation::messaging::Message message;

    message.set_message_id(GenerateMessageId());
    message.set_timestamp(GenerateTimestamp());
    message.set_type(type);

    std::string serializedPayload;

    if (!payload.SerializeToString(&serializedPayload)) {
        throw std::runtime_error("Failed to serialize Protobuf payload");
    }

    message.set_payload(std::move(serializedPayload));

    return message;
}

template <typename T>
hesitation::messaging::Message MessageFactory::Build(const std::string& type) {
    T payload;
    return Build(type, payload);
}

} // namespace Hesitation::Messaging
