include(FetchContent)

# ==============================================================================
# ZEROMQ
# ==============================================================================

set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(WITH_PERF_TOOL OFF CACHE BOOL "" FORCE)
set(WITH_DOC OFF CACHE BOOL "" FORCE)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

if(WIN32 AND MINGW)
    set(ZMQ_HAVE_WINDOWS ON CACHE BOOL "" FORCE)
    set(ZMQ_HAVE_IPC OFF CACHE BOOL "" FORCE)
endif()

FetchContent_Declare(
    libzmq
    GIT_REPOSITORY https://github.com/zeromq/libzmq.git
    GIT_TAG v4.3.5
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(libzmq)

# ==============================================================================
# CPPZMQ
# ==============================================================================

set(CPPZMQ_BUILD_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    cppzmq
    GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
    GIT_TAG v4.10.0
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(cppzmq)

# ==============================================================================
# PROTOBUF
# ==============================================================================

set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    protobuf
    GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
    GIT_TAG v33.0
    GIT_SHALLOW TRUE
    GIT_SUBMODULES_RECURSE TRUE
)

FetchContent_MakeAvailable(protobuf)

include("${protobuf_SOURCE_DIR}/cmake/protobuf-generate.cmake")

# ==============================================================================
# MESSAGING LIBRARY
# ==============================================================================

set(MESSAGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(messaging STATIC
    # Core
    ${MESSAGING_DIR}/core/MessageFactory.cpp
    ${MESSAGING_DIR}/core/Sender.cpp
    ${MESSAGING_DIR}/core/Receiver.cpp
    ${MESSAGING_DIR}/core/Channel.cpp

    # Environment channel
    ${MESSAGING_DIR}/channels/environment/EnvironmentChannel.cpp
    ${MESSAGING_DIR}/channels/environment/EnvironmentMessages.cpp
)

# ==============================================================================
# PROTOBUF CODE GENERATION
# ==============================================================================

protobuf_generate(
    TARGET messaging
    LANGUAGE cpp
    PROTOS
        ${MESSAGING_DIR}/proto/messaging.proto
        ${MESSAGING_DIR}/proto/environment.proto
    IMPORT_DIRS
        ${MESSAGING_DIR}/proto
        ${protobuf_SOURCE_DIR}/src
    APPEND_PATH
)

# ==============================================================================
# INCLUDE DIRECTORIES
# ==============================================================================

target_include_directories(messaging
    PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${MESSAGING_DIR}/core
        ${MESSAGING_DIR}/proto
        ${MESSAGING_DIR}/channels/environment
)

# ==============================================================================
# LINK DEPENDENCIES
# ==============================================================================

target_link_libraries(messaging
    PUBLIC
        cppzmq
        libzmq
        protobuf::libprotobuf
)
