cmake_minimum_required(VERSION 3.16)

project(HesitationPhysics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(json)

# add_executable(one_loop_test
  #  system_internal/cte1.cpp
   # system_internal/cte2.cpp
    #system_internal/test_one_loop.cpp
#)

add_executable(config_test
    tests/config_test.cpp
    configs/config_loader.cpp
)

target_include_directories(config_test PRIVATE include)
target_link_libraries(config_test PRIVATE nlohmann_json::nlohmann_json)

# Link it to your executable or library target
target_link_libraries(nep_sim PRIVATE nlohmann_json::nlohmann_json)
