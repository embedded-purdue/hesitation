# ==============================================================================
# CMakeLists.txt - TD3 Spacecraft Attitude Control Build Configuration
#
# Builds the C++/LibTorch executables:
#   - bin/train : Continuous curriculum TD3 training executable
#   - bin/eval  : Deterministic greedy policy evaluation executable
#
# Author: Eric Yu
# Date: October 2026
# Version: 1.0.0
# Reference: Elkins et al., AAS 20-475
# ==============================================================================

cmake_minimum_required(VERSION 3.18)
project(hesitation_td3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/third_party/libtorch")
find_package(Torch REQUIRED)

add_executable(train src/train.cpp)
target_include_directories(train PRIVATE include)
target_link_libraries(train "${TORCH_LIBRARIES}")
set_property(TARGET train PROPERTY CXX_STANDARD 17)
set_target_properties(train PROPERTIES
  INSTALL_RPATH "${CMAKE_SOURCE_DIR}/third_party/libtorch/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

add_executable(eval src/eval.cpp)
target_include_directories(eval PRIVATE include)
target_link_libraries(eval "${TORCH_LIBRARIES}")
set_property(TARGET eval PROPERTY CXX_STANDARD 17)
set_target_properties(eval PROPERTIES
  INSTALL_RPATH "${CMAKE_SOURCE_DIR}/third_party/libtorch/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)
