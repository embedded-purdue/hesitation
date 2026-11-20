cmake_minimum_required(VERSION 3.18)
project(hesitation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(HESITATION_BUILD_TESTS "Build kalman filtering tests" ON)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(kalman_filter STATIC
  src/quat_utils.cpp
  src/mekf.cpp
  src/thermal_power_ekf.cpp
  src/sensor_models.cpp
  src/estimator_bank.cpp
  src/orbit_ekf.cpp   
  src/scalar_kf.cpp  
)

target_include_directories(kalman_filter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(kalman_filter PUBLIC Eigen3::Eigen)

target_compile_options(kalman_filter PRIVATE
  -Wall -Wextra -Wpedantic
)

if(HESITATION_BUILD_TESTS)
  enable_testing()

  add_executable(test_mekf tests/test_mekf.cpp)
  target_link_libraries(test_mekf PRIVATE kalman_filter)
  add_test(NAME mekf COMMAND test_mekf)
endif()


