cmake_minimum_required(VERSION 3.16)
project(star_tracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization and warning flags
if(MSVC)
    add_compile_options(/W4 /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

include(FetchContent)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Star Tracker Core Library
set(STAR_TRACKER_SRCS
    src/catalog/star_catalog.cpp
    src/detection/image.cpp
    src/detection/centroid_extractor.cpp
    src/detection/star_detector.cpp
    src/database/reference_image.cpp
    src/database/reference_database.cpp
    src/database/database_generator.cpp
    src/lost_in_space/distance_transform.cpp
    src/lost_in_space/centroid_matcher.cpp
    src/lost_in_space/lost_in_space_matcher.cpp
    src/aim/aim_estimator.cpp
    src/tracker/attitude_manager.cpp
)

add_library(star_tracker_lib STATIC ${STAR_TRACKER_SRCS})
target_include_directories(star_tracker_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Database Generator Tool
add_executable(generate_database tools/generate_database.cpp)
target_link_libraries(generate_database PRIVATE star_tracker_lib)

# Main Application & Simulation Runner
add_executable(star_tracker_main src/main.cpp)
target_link_libraries(star_tracker_main PRIVATE star_tracker_lib)

# Testing with GoogleTest
enable_testing()

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

macro(add_star_tracker_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_link_libraries(${test_name} PRIVATE star_tracker_lib GTest::gtest_main)
    add_test(NAME ${test_name} COMMAND ${test_name})
endmacro()

add_star_tracker_test(test_quaternion tests/test_quaternion.cpp)
add_star_tracker_test(test_camera_model tests/test_camera_model.cpp)
add_star_tracker_test(test_centroid_extractor tests/test_centroid_extractor.cpp)
add_star_tracker_test(test_distance_transform tests/test_distance_transform.cpp)
add_star_tracker_test(test_centroid_matcher tests/test_centroid_matcher.cpp)
add_star_tracker_test(test_lost_in_space tests/test_lost_in_space.cpp)
add_star_tracker_test(test_aim tests/test_aim.cpp)
add_star_tracker_test(test_attitude_manager tests/test_attitude_manager.cpp)
