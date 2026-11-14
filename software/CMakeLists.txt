cmake_minimum_required(VERSION 3.18)
project(HesitationSoftware VERSION 1.0.0 LANGUAGES CXX)

# Policy settings
if(POLICY CMP0135)
    # Prefer extraction timestamp to time of extraction for FetchContent archives
    cmake_policy(SET CMP0135 NEW)
endif()

# Global C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compilation database for LSP / clangd tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
endif()

# Modular subsystem build options
option(BUILD_STAR_TRACKING    "Build Star Tracking subsystem"      ON)
option(BUILD_TD3_ATTITUDE     "Build TD3 Attitude Control module"  ON)
option(BUILD_THRUST_VECTORING  "Build Thrust Vectoring module"      ON)
option(BUILD_TESTING          "Enable test suites"                 ON)

# Global test harness
if(BUILD_TESTING)
    enable_testing()
endif()

# Subdirectories
if(BUILD_STAR_TRACKING)
    message(STATUS "[Hesitation] Including submodule: star_tracking")
    add_subdirectory(star_tracking)
endif()

if(BUILD_TD3_ATTITUDE)
    message(STATUS "[Hesitation] Including submodule: td3_attitude")
    add_subdirectory(td3_attitude)
endif()

if(BUILD_THRUST_VECTORING)
    message(STATUS "[Hesitation] Including submodule: thrust_vectoring")
    add_subdirectory(thrust_vectoring)
endif()
