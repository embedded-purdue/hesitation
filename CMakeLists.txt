
cmake_minimum_required(VERSION 3.20)

project(nep_controls
    VERSION 0.1.0
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep all executables in a predictable location.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Enable testing support.
include(CTest)

# ==============================================================================
# PROJECT MODULES
# ==============================================================================

# Engineering subsystems.
# Each subsystem manages its own build configuration.
add_subdirectory(software)

# Desktop application.
# All GUI, rendering and desktop dependencies are managed here.
add_subdirectory(app)
