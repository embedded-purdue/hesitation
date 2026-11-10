cmake_minimum_required(VERSION 3.20)

project(Hesitation
    VERSION 0.1.0
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

include(CTest)

# ==============================================================================
# PROJECT MODULES
# ==============================================================================

add_subdirectory(software)
add_subdirectory(messaging)
add_subdirectory(app)
