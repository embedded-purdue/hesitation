cmake_minimum_required(VERSION 3.16)
project(thrust_vectoring VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Header-only library for thrust vectoring allocation algorithms
add_library(thrust_vectoring INTERFACE)
target_include_directories(thrust_vectoring INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Main simulation / demonstration runner
add_executable(thrust_vectoring_main src/main.cpp)
target_link_libraries(thrust_vectoring_main PRIVATE thrust_vectoring)
