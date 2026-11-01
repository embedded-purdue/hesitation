cmake_minimum_required(VERSION 3.16)
project(nep_controls LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)   # turns on testing (BUILD_TESTING=ON)

# ---- Modules ----
add_subdirectory(software/thrust_vectoring)
# add_subdirectory(software/td3_attitude)

# ---- Top-level app ----
add_executable(nep_controls main.cpp)
target_include_directories(nep_controls PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/app/src
)
target_link_libraries(nep_controls PRIVATE thrust_vectoring)