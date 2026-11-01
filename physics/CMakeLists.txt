cmake_minimum_required(VERSION 3.16)

project(HesitationPhysics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(one_loop_test
    system_internal/cte1.cpp
    system_internal/cte2.cpp
    system_internal/test_one_loop.cpp
)