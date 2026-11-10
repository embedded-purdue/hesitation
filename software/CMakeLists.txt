add_library(thrust_vectoring thrust_vectoring/thrust.cpp)
target_include_directories(thrust_vectoring PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(thrust_vectoring PUBLIC cxx_std_17)
target_compile_options(thrust_vectoring PRIVATE -Wall -Wextra -Wpedantic)

