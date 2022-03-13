cmake_minimum_required(VERSION 3.15)

project(smallpt)

set(CMAKE_CXX_STANDARD 23)
add_executable(smallpt nanopt.cpp)
target_compile_options(smallpt PUBLIC -fopenmp)

find_package(OpenMP)
target_link_libraries(smallpt PUBLIC OpenMP::OpenMP_CXX)