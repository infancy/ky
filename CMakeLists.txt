cmake_minimum_required(VERSION 3.15)

project(ky)

set(CMAKE_CXX_STANDARD 23)
add_executable(ky ky.cpp)
target_compile_options(ky PUBLIC -fopenmp)

find_package(OpenMP)
target_link_libraries(ky PUBLIC OpenMP::OpenMP_CXX)