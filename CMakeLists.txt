cmake_minimum_required(VERSION 3.15)
#cmake_minimum_required(VERSION 3.30)

project(ky)

set(CMAKE_CXX_STANDARD 23)
#set(CMAKE_CXX_MODULE_STD 1)

add_executable(ky ky.cpp)
target_compile_options(ky PUBLIC -fopenmp)

find_package(OpenMP)
target_link_libraries(ky PUBLIC OpenMP::OpenMP_CXX)