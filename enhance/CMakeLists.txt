cmake_minimum_required(VERSION 3.15)

project(kye)
set(CMAKE_CXX_STANDARD 23)

add_executable(kye kye.cpp)
target_compile_options(kye PUBLIC -fopenmp)

find_package(OpenMP)
target_link_libraries(kye PUBLIC OpenMP::OpenMP_CXX)