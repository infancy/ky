cmake_minimum_required(VERSION 3.15)

project(smallpt)

set(CMAKE_CXX_STANDARD 23)

file(GLOB src_files "*.cpp")
foreach (file ${src_files})
    string(REGEX REPLACE ".+/(.+)\\..*" "\\1" name ${file})
    if(name STREQUAL "smallpt_kernel")
        set(name "smallpt_kernel_cpu")
    endif()
    message("name: ${name}, file: ${file}")

    add_executable(${name} ${file})
    target_compile_options(${name} PUBLIC -fopenmp)

    find_package(OpenMP)
    target_link_libraries(${name} PUBLIC OpenMP::OpenMP_CXX)
endforeach ()

enable_language(CUDA)
#add_definitions(-D USE_CUDA)
add_executable(smallpt_kernel_cuda smallpt_kernel.cu)
target_compile_definitions(smallpt_kernel_cuda PRIVATE USE_CUDA=1)
