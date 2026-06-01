cmake_minimum_required(VERSION 3.16)
project(calg_cpp_backend LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

add_executable(calg_cpp_benchmark calg_cpp_benchmark.cpp)
target_link_libraries(calg_cpp_benchmark PRIVATE Eigen3::Eigen)
target_compile_options(calg_cpp_benchmark PRIVATE -O3 -DNDEBUG -march=native)
