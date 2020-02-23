cmake_minimum_required(VERSION 3.0)
project(RRT)

add_compile_options(-Wall)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)


add_executable(rrt main.cpp rrt.cpp)
