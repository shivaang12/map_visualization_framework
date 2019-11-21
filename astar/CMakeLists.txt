cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
project(astar)

add_library(astar STATIC
    include/astar.hpp
    src/astar.cpp
)

target_include_directories(astar PUBLIC
    include
    ../utils
)
