cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
project(utils)

add_library(utils INTERFACE)
target_include_directories(utils INTERFACE include)
