cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project (main_project)

add_subdirectory(astar)
add_subdirectory(map_viz_framework)
add_subdirectory(playground)

