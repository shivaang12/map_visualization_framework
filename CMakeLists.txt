cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project (project_mvf) # Map Visualizing Framework

add_subdirectory(utils)
add_subdirectory(astar)
add_subdirectory(map_core)
add_subdirectory(playground)
