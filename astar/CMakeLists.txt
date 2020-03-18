cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
project(astar)

add_library(${PROJECT_NAME} STATIC src/astar.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME} PUBLIC utils)

add_library(global_planner ALIAS ${PROJECT_NAME})

add_subdirectory(test)
