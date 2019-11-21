cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project (map_viz_framework)

INCLUDE(FindPkgConfig)

PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)

add_executable(shell-app src/main.cpp src/frame.cpp)

target_include_directories(shell-app PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(shell-app ${SDL2_LIBRARIES} astar)
