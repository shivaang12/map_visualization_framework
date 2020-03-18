set(GTEST_SHUFFLE 1)

add_executable(
    astar-test
    main.cpp
    test_astar.cpp
)

target_link_libraries(astar-test PUBLIC gtest)
target_link_libraries(astar-test PUBLIC astar)