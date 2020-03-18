#include <gtest/gtest.h>
#include <astar/astar.hpp>

TEST(astarTest, setStartGoalTest) {
  planner::Astar astar;
  astar.initialize(10, 10);
  astar.setGoalPoint(9, 9);
  astar.setStartPoint(1, 1);
  int goal_x{0};
  int goal_y{0};
  int start_x{0};
  int start_y{0};
  astar.getGoalPoint(goal_x, goal_y);
  astar.getStartPoint(start_x, start_y);
  EXPECT_EQ(goal_x, 9);
  EXPECT_EQ(goal_y, 9);
  EXPECT_EQ(start_x, 1);
  EXPECT_EQ(start_y, 1);
}