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

TEST(astarTest, utilityFuctionTests) {
  planner::Astar astar;
  astar.initialize(10, 10);
  astar.setGoalPoint(9, 9);
  astar.setStartPoint(1, 1);
  int cell1{0};
  int cell2{10};
  int cell3{81};
  int goal_x1{0};
  int goal_x2{0};
  int goal_x3{0};
  int goal_y1{0};
  int goal_y2{0};
  int goal_y3{0};
  astar.convertCellToCoordinate(cell1, goal_x1, goal_y1);
  EXPECT_EQ(goal_x1, 0);
  EXPECT_EQ(goal_y1, 0);
  
  astar.convertCellToCoordinate(cell2, goal_x2, goal_y2);
  EXPECT_EQ(goal_x2, 0);
  EXPECT_EQ(goal_y2, 1);

  astar.convertCellToCoordinate(cell3, goal_x3, goal_y3);
  EXPECT_EQ(goal_x3, 1);
  EXPECT_EQ(goal_y3, 8);

  int cell4{0};
  int cell5{0};
  int cell6{0};

  astar.convertCoordinateToCell(goal_x1, goal_y1, cell4);
  EXPECT_EQ(cell4, cell1);

  astar.convertCoordinateToCell(goal_x2, goal_y2, cell5);
  EXPECT_EQ(cell5, cell2);

  astar.convertCoordinateToCell(goal_x3, goal_y3, cell6);
  EXPECT_EQ(cell6, cell3);

  auto hcost = astar.getHCost(cell3);
  EXPECT_NEAR(hcost, 8.0, 0.1);

  auto gcost = astar.calcGCost(cell4, cell5);
  EXPECT_NEAR(gcost, 1.0, 0.1);
}
