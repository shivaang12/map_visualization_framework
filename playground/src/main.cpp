#include <cmath>
#include <iostream>
#include "map_core/frame.hpp"
#include "astar/astar.hpp"

void generateCirclePoints(int               center_x,
                          int               center_y,
                          int               radius,
                          std::vector<int>& points,
                          int               width) {
  for (int i = center_x - radius; i < center_x + radius; i++) {
    for (int j = center_y - radius; j < center_y + radius; j++) {
      if (hypot(i - center_x, j - center_y) <= radius) {
        points.emplace_back(i + (j * width));
      }
    }
  }
}

int main() {
  {
    int width  = 800;
    int height = 400;
    std::vector<int> points;
    std::shared_ptr<planner::GlobalPlanner> planner_ = std::make_shared<planner::Astar>();
    map_viz::Framework fw(width, height);
    generateCirclePoints(width / 2, height / 2, 60, points, width);
    fw.setGlobalPlanner(planner_);
    fw.initFramework();
    planner_->setStartPoint(0, 0);
    planner_->setGoalPoint((width / 2) + 65, (height / 2) - 30);
    fw.globalPlannerDrawRate(30);
    fw.loadObstacles(points, std::string("red"));
    fw.spinCurrentState();
  }
  return 0;
}
