/**
 * Copyright 2020 Shivang Patel
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <cmath>
#include <iostream>
#include "map_core/frame.hpp"
#include "astar/astar.hpp"

void generateCirclePoints(int center_x,
                          int center_y,
                          int radius,
                          std::vector<int>& points,
                          int width) {
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
