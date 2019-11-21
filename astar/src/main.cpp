#include <iostream>
#include <cmath>
#include <memory>
#include <astar.hpp>

int main() {
    planner::Astar planner;
    planner.initialize(3, 3);
    planner.setGoalPoint(2, 2);
    planner.setStartPoint(0,0);
    std::shared_ptr<std::vector<bool>> obst(new std::vector<bool>(3*3, 0));
    (*obst)[4] = 1;
    (*obst)[5] = 1;
    planner.loadObstacleInfo(obst);
    // for (auto point2d : planner.getNeighbors(planner.start_)) {
    //     std::cout << point2d << "\n";
    // }
    for (auto point2d : planner.makePlanCoordinate()) {
        std::cout << point2d.first << " " << point2d.second << "\n";
    }
}