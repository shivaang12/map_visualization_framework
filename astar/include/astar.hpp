#pragma once
#include <vector>
#include <memory>
#include <utils/global_planner.hpp>

namespace planner {
struct comparator {
  bool operator()(const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) {
    return lhs.second > rhs.second;
  }
};

class Astar : public GlobalPlanner {
public:

  Astar();
  void                             initialize(int width,
                                              int height);
  void                             setStartPoint(int start_x,
                                                 int start_y) override;
  void                             setGoalPoint(int goal_x,
                                                int goal_y) override;
  std::vector<int>                 makePlan() override;
  void                             loadObstacleInfo(
    std::shared_ptr<std::vector<bool> >obstacle_info) override;
  std::vector<std::pair<int, int> >makePlanCoordinate();

private:

  double          calcGCost(int cell1,
                            int cell2);
  double          getHCost(int cell);
  bool            isValid(int cell);
  std::vector<int>getNeighbors(int current_cell);
  void            convertCoordinateToCell(int  x,
                                          int  y,
                                          int& cell);
  void            convertCellToCoordinate(int  cell,
                                          int& x,
                                          int& y);

  int width_;
  int height_;

  int start_;
  int goal_;
  bool obstacle_flag;
  std::shared_ptr<std::vector<bool> >obstacle_info_ptr_;
};
}
