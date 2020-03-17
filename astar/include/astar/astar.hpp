#ifndef ASTAR_INCLUDE_ASTAR_ASTAR_CPP
#define ASTAR_INCLUDE_ASTAR_ASTAR_CPP
#include <vector>
#include <memory>
#include "utils/global_planner.hpp"

namespace planner {
struct comparator {
  bool operator()(const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) {
    return lhs.second > rhs.second;
  }
};

class Astar : public GlobalPlanner {
public:

  Astar();
  void                             initialize(const int width,
                                              const int height);
  void                             setStartPoint(const int start_x,
                                                 const int start_y) override;
  void                             setGoalPoint(const int goal_x,
                                                const int goal_y) override;
  std::vector<int>                 makePlan() override;
  void                             loadObstacleInfo(
    const std::shared_ptr<std::vector<bool> >obstacle_info) override;
  std::vector<std::pair<int, int> >makePlanCoordinate();

private:

  double          calcGCost(const int& cell1,
                            const int& cell2) const;
  double          getHCost(const int& cell) const;
  bool            isValid(const int& cell) const;
  std::vector<int>getNeighbors(const int& current_cell) const;
  void            convertCoordinateToCell(const int& x,
                                          const int& y,
                                          int& cell) const;
  void            convertCellToCoordinate(const int& cell,
                                          int& x,
                                          int& y) const;

  int width_;
  int height_;

  int start_;
  int goal_;
  bool obstacle_flag;
  std::shared_ptr<std::vector<bool> >obstacle_info_ptr_;
};
}
#endif //ASTAR_INCLUDE_ASTAR_ASTAR_CPP
