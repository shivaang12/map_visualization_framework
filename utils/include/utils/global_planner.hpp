#pragma once
#include <vector>

namespace planner {
class GlobalPlanner {
public:

  virtual ~GlobalPlanner() {}

  virtual std::vector<int>makePlan() = 0;
  virtual void            initialize(const int width,
                                     const int height) = 0;
  virtual void            setStartPoint(const int start_x,
                                        const int start_y) = 0;
  virtual void            setGoalPoint(const int goal_x,
                                       const int goal_y) =
    0;
  virtual void            loadObstacleInfo(const std::shared_ptr<std::vector<bool> >obstacle_info) =
    0;
};
}
