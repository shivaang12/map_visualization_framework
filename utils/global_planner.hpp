#pragma once
#include <vector>

namespace planner {

class GlobalPlanner {
public:
    virtual ~GlobalPlanner(){}
    virtual std::vector<int> makePlan()=0;
    virtual void initialize(int width, int height)=0;
    virtual void setStartPoint(int start_x, int start_y)=0;
    virtual void setGoalPoint(int goal_x, int goal_y)=0;
    virtual void loadObstacleInfo(std::shared_ptr<std::vector<bool>> obstacle_info)=0;
};

}