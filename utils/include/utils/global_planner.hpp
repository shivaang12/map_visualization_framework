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

/** @file global_planner.hpp
 *  @brief Definition of class GlobalPlanner
 *  This file contains definitions of class GlobalPlanner which is base class for global planners
 *
 *  @author Shivang Patel
 */

#ifndef UTILS_INCLUDE_UTILS_GLOBAL_PLANNER_HPP
#define UTILS_INCLUDE_UTILS_GLOBAL_PLANNER_HPP
#include <vector>

namespace planner {
/**
 * @class planner::GlobalPlanner
 * 
 * @brief Base class for global planners
 */
class GlobalPlanner {
public:
  /**
   * @brief Virtual class destructor
   */
  virtual ~GlobalPlanner() {}
  /**
   * @brief The overriding method returns the plan for the given start and end points
   */
  virtual std::vector<int>makePlan() = 0;
  /**
   * @brief Overriding method initialize the required variables
   * 
   * @param[in] width Width of the environment
   * @param[in] height Height of the environment
   */
  virtual void            initialize(const int width,
                                     const int height) = 0;
  /**
   * @brief Overriding method would set the start point
   * 
   * @param[in] start_x X coordinate of start point
   * @param[in] start_y Y coordinates of start point
   */
  virtual void            setStartPoint(const int start_x,
                                        const int start_y) = 0;
  /**
   * @brief Overriding method would set the end point or goal point
   * 
   * @param[in] goal_x X coordinate of goal point
   * @param[in] goal_y Y coordinates of goal point
   */
  virtual void            setGoalPoint(const int goal_x,
                                       const int goal_y) =
    0;
  /**
   * @brief Overriding method loads obstacle info
   * 
   * @param[in] obstacle_info Pointer of obstacle vector
   */
  virtual void            loadObstacleInfo(const std::shared_ptr<std::vector<bool> >obstacle_info) =
    0;
};
}
#endif //UTILS_INCLUDE_UTILS_GLOBAL_PLANNER_HPP
