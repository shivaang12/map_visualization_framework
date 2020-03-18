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

/** @file astar.hpp
 *  @brief Definition of class Astar
 *  This file contains definitions of class Astar which implements Astar planner
 *
 *  @author Shivang Patel
 */

#ifndef ASTAR_INCLUDE_ASTAR_ASTAR_CPP
#define ASTAR_INCLUDE_ASTAR_ASTAR_CPP
#include <vector>
#include <memory>
#include "utils/global_planner.hpp"

namespace planner {
/**
 * @struct planner::comparator
 * @brief Struct as a comparator for priority queue
 */
struct comparator {
  /**
   * @brief Operator() overload to enable sorting for priority queue
   *
   * @param[in] lhs Refers to left most param
   * @param[in] rhs Refers to right most param
   */
  bool operator()(const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) {
    return lhs.second > rhs.second;
  }
};

/**
 * @class planner::Astar
 * @brief Implements basic astar heuristic base planner
 */
class Astar : public GlobalPlanner {
public:

  /**
   * @brief A constructor for the class planner::Astar
   */
  Astar();

  /**
   * @brief Function to initialize member variables
   *
   * @param[in] width Width of the area
   * @param[in] height height of the area
   *
   * @return Void
   */
  void initialize(const int width,
                  const int height);

  /**
   * @brief Setter function which sets start position for planner
   *
   * @param[in] start_x Start position's x coordinate
   * @param[in] start_y Start position's y coordinate
   *
   * @return Void
   */
  void setStartPoint(const int start_x,
                     const int start_y) override;

  /**
   * @brief Setter function which sets goal position for planner
   *
   * @param[in] goal_x Goal position's x coordinate
   * @param[in] goal_y Goal position's y coordinate
   *
   * @return Void
   */
  void setGoalPoint(const int goal_x,
                    const int goal_y) override;

  /**
   * @brief Generates plan from given start and goal points
   */
  std::vector<int>makePlan() override;

  /**
   * @brief This method loads obstacle information
   *
   * @param[in] obstacle_info Shared pointer to vector of bool which contains obstacle infomation
   */
  void            loadObstacleInfo(
    const std::shared_ptr<std::vector<bool> >obstacle_info) override;

  /**
   * @brief This method converts cells into x-y coordinates and return vector of pair
   *
   * @return Vector of pairs containing x-y coordinates of the plan
   */
  std::vector<std::pair<int, int> >makePlanCoordinate();

  /**
   * @brief Method calculate cost-to-come (G) for Astar
   *
   * @param[in] cell1 Cell
   * @param[in] cell2 Cell
   *
   * @return cost-to-come in double
   */
  double calcGCost(const int& cell1,
                   const int& cell2) const;

  /**
   * @brief Method calculates Heuristic Cost (H) for Astar
   *
   * @param[in] cell Cell
   *
   * @return Heuristic cost in double
   */
  double          getHCost(const int& cell) const;

  /**
   * @brief Method checks if the cell is not in obstacle
   *
   * @param[in] cell Cell
   *
   * @return True if cell is in obstacle else False
   */
  bool            isValid(const int& cell) const;

  /**
   * @brief Method retrives the neighbor cells for the given cells
   *
   * @param[in] current_cell Cell
   *
   * @return Vector of cell which are neighbor to current_cell
   */
  std::vector<int>getNeighbors(const int& current_cell) const;

  /**
   * @brief Converts x-y coordinate to cell
   *
   * @param[in] x x-coordinate
   * @param[in] y y-coordinate
   *
   * @param[out] cell Cell representing x-y coordinate
   */
  void            convertCoordinateToCell(const int& x,
                                          const int& y,
                                          int& cell) const;

  /**
   * @brief Coverts cell to x-y coordinate
   *
   * @param[in] cell Cell
   *
   * @param[out] x x-coordinate
   * @param[out] y y-coordinate
   */
  void convertCellToCoordinate(const int& cell,
                               int& x,
                               int& y) const;
  /**
   * @brief Getter method gets start point
   * 
   * @param[out] start_x X coordinate for start point
   * @param[out] start_y Y coordinate for start point
   */
  void getStartPoint(int& start_x, int& start_y) const;
  /**
   * @brief Getter method gets goal point
   * 
   * @param[out] goal_x X coordinate for goal point
   * @param[out] goal_y Y coordinate for goal point
   */
  void getGoalPoint(int& goal_x, int& goal_y) const;
private:
  int width_;
  int height_;

  int start_;
  int goal_;
  bool obstacle_flag;
  std::shared_ptr<std::vector<bool> >obstacle_info_ptr_;
};
}
#endif //ASTAR_INCLUDE_ASTAR_ASTAR_CPP
