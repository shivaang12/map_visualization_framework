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

/** @file frame.hpp
 *  @brief Definition of class Framework
 *  This file contains definitions of class Framework which implements graphics for the program
 *
 *  @author Shivang Patel
 */
#ifndef MAP_CORE_INCLUDE_MAP_CORE_FRAME_HPP_
#define MAP_CORE_INCLUDE_MAP_CORE_FRAME_HPP_

#include <SDL2/SDL.h>
#include <string>
#include <vector>
#include <memory>
#include "utils/global_planner.hpp"

namespace map_viz {
/**
 * @struct map_viz::point_2d
 * @brief Struct to store data for 2d coordinate
 */
struct point_2d {
  int x, y;
  point_2d() = default;
  point_2d(int x,
           int y);
};
/**
 * @class map_viz::Framework
 */
class Framework {
public:

  /**
   * @brief Constructor for class map_viz::Framework
   * 
   * @param[in] width Width of the environment
   * @param[in] height Height of the environment
   */
  Framework(int width,
            int height);
  /**
   * @brief Constructor for class map_viz::Framework
   */
  Framework();
  /**
   * @brief Deleted copy constructor
   */
  Framework(const Framework&) = delete;
  /**
   * @brief Deleted copy constructor
   */
  Framework& operator=(const Framework&) = delete;
  /**
   * @brief Destructor
   */
  ~Framework();
  /**
   * @brief This is main method
   */
  void show();
  /**
   * @brief This method loads obstacle points
   * 
   * @param[in] points Vector of obstacle points
   * @param[in] color Color of the obstacle
   */
  void loadObstacles(const std::vector<int>& points,
                     std::string color = "red");
  /**
   * @brief Initialize the member variables
   */
  void initFramework();
  /**
   * @brief This Method handles animation sequence
   */
  void spinCurrentState();
  /**
   * @brief Setter for global planner
   * 
   * @param[in] gp_ptr Global Planner pointer
   */
  void setGlobalPlanner(std::shared_ptr<planner::GlobalPlanner>gp_ptr);
  /**
   * @brief This method sets global planner drawing rate
   */
  void globalPlannerDrawRate(unsigned int rate);

private:
  /**
   * @brief Method updates global planner path while drawing
   */
  void updateGlobalPlanner(const unsigned int time);
  /**
   * @brief Draws point on the sdl window screen
   */
  void drawPointOnScreen(int points,
                         std::string color);
  /**
   * @brief Draws vector of points on the sdl window screen
   */
  void drawVectorOnScreen(const std::vector<int>& points,
                          std::string color);
  /**
   * @brief Converts cell to coordinates (1d to 2d)
   * 
   * @param[in] cell Cell
   * 
   * @param[out] x X coordinate
   * @param[out] y Y coordinate
   */
  void convertCellToCoordinate(const int& cell,
                               int& x,
                               int& y);
  /**
   * @brief Converts coordinates to cell
   * 
   * @param[in] x X coordinate
   * @param[in] y Y coordinate
   * 
   * @param[out] cell Cell
   */
  void convertCoordinateToCell(const int& x,
                               const int& y,
                               int& cell);
  /**
   * @brief This method updates display of sdl window
   */
  void updateDisplay();
  /**
   * @brief Sets renderer color
   * 
   * @param[in] color Color
   */
  void setRendererColor(std::string& color);
  /**
   * @brief This method draws point with set color
   * 
   * @param[in] point_2d A 2d Point
   */
  void drawPointWithoutColor(const int& point_2d);
  /**
   * @brief Method resets renderer color to default color which is black
   */
  void resetRendererColor();

  int width_;
  int height_;
  SDL_Renderer *sdl_renderer_;
  SDL_Window *sdl_window_;
  bool is_polling_;
  bool is_obstacle_info_loaded_;
  unsigned int gp_draw_time_;
  std::shared_ptr<planner::GlobalPlanner>gp_ptr_;
  int start_;
  int goal_;
};
} // namespace map_viz
#endif // MAP_CORE_INCLUDE_MAP_CORE_FRAME_HPP_
