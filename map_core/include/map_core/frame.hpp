#pragma once
#include <SDL2/SDL.h>
#include <string>
#include <vector>
#include <memory>
#include "utils/global_planner.hpp"

namespace map_viz {
struct point_2d {
  int x, y;
  point_2d() = default;
  point_2d(int x,
           int y);
};

class Framework {
public:

  // Constructors
  Framework(int width,
            int height);
  Framework();
  Framework(const Framework&) = delete;

  ~Framework();

  void show();
  void loadObstacles(std::vector<int>& points,
                     std::string       color = "red");
  void initFramework();
  void spinCurrentState();
  void setGlobalPlanner(std::shared_ptr<planner::GlobalPlanner>gp_ptr);
  void globalPlannerDrawRate(unsigned int rate);

private:

  void updateGlobalPlanner(const unsigned int time);
  void drawPointOnScreen(int         points,
                         std::string color);
  void drawVectorOnScreen(std::vector<int>& points,
                          std::string       color);
  void convertCellToCoordinate(int  cell,
                               int& x,
                               int& y);
  void convertCoordinateToCell(int  x,
                               int  y,
                               int& cell);
  void updateDisplay();
  void setRendererColor(std::string& color);
  void drawPointWithoutColor(int& point_2d);
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
}
