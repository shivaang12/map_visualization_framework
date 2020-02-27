#include <future>
#include <iostream>
#include <frame.hpp>

namespace map_viz{

point_2d::point_2d(int x, int y) {
    this->x = x;
    this->y = y;
}

Framework::Framework(int width, int height):
    width_(width),
    height_(height),
    sdl_renderer_(NULL),
    sdl_window_(NULL),
    is_polling_(true),
    is_obstacle_info_loaded_(false),
    gp_ptr_(nullptr),
    start_(0),
    goal_(0) {}

Framework::~Framework() {
    SDL_DestroyRenderer(sdl_renderer_);
    SDL_DestroyWindow(sdl_window_);
}

Framework::Framework():
    width_(10),   // Default
    height_(10),    // Default
    sdl_renderer_(NULL),
    sdl_window_(NULL),
    is_polling_(true),
    is_obstacle_info_loaded_(false),
    gp_ptr_(nullptr),
    start_(0),
    goal_(0) {}

void
Framework::initFramework() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(width_-1, height_-1, 0, &sdl_window_, &sdl_renderer_);
    SDL_SetRenderDrawColor(sdl_renderer_, 0, 0, 0, 0);
    SDL_RenderClear(sdl_renderer_);
    SDL_RenderPresent(sdl_renderer_);
    if (gp_ptr_) {
        gp_ptr_->initialize(width_, height_);
    }
}

void
Framework::spinCurrentState() {
    std::cout << "[SPIN] STARTING\n";
    std::vector<int> path;
    if (gp_ptr_) {
        path = gp_ptr_->makePlan();
        drawOnScreen(path, "green");
    }
    // std::cout << "[PATH] LAST NUMBER IS " << path[path.size()-1] << '\n';
    updateDisplay();
    auto fut = std::async([](bool &is_polling_){
        while (is_polling_) {
            SDL_Event event;
            SDL_PollEvent(&event);
            if (event.type == SDL_QUIT) {
                is_polling_ = false;
            }
            SDL_Delay(10);
        }
    }, std::ref(this->is_polling_));

    while (is_polling_) {
        SDL_Delay(10);
    }

    fut.wait();
}

void
Framework::show() {
    this->initFramework();
    this->spinCurrentState();
}

void
Framework::loadObstacles(std::vector<int> &points, std::string color) {
    std::shared_ptr<std::vector<bool>> obst_ptr(new std::vector<bool>(width_*height_,0));
    for (auto& point : points) {
        // drawPointWithoutColor(point);
        (*obst_ptr)[point] = 1;
    }
    if (gp_ptr_) {
        gp_ptr_->loadObstacleInfo(obst_ptr);
    }
    drawOnScreen(points, color);
}

void
Framework::drawOnScreen(std::vector<int> &points, std::string color) {
    setRendererColor(color);
    for (auto& point : points) {
        drawPointWithoutColor(point);
    }
    resetRendererColor();
}

void
Framework::drawPointWithoutColor(int &point) {
    int x, y;
    convertCellToCoordinate(point, x, y);
    SDL_RenderDrawPoint(sdl_renderer_, x, y);
}

void
Framework::setRendererColor(std::string &color) {
    if (color == "red") {
        SDL_SetRenderDrawColor(sdl_renderer_, 255, 0, 0, 255);
    } else if (color == "green") {
        SDL_SetRenderDrawColor(sdl_renderer_, 0, 255, 0, 255);
    } else {
        SDL_SetRenderDrawColor(sdl_renderer_, 255, 255, 255, 255);
    }
}

void
Framework::resetRendererColor() {
    SDL_SetRenderDrawColor(sdl_renderer_, 0, 0, 0, 0);
}

void
Framework::updateDisplay() {
    SDL_RenderPresent(sdl_renderer_);
}

void
Framework::convertCellToCoordinate(int cell, int &x, int &y) {
    x = cell%width_;
    y = cell/width_;
    return;
}

void
Framework::convertCoordinateToCell(int x, int y, int &cell) {
    cell = (y * width_) + x;
    return;
}

void
Framework::setGlobalPlanner(std::shared_ptr<planner::GlobalPlanner> gp_ptr) {
    gp_ptr_ = gp_ptr;
}

}