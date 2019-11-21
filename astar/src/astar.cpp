#include <utility>
#include <unordered_map>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>
#include <iostream>
#include <queue>
#include <memory>
#include <astar.hpp>

namespace planner{

Astar::Astar():
    width_(0),
    height_(0),
    start_(0),
    goal_(0),
    obstacle_flag(0),
    obstacle_info_ptr_(NULL) {}

void Astar::initialize(int width, int height) {
    width_ = width;
    height_ = height;
}

void Astar::setStartPoint(int x, int y) {
    convertCoordinateToCell(x, y, start_);
}

void Astar::setGoalPoint(int x, int y) {
    convertCoordinateToCell(x, y, goal_);
    std::cout<<"[ASTAR] GOAL is " << goal_ << '\n';
}

void Astar::convertCellToCoordinate(int cell, int &x, int &y) {
    x = cell%width_;
    y = cell/width_;
    return;
}

void Astar::convertCoordinateToCell(int x, int y, int &cell) {
    cell = (y * width_) + x;
    return;
}

std::vector<int> Astar::makePlan() {
    std::vector<int> path;

    // Checking if goal and start is valid
    if (obstacle_info_ptr_) {
        if (!isValid(start_) || !isValid(goal_)) {
            return path;
        }
    }
    // Open List
    std::priority_queue<
        std::pair<int, double>,
        std::vector<std::pair<int, double>>,
        comparator> open_list;
    open_list.emplace(start_, 0);

    // Dictionary
    std::unordered_map<int, int> parent_dict;
    parent_dict[start_] = start_;

    // cost to come (g cost)
    std::unordered_map<int, double> gCost;
    gCost[start_] = 0.0;

    // exhaustive loop
    while (!open_list.empty()) {
        auto current_cell = open_list.top();
        open_list.pop();

        if (current_cell.first == goal_) {
            break;
        }

        for (auto& neighbor : getNeighbors(current_cell.first)) {
            double new_g_cost = gCost[current_cell.first] +
                calcGCost(neighbor, current_cell.first) + getHCost(neighbor);
            if (gCost.find(neighbor) == gCost.end() || new_g_cost < gCost[neighbor]) {
                gCost[neighbor] = new_g_cost;
                double f_cost = new_g_cost + getHCost(neighbor);
                open_list.emplace(neighbor, f_cost);
                parent_dict[neighbor] = current_cell.first;
            }
        }
    }

    // Building path
    int current_cell = goal_;
    path.emplace_back(goal_);
    while (current_cell != start_) {
        current_cell = parent_dict[current_cell];
        path.emplace_back(current_cell);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> Astar::getNeighbors(int current_cell) {
    int current_x, current_y;
    convertCellToCoordinate(current_cell, current_x, current_y);
    std::vector<int> return_vector;
    return_vector.reserve(8);

    for (int i=current_cell-width_; i <= current_cell+width_; i=i+width_) {
        for (int j=-1; j<=1; j++) {
            // Handle Corner cases
            // Right corner
            if (current_cell%width_ == width_-1) {
                if (j>0) {
                    continue;
                }
            }
            // Left corner
            if (current_cell%width_ == 0) {
                if (j<0) {
                    continue;
                }
            }

            if ((i+j >= 0) && (i+j < width_*height_)) {
                if (isValid(i+j) && (current_cell != (i+j))) {
                    return_vector.emplace_back(i+j);
                }
            }
        }
    }

    return return_vector;
}

bool Astar::isValid(int cell) {
    if (obstacle_info_ptr_) {
        return !(*obstacle_info_ptr_)[cell];
    }
    return true;
}

void Astar::loadObstacleInfo(std::shared_ptr<std::vector<bool>> obstacle_info){
    obstacle_info_ptr_ = obstacle_info;
}

double Astar::getHCost(int cell) {
    int cell_x, cell_y;
    int goal_x, goal_y;

    convertCellToCoordinate(cell, cell_x, cell_y);
    convertCellToCoordinate(goal_, goal_x, goal_y);

    return std::hypot(cell_x - goal_x, cell_y - goal_y);
}

double Astar::calcGCost(int cell1, int cell2) {
    int cell1_x, cell1_y;
    int cell2_x, cell2_y;

    convertCellToCoordinate(cell1, cell1_x, cell1_y);
    convertCellToCoordinate(cell2, cell2_x, cell2_y);

    return std::hypot(cell1_x - cell2_x, cell1_y - cell2_y);
}

std::vector<std::pair<int, int>> Astar::makePlanCoordinate() {
    std::vector<std::pair<int, int>> return_vector;
    for (auto &cell : makePlan()) {
        int x, y;
        convertCellToCoordinate(cell, x, y);
        return_vector.emplace_back(x, y);
    }
    return return_vector;
}

}