# Map Vizualization Framework [![Build Status](https://travis-ci.org/shivaang12/map_visualization_framework.svg?branch=master)](https://travis-ci.org/shivaang12/map_visualization_framework)

A vizualization framework for people who are working in domain of "motion planning" and want to vizualize the planner that they have developed.

## Overview

The motivation of this project is born out of research lab where I worked on developing global planners. One time I had to present my work and the night before I had no graphical representation of the planner I was working on. An idea of having ready to go framework was born (this project of course).

## How to use this

Well the project is still in its initial stage and currently it only supports global planner. But before you further, please look for build guidlines.

### Steps

- First you need a planner (or you can use default planner)
- You need to put your planner into a wrapper of `global_planner.hpp`. This file is in include directory of `utils` module. (for reference see `astar` module)
- Now you need to change your (or other) CMakeLists.txt files.
- In your wrapper module's CMakeLists.txt, make sure you add and `ALIAS` lib of name `global_planner`. (again see `astar` module)
- Add `add_subdirectory` to the main `CMakeLists.txt` in place of `astar`.

And you are good to go.

## Build

### Requirements
- SDL2
- cmake

### Steps

```bash
$ git clone https://github.com/shivaang12/map_visualization_framework.git
$ cd ~/<path/to/this/repo>
$ mkdir build && cd buid
$ cmake ..
$ make
```
And to run, simply type further
```bash
$ ./playground/playground
```

## Contribution

Any improvements and features are always welcome. I will soon post the proper guidelines and targeted feature lists. So stay tuned! If you like this work, don't forget to starred this repo. Happy Coding