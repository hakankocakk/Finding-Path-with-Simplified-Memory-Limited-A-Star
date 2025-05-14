# Simplified Memory Limited A* Path Finding Algorithm

This project includes SMA* - Simplified Memory Limited A* search algorithm written in C++ to find the shortest path on grid-based maps. The program visualizes the search process step by step in the console.

## Problem Description

The main problem solved by this project is to find the shortest path from a given starting point to a target point on a two-dimensional grid map full of obstacles. The algorithm focuses on working within limited memory resources using strategies such as node pruning when memory usage reaches a certain limit, making it suitable for large maps or search areas where classical A* may exceed the available memory.

## Features

* Path finding on a grid map.

* Handling of traversable and impassable (obstacle) areas.

* Implementation of the basic principles of the A* search algorithm.

* Limiting and managing memory usage; implementation of a memory management mechanism (pruning of less promising nodes) when a predefined limit is reached.
* Step-by-step console visualization of the search process.

* Easy configuration of start and destination points.

* User interaction to step through the algorithm or let it run automatically.

## File Structure

* `main.cpp`: Main entry point of the program. Defines map dimensions and obstacle data (`pMap`), sets start and destination coordinates, calls pathfinding (`FindPath`) function, and processes user input for step-by-step visualization of the search process.
* `sm-astar.h`, `sm-astar.cpp`: These files contain the basic logic of the A* algorithm. They implement the Node and Map classes, cost calculations, node expansion (discovering neighbors), memory management mechanisms, and the main search loop.
* `output.h`, `output.cpp`: Contains functions (`Paint`, `RenderMap`) used to visualize the search process and the map in the console. Prints the map and search progress in a readable format.

## Compile

Compile the project with a C++ compiler (e.g. g++).
```bash
g++ -std=c++11 main.cpp src/sm-astar.cpp src/output.cpp -o program -I src -Wall
```

Run the program.
```bash
./program
```
