/* Copyright (C)
 * 2019 - Bhargav Dandamudi
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the 'Software'), to deal in the Software without
 * restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so,subject to
 * the following conditions:
 * The above copyright notice and this permission notice shall
 * be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED ''AS IS'', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM,OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
/**
 * @file optimalPlanner.h
 * @brief  Optimal Planner using astar algorithm to reach goal
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-04-04
 */
#include "./node.h"
#include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <queue>
#include <stdlib.h>
#include <utility>
#include <vector>
// defining types for ease in usage for programs
typedef std::pair<double, std::pair<int, int>> DoublePair;

/* ----------------------------------------------------------------*/
/**
 * @brief  Optimal Planner class members declaration
 *          we are using Astar algorithm for finding optimal path
 *
 */
/* ----------------------------------------------------------------*/
class optimalPlanner {
public:
  /* ----------------------------------------------------------------*/
  /**
   * @brief Default Constructor for the class
   */
  /* ----------------------------------------------------------------*/
  optimalPlanner();
  /* ----------------------------------------------------------------*/
  /**
   * @brief  Parameterised Constructor with given inputs
   *
   * @param std::vector<std::vector<int>> world map
   * @param std::pair  robot start location
   * @param std::pair  robot goal location
   */
  /* ----------------------------------------------------------------*/
  optimalPlanner(std::vector<std::vector<int>>, std::pair<int, int>,
                 std::pair<int, int>);

  /* ----------------------------------------------------------------*/
  /**
   * @brief  Searches the path using astar algorithm
   *
   * @param std::vector<std::vector<int,int>> world map
   * @param std::pair robot start location
   * @param std::pair robot goal location
   *
   * @return stack containing path positions
   */
  /* ----------------------------------------------------------------*/
  std::stack<std::pair<int, int>> search(std::vector<std::vector<int>>,
                                         std::pair<int, int>,
                                         std::pair<int, int>);

  /* ----------------------------------------------------------------*/
  /**
   * @brief  getter function to get start location of robot
   *
   * @return robot starting location
   */
  /* ----------------------------------------------------------------*/
  std::pair<int, int> getStartingPoint();
  /* ----------------------------------------------------------------*/
  /**
   * @brief gettter function for Goal Point
   *
   * @return destination of the robot
   */
  /* ----------------------------------------------------------------*/
  std::pair<int, int> getGoalPoint();
  /* ----------------------------------------------------------------*/
  /**
   * @brief  getter fucntion for X length
   *
   * @return  x-length of the world map
   */
  /* ----------------------------------------------------------------*/
  int getXLength();
  /* ----------------------------------------------------------------*/
  /**
   * @brief  getter function for Y Length
   *
   * @return  depth of the world map
   */
  /* ----------------------------------------------------------------*/
  int getYLength();
  /* ----------------------------------------------------------------*/
  /**
   * @brief Node INformation stores all nodes at respective locations
   *
   * @param std::pair<int,int> node information needed
   *
   * @return node at respective location
   */
  /* ----------------------------------------------------------------*/
  node getNodeInformation(std::pair<int, int>);

  /* ----------------------------------------------------------------*/
  /**
   * @brief  To check if the given valid by checking boundaries
   *
   * @param std::pair<int,int> location whose validity needs to checked
   *
   * @return false if not valid , else true
   */
  /* ----------------------------------------------------------------*/
  bool isValid(std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  to check if the location has obstacle or not
   *
   * @param std::pair<int,int> location coordinates
   *
   * @return true if it has value of 1(obstacle ),else false
   */
  /* ----------------------------------------------------------------*/
  bool isBlocked(std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  To check if the location is goal or not
   *
   * @param std::pair<int, int> location (y,x) coordinates
   *
   * @return true if its same as goal
   */
  /* ----------------------------------------------------------------*/
  bool isItGoalYet(std::pair<int, int>);

  /* ----------------------------------------------------------------*/
  /**
   * @brief  To calculate H Cost, i.e, cost to reach the goal
   *
   * @param std::pair<int,int> location coordinates
   *
   * @return H cost of the location .
   */
  /* ----------------------------------------------------------------*/
  double calculateHCost(std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  To track the path by tracking individual parents
   */
  /* ----------------------------------------------------------------*/
  void trackPath();

  /* ----------------------------------------------------------------*/
  /**
   * @brief  Update node informations as we explore the map with all costs
   *
   * @param std::pair child node location
   * @param std::pair parent node location
   */
  /* ----------------------------------------------------------------*/
  void moveAndUpdateNodes(std::pair<int, int>, std::pair<int, int>);

  /* ----------------------------------------------------------------*/
  /**
   * @brief   To move in respective location
   *
   * @param std::pair location to be moved
   *
   * @return new location
   */
  /* ----------------------------------------------------------------*/
  std::pair<int, int> goTop(std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief   To move in respective location
   *
   * @param std::pair location to be moved
   *
   * @return new location
   */
  /* ----------------------------------------------------------------*/
  std::pair<int, int> goLeft(std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief   To move in respective location
   *
   * @param std::pair location to be moved
   *
   * @return new location
   */
  /* ----------------------------------------------------------------*/
  std::pair<int, int> goBottom(std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief   To move in respective location
   *
   * @param std::pair location to be moved
   *
   * @return new location
   */
  /* ----------------------------------------------------------------*/
  std::pair<int, int> goRight(std::pair<int, int>);

private:
  std::pair<int, int> start_position_;
  std::pair<int, int> goal_position_;
  bool found_goal;
  std::vector<std::vector<int>> world_map_;
  int x_length;
  int y_length;
  std::stack<std::pair<int, int>> path_;
  std::vector<std::vector<node>> node_information;
  std::vector<std::vector<bool>> closed_list; // to store visited nodes
  std::set<DoublePair> open_list; // to store all possible nodes which needs
                                  // to visited while exploring the map
};
