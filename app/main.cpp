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
 * @file main.cpp
 * @brief  to call random and optimal planners
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-04-04
 */
#include "../include/Node.h"
#include "../include/RandomPlanner.h"
#include "../include/node.h"
#include "../include/optimalPlanner.h"

int main() {
  std::vector<std::vector<int>> world_state{
      {0, 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 1, 0},
      {0, 0, 0, 0, 1, 0}, {0, 0, 1, 1, 1, 0}, {0, 0, 0, 0, 0, 0}};

  std::pair<int, int> robot_pose(2, 0);
  std::pair<int, int> goal_pose(5, 5);
  // calling Random PLanner object and searching for path
  RandomPlanner rp;
  std::cout << " This is the path found by random planner: " << std::endl;

  std::vector<std::pair<int, int>> path =
      rp.search(world_state, robot_pose, goal_pose);

  // calling optimal planner for the map
  optimalPlanner optimalPlan;
  std::stack<std::pair<int, int>> optimal_path_ =
      optimalPlan.search(world_state, robot_pose, goal_pose);
  std::cout << "   " << std::endl;
  std::cout << "This is the path found by optimal planner " << std::endl;

  // tracing and printing the path as found by optimal planner
  while (!optimal_path_.empty()) {
    std::pair<int, int> p = optimal_path_.top();
    optimal_path_.pop();
    std::cout << "(" << p.first << "," << p.second << ")"
              << ",";
  }

  return 0;
}
