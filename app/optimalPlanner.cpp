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
 * @file optimalPlanner.cpp
 * @brief  Astar Algorithm
A* Search Algorithm: At each step it picks the node according to a
value-‘f’ which is a parameter equal to the sum of two other parameters – ‘g’
and ‘h’. At each step it picks the node/cell having the lowest ‘f’, and process
that node/cell.
g cost = the movement cost to move from the starting point to a given node on
the
grid, following the path generated to get there.
h cost = the estimated movement cost to move from that given node on the grid
to
the final destination
 *
 * Here we are using Manhattan Distance for calculating h cost for each nodes
 *
Working:
1. Add the starting node (or node) to the open list.

2. Repeat the following:

A) Look for the lowest F cost node on the open list. We refer to this as the
current node.

B). Switch it to the closed list.

C) For each of the 4 nodes adjacent to this current node …

If it is not walkable or if it is on the closed list, ignore it. Otherwise do
the following.
If it isn’t on the open list, add it to the open list. Make the current node
the parent of this node. Record the F, G, and H costs of the node.
If it is on the open list already, check to see if this path to that node is
better, using G cost as the measure. A lower G cost means that this is a better
path. If so, change the parent of the node to the current node, and
recalculate the G and F scores of the node. If you are keeping your open list
sorted by F score, you may need to resort the list to account for the change.
D) Stop when you:

Add the target node to the closed list, in which case the path has been found,
or
Fail to find the target node, and the open list is empty. In this case, there
is no path.
3. Save the path. Working backwards from the target node, go from each node
to its parent node until you reach the starting node. That is your path.

// ********************************************************************
*
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-04-04
 */
// ********************************************************** */
#include "../include/optimalPlanner.h"
#include "../include/node.h"

/* ----------------------------------------------------------------*/
/**
 * @brief  Default Planner for optimal planner class
 */
/* ----------------------------------------------------------------*/
optimalPlanner::optimalPlanner() {}
/* ----------------------------------------------------------------*/
/**
 * @brief  Parameterised Constructor for the class
 *
 * @param world_map
 * @param robot_pose
 * @param goal_pose
 */
/* ----------------------------------------------------------------*/
optimalPlanner::optimalPlanner(std::vector<std::vector<int>> world_map,
                               std::pair<int, int> robot_pose,
                               std::pair<int, int> goal_pose) {
  this->world_map_ = world_map;
  this->start_position_ = robot_pose;
  this->goal_position_ = goal_pose;
  this->y_length = int(world_map.size()) - 1;
  this->x_length = int(world_map[1].size()) - 1;
}

// getter functions
std::pair<int, int> optimalPlanner::getStartingPoint() {
  return this->start_position_;
}
std::pair<int, int> optimalPlanner::getGoalPoint() {
  return this->goal_position_;
}

int optimalPlanner::getXLength() { return this->x_length; }

int optimalPlanner::getYLength() { return this->y_length; }

/* ----------------------------------------------------------------*/
/**
 * @brief  To check Validity by checking if its located in map or not
 *
 * @param position
 *
 * @return false if not located inside map else true
 */
/* ----------------------------------------------------------------*/
bool optimalPlanner::isValid(std::pair<int, int> position) {
  int y = position.first;
  int x = position.second;

  bool valid =
      (x >= 0) && (y >= 0) && (x <= this->x_length) && (y <= this->y_length);
  return valid;
}

/* ----------------------------------------------------------------*/
/**
 * @brief  To check if there is obstacle or not
 *
 * @param position
 *
 * @return true if obstacle ,false if not obstacle
 */
/* ----------------------------------------------------------------*/
bool optimalPlanner::isBlocked(std::pair<int, int> position) {
  if (this->world_map_[position.first][position.second] == 1) {
    return true;
  } else {
    return false;
  }
}

/* ----------------------------------------------------------------*/
/**
 * @brief  Check for goal position
 *
 * @param position
 *
 * @return bool
 */
/* ----------------------------------------------------------------*/
bool optimalPlanner::isItGoalYet(std::pair<int, int> position) {
  if (position == this->goal_position_) {
    return true;
  } else
    return false;
}

/* ----------------------------------------------------------------*/
/**
 * @brief  Using Manhattan DIstance to calculate H cost for the location in map
 *
 * @param position
 *
 * @return double , Hcost value
 */
/* ----------------------------------------------------------------*/
double optimalPlanner::calculateHCost(std::pair<int, int> position) {
  double hcost = abs(position.first - this->goal_position_.first) +
                 abs(position.second - this->goal_position_.second);

  return hcost;
}

/* ----------------------------------------------------------------*/
/**
 * @brief  NodeInformation stores all nodes informatin , locaiton and all costs
 *
 * @param position
 *
 * @return find node at that location and return it as output
 */
/* ----------------------------------------------------------------*/
node optimalPlanner::getNodeInformation(std::pair<int, int> position) {
  return this->node_information[position.first][position.second];
}
/* ----------------------------------------------------------------*/
/**
 * @brief  Find Path based on parents of each node backwards
 */
/* ----------------------------------------------------------------*/
void optimalPlanner::trackPath() {
  std::pair<int, int> g = this->goal_position_;
  while (!(this->node_information[g.first][g.second].parent == g)) {
    this->path_.push(std::make_pair(g.first, g.second));
    g = this->node_information[g.first][g.second].parent;
  }
}

/* ----------------------------------------------------------------*/
/**
 * @brief  To move in respective direction to produce more children
 *
 * @param position
 *
 * @return new location of the child if robot moves
 */
/* ----------------------------------------------------------------*/
std::pair<int, int> optimalPlanner::goTop(std::pair<int, int> position) {
  return std::make_pair((position.first - 1), (position.second));
}
/* ----------------------------------------------------------------*/
/**
 * @brief  To move in respective direction to produce more children
 *
 * @param position
 *
 * @return new location of the child if robot moves
 */
/* ----------------------------------------------------------------*/
std::pair<int, int> optimalPlanner::goLeft(std::pair<int, int> position) {
  return std::make_pair((position.first), (position.second - 1));
}
/* ----------------------------------------------------------------*/
/**
 * @brief  To move in respective direction to produce more children
 *
 * @param position
 *
 * @return new location of the child if robot moves
 */
/* ----------------------------------------------------------------*/
std::pair<int, int> optimalPlanner::goBottom(std::pair<int, int> position) {
  return std::make_pair((position.first + 1), (position.second));
}
/* ----------------------------------------------------------------*/
/**
 * @brief  To move in respective direction to produce more children
 *
 * @param position
 *
 * @return new location of the child if robot moves
 */
/* ----------------------------------------------------------------*/
std::pair<int, int> optimalPlanner::goRight(std::pair<int, int> position) {
  return std::make_pair((position.first), (position.second + 1));
}

/* ----------------------------------------------------------------*/
/**
 * @brief
 *
 * @param child
 * @param parent
 */
/* ----------------------------------------------------------------*/
void optimalPlanner::moveAndUpdateNodes(std::pair<int, int> child,
                                        std::pair<int, int> parent) {
  // Ignore if child is not valid
  if (isValid(child)) {
    // check if we reached goal or not
    if (isItGoalYet(child)) {
      this->node_information[child.first][child.second].parent = parent;
      this->found_goal = true;
      return;
      // if we didnot reach goal , check if it was visited before as in
      // closed list
      // and if the child is blocked
    } else if (!(this->closed_list[child.first][child.second]) &&
               !isBlocked(child)) {
      double g_new =
          this->node_information[parent.first][parent.second].g_cost + 1;
      double h_new = this->calculateHCost(child);
      double f_new = g_new + h_new;
      //
      // Update the least cost to the respective location
      //
      // If it isn’t on the open list, add it to
      // the open list. Make the current node
      // the parent of this location, Record the
      // f, g, and h costs of the node cell
      //                OR
      // If it is on the open list already, check
      // to see if this path to that location  is better,
      // using 'f' cost as the measure. /
      if (this->node_information[child.first][child.second].f_cost == FLT_MAX ||
          this->node_information[child.first][child.second].f_cost < f_new) {
        this->open_list.insert(std::make_pair(f_new, child));
        this->node_information[child.first][child.second].f_cost = f_new;
        this->node_information[child.first][child.second].g_cost = g_new;
        this->node_information[child.first][child.second].h_cost = h_new;
        this->node_information[child.first][child.second].parent = parent;
      }
    }
  }
}
/* ----------------------------------------------------------------*/
/**
 * @brief  TO search using Astar Algorithm
 *
 * @param world_map
 * @param robot_pose
 * @param goal_pose
 *
 * @return Path
 */
/* ----------------------------------------------------------------*/
std::stack<std::pair<int, int>>
optimalPlanner::search(std::vector<std::vector<int>> world_map,
                       std::pair<int, int> robot_pose,
                       std::pair<int, int> goal_pose) {
  // Update all this members and do sanity checks

  this->world_map_ = world_map;
  this->start_position_ = robot_pose;
  this->goal_position_ = goal_pose;
  this->y_length = int(world_map.size()) - 1;
  this->x_length = int(world_map[1].size()) - 1;

  // Sanity checks for the start and goal position
  if (!isValid(this->start_position_)) {
    std::cout << "invalid robot pose" << std::endl;
  }
  if (!isValid(this->goal_position_)) {
    std::cout << "invalid goal pose" << std::endl;
  }

  if (isBlocked(this->start_position_) || isBlocked(this->goal_position_)) {
    std::cout << "wrong start/goal coordinates entered" << std::endl;
  }

  if (isItGoalYet(this->start_position_)) {
    std::cout << "Already at goal position" << std::endl;
  }
  // Inistialise closed list with false of size same as map
  // to store if location is visited or not
  //
  std::vector<std::vector<bool>> v(y_length + 1,
                                   std::vector<bool>(x_length + 1, false));

  this->closed_list = v;
  // INitialising all nodes in nodesInformation member to acess them
  // individually
  //
  node fake_node;
  fake_node.f_cost = FLT_MAX;
  fake_node.g_cost = FLT_MAX;
  fake_node.h_cost = FLT_MAX;
  fake_node.location_ = std::make_pair(-1, -1);
  fake_node.parent = std::make_pair(-1, -1);
  std::vector<std::vector<node>> fake_info(
      y_length + 1, std::vector<node>(x_length + 1, fake_node));
  this->node_information = fake_info;

  // Initialising the search by storing first/start node in nodeinfo and
  // in openlist
  int i = this->start_position_.first;
  int j = this->start_position_.second;
  this->node_information[i][j].f_cost = 0.0;
  this->node_information[i][j].g_cost = 0.0;
  this->node_information[i][j].h_cost = 0.0;
  this->node_information[i][j].parent = this->start_position_;

  this->open_list.insert(std::make_pair(0.0, this->start_position_));
  this->found_goal = false;

  // checking all the elements in openlist until goal is reached
  //
  while (!open_list.empty()) {
    DoublePair first_element = *this->open_list.begin();
    this->open_list.erase(this->open_list.begin());

    i = first_element.second.first;
    j = first_element.second.second;
    //
    // moving in all locations and updating nodes in nodesinformation
    // and checking if goal is reached
    //
    this->closed_list[i][j] = true;
    std::pair<int, int> temp = goTop(first_element.second);
    moveAndUpdateNodes(temp, first_element.second);
    if (found_goal == true) {
      trackPath();
      break;
    }

    temp = goLeft(first_element.second);
    moveAndUpdateNodes(temp, first_element.second);
    if (found_goal == true) {
      trackPath();
      break;
    }

    temp = goBottom(first_element.second);
    moveAndUpdateNodes(temp, first_element.second);
    if (found_goal == true) {
      trackPath();
      break;
    }

    temp = goRight(first_element.second);
    moveAndUpdateNodes(temp, first_element.second);
    if (found_goal == true) {
      trackPath();
      break;
    }
  }
  // check if goal is reached or not using found_goal flag
  if (found_goal == false) {
    std::cout << "no path found" << std::endl;
  }
  this->path_.push(start_position_);
  return this->path_;
}
