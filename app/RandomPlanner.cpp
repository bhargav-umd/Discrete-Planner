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
 * @file RandomPlanner.cpp
 * @brief  Random Planner class definitions
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-04-03
 */
#include "../include/RandomPlanner.h"
#include "../include/Node.h"

/* ----------------------------------------------------------------*/
/**
 * @brief  Default Constructor
 */
/* ----------------------------------------------------------------*/
RandomPlanner::RandomPlanner() {}
/* ----------------------------------------------------------------*/
/**
 * @brief  Parameterised constructor which initializes planner with input
 *
 * @param map
 * @param robot_pose starting location of robot
 * @param goal goal location of robot
 */
/* ----------------------------------------------------------------*/
RandomPlanner::RandomPlanner(std::vector<std::vector<int>> map,
                             std::pair<int, int> robot_pose,
                             std::pair<int, int> goal_pose) {
  this->world_map = map;
  this->robot_pose = robot_pose; // starting location of the robot
  this->goal_pose = goal_pose;   // destination of the robot
  this->current_node.position_ = robot_pose;
  // variable containing moving robot position
  this->x_length = int(world_map.size()) - 1; // size of map in x direction
  this->y_length =
      int(world_map[1].size()) - 1; /// size of the map in y direction

  this->setStartNode();
  this->setGoalNode();
}
/* ----------------------------------------------------------------*/
/**
 * @brief  to produce random direction in 1-4 rangle denoting
 * Directions
 * 1 -> Up
 * 2 -> Left
 * 3 -> DOwn
 * 4 ->  Right
 *
 * @return  direction
 */
/* ----------------------------------------------------------------*/
int RandomPlanner::randomDirection() {
  int direction;  // to generate random direction
  srand(time(0)); // random number generation based on time
  return direction = rand() % 4 + 1;
}

// defining getter function for goal and starting location of therobot;
std::pair<int, int> RandomPlanner::getGoal() { return this->goal_pose; }

std::pair<int, int> RandomPlanner::getStart() { return this->robot_pose; }
/* ----------------------------------------------------------------*/
/**
 * @brief  sets goal Node from inputs of value(0=free) and location
 *
 */
/* ----------------------------------------------------------------*/
void RandomPlanner::setGoalNode() {
  this->goal_node.value_ = this->world_map[goal_pose.first][goal_pose.second];
  this->goal_node.position_ = this->goal_pose;
}
/* ----------------------------------------------------------------*/
/**
 * @brief  sets start node from starting locaiton and value (0=free)
 */
/* ----------------------------------------------------------------*/
void RandomPlanner::setStartNode() {
  this->start_node.value_ =
      this->world_map[robot_pose.first][robot_pose.second];
  this->start_node.position_ = this->robot_pose;
  //  this->start_node.parent_ = &start_node;
}

/* ----------------------------------------------------------------*/
/**
 * @brief  Checks if the position is valid positon in map and is obstacle or not
 *         1 = obstacle, 0= free
 *
 * @param to_be_checked location to be checked
 *
 * @return bool if its obstacle or free space
 */
/* ----------------------------------------------------------------*/
bool RandomPlanner::isObstacle(std::pair<int, int> to_be_checked) {
  if (to_be_checked.first > y_length || to_be_checked.first < 0 ||
      to_be_checked.second > x_length || to_be_checked.second < 0) {
    return true;
  } else if (this->world_map[to_be_checked.first][to_be_checked.second] == 1) {
    return true;
  } else
    return false;
}
int RandomPlanner::getXLength() { return this->x_length; }
int RandomPlanner::getYLength() { return this->y_length; }
/* ----------------------------------------------------------------*/
/**
 * @brief  Update Last steps with new Node, remove first in and add new element
 *          to planners short memory of length sqrt(max_step_number)
 *
 * @param a_node new node to be included in memory
 */
/* ----------------------------------------------------------------*/
void RandomPlanner::updateLastSteps(Node a_node) {
  if (int(this->last_steps.size()) <= int(sqrt(this->max_step_number))) {
    this->last_steps.push_back(a_node.position_);
  } else {
    this->last_steps.pop_front();
    this->last_steps.push_back(a_node.position_);
  }
}
/* ----------------------------------------------------------------*/
/**
 * @brief  Check if input node is visited already in recent steps
 *
 * @param toSearch Node to searched
 *
 * @return false if not visited else true i.,e visited recently
 */
/* ----------------------------------------------------------------*/
bool RandomPlanner::checkLastNSteps(Node toSearch) {
  for (auto i = this->last_steps.begin(); i != this->last_steps.end(); ++i) {
    std::pair<int, int> p = *i;
    if (toSearch.position_.first == p.first &&
        toSearch.position_.second == p.second) {
      return true;
    }
  }
  return false;
}
/* ----------------------------------------------------------------*/
/**
 * @brief  To check if the input position is visited recently or not
 *
 * @param position to check if position is visited
 *
 * @return false if not visited, true if visited recently
 */
/* ----------------------------------------------------------------*/
bool RandomPlanner::checkLastNPositions(std::pair<int, int> position) {
  for (auto i = this->last_steps.begin(); i != this->last_steps.end(); ++i) {
    std::pair<int, int> p = *i;
    if (position.first == p.first && position.second == p.second) {
      return true;
    }
  }
  return false;
}
/*
 * Directions
 * 1- Up
 * 2 -Left
 * 3 -DOwn
 * 4 - Right
 */
/* ----------------------------------------------------------------*/
/**
 * @brief  Move in respective direction
 *
 * @param someNode
 *
 * @return Updated NOde
 */
/* ----------------------------------------------------------------*/
Node RandomPlanner::moveUp(Node someNode) {
  Node tempNode;
  // tempNode.parent_ = &someNode;
  tempNode.position_.first = someNode.position_.first - 1;
  tempNode.position_.second = someNode.position_.second;
  tempNode.value_ =
      this->world_map[tempNode.position_.first][tempNode.position_.second];

  return tempNode;
}
/* ----------------------------------------------------------------*/
/**
 * @brief  Move in respective direction
 *
 * @param someNode node to be moved
 *
 * @return Updated NOde
 */
/* ----------------------------------------------------------------*/
Node RandomPlanner::moveLeft(Node someNode) {
  Node tempNode;
  // tempNode.parent_ = &someNode;
  tempNode.position_.first = someNode.position_.first;
  tempNode.position_.second = someNode.position_.second - 1;
  tempNode.value_ =
      this->world_map[tempNode.position_.first][tempNode.position_.second];
  return tempNode;
}

/* ----------------------------------------------------------------*/
/**
 * @brief  Move in respective direction
 *
 * @param someNode Node to be moved
 *
 * @return Updated NOde
 */
/* ----------------------------------------------------------------*/
Node RandomPlanner::moveDown(Node someNode) {
  Node tempNode;
  // tempNode.parent_ = &someNode;
  tempNode.position_.first = someNode.position_.first + 1;
  tempNode.position_.second = someNode.position_.second;
  tempNode.value_ =
      this->world_map[tempNode.position_.first][tempNode.position_.second];
  return tempNode;
}

/* ----------------------------------------------------------------*/
/**
 * @brief  Move in respective direction
 *
 * @param someNode
 *
 * @return Updated NOde
 */
/* ----------------------------------------------------------------*/
Node RandomPlanner::moveRight(Node someNode) {
  Node tempNode;
  // tempNode.parent_ = &someNode;
  tempNode.position_.first = someNode.position_.first;
  tempNode.position_.second = someNode.position_.second + 1;
  tempNode.value_ =
      this->world_map[tempNode.position_.first][tempNode.position_.second];
  return tempNode;
}
/*
 * Directions
 * 1- Up
 * 2 -Left
 * 3 -DOwn
 * 4 - Right
 */

/* ----------------------------------------------------------------*/
/**
 * @brief  locate all neigtbors by moving in all directions and checking if they
 *          are not obstacles
 *
 * @param position
 *
 * @return vector of all possible neighbors
 */
/* ----------------------------------------------------------------*/
std::vector<std::pair<int, int>>
RandomPlanner::findNeighbors(std::pair<int, int> position) {
  // possible neighbors if moved in orthogonal direction
  std::pair<int, int> up(position.first - 1, position.second);
  std::pair<int, int> left(position.first, position.second - 1);
  std::pair<int, int> down(position.first + 1, position.second);
  std::pair<int, int> right(position.first, position.second + 1);
  std::vector<std::pair<int, int>> neighbors;
  if (!this->isObstacle(up)) {
    neighbors.push_back(up);
  }
  if (!this->isObstacle(left)) {
    neighbors.push_back(left);
  }
  if (!this->isObstacle(down)) {
    neighbors.push_back(down);
  }
  if (!this->isObstacle(right)) {
    neighbors.push_back(right);
  }
  return neighbors;
}

/* ----------------------------------------------------------------*/
/**
 * @brief  for checking if any of the neighbors are visited recently ornot
 *
 * @param position
 *
 * @return true if any one of the neighbot was in memory ,else false
 */
/* ----------------------------------------------------------------*/
bool RandomPlanner::allNeighborsInMemory(std::pair<int, int> position) {
  std::vector<std::pair<int, int>> neighbors = findNeighbors(position);
  bool check = true;
  for (int i = 0; i < int(neighbors.size()); i++) {
    check = check & checkLastNPositions(neighbors[i]);
  }
  if (check)
    return true;
  else
    return false;
}
/* ----------------------------------------------------------------*/
/**
 * @brief  Moves in random directions, can update the locaiton only if the
 *         robot can move in that direction and the new node is stored in
 *         queue unless its the only option forrobot to move
 */
/* ----------------------------------------------------------------*/
std::vector<std::pair<int, int>>
RandomPlanner::search(std::vector<std::vector<int>> map,
                      std::pair<int, int> robot_pose,
                      std::pair<int, int> goal_pose) {
  // updating all this variables as in constructor to maintain
  // common interface for all planners

  this->world_map = map;
  this->robot_pose = robot_pose;
  this->goal_pose = goal_pose;
  this->current_node.position_ = robot_pose;
  this->x_length = int(world_map.size()) - 1; // size of map in x direction
  this->y_length = int(world_map[1].size()) - 1;

  this->setStartNode();
  this->setGoalNode();
  this->current_node = this->start_node;
  this->path_.push_back(start_node.position_);
  int steps_taken = 0;

  //  Sanity checks for goal and starting locaition

  if (this->start_node.position_ == this->goal_node.position_) {
    std::cout << "start node and goal node are same" << std::endl;
  }

  if (isObstacle(this->start_node.position_)) {
    std::cout << "start node is Obstacle" << std::endl;
  }
  if (isObstacle(this->goal_node.position_)) {
    std::cout << "goal node is Obstacle" << std::endl;
  }
  //
  // Starting loop for robot to move in random direction
  //
  //
  while (steps_taken <= this->max_step_number) {
    int dir = randomDirection();
    // std::cout << "random direction: " << dir << std::endl;

    if (dir == 1) {
      std::pair<int, int> cu_pose = this->current_node.position_;
      cu_pose.first = cu_pose.first - 1;
      if (!isObstacle(cu_pose)) { // validity of node
        Node tempNode = this->moveUp(current_node);
        // upFlag = true;
        if (!checkLastNSteps(tempNode)) { // if its not in memory
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node); // updated node to memory
          this->path_.push_back(
              this->current_node.position_); // as we moved to new
                                             // location , updating
                                             // to keep track of
                                             // path
          steps_taken++;                     // iupdating number of steps taken
        }
      }
    }
    if (dir == 2) {
      std::pair<int, int> cu_pose = this->current_node.position_;
      cu_pose.second = cu_pose.second - 1;
      if (!isObstacle(cu_pose)) {
        Node tempNode = this->moveLeft(current_node);
        if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
        }
      }
    }
    if (dir == 3) {
      std::pair<int, int> cu_pose = this->current_node.position_;
      cu_pose.first = cu_pose.first + 1;
      if (!isObstacle(cu_pose)) {
        Node tempNode = this->moveDown(current_node);
        if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
        }
      }
    }
    if (dir == 4) {
      std::pair<int, int> cu_pose = this->current_node.position_;
      cu_pose.second = cu_pose.second + 1;
      if (!isObstacle(cu_pose)) {
        Node tempNode = this->moveRight(current_node);
        if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
        }
      }
    }
    // if robot cannot move in any direction,all flags are 0 as all nodes
    // are visited before or blocked , then it can move in random direction
    // as produced by random direction generator function above
    //
    if (allNeighborsInMemory(current_node.position_)) {
      if (dir == 1) {
        std::pair<int, int> cu_pose = this->current_node.position_;
        cu_pose.first = cu_pose.first - 1;
        if (!isObstacle(cu_pose)) {
          Node tempNode = this->moveUp(current_node);
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
        }
      }
      if (dir == 2) {
        std::pair<int, int> cu_pose = this->current_node.position_;
        cu_pose.second = cu_pose.second - 1;
        if (!isObstacle(cu_pose)) {
          Node tempNode = this->moveLeft(current_node);
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
        }
      }
      if (dir == 3) {
        std::pair<int, int> cu_pose = this->current_node.position_;
        cu_pose.first = cu_pose.first + 1;
        if (!isObstacle(cu_pose)) {
          Node tempNode = this->moveDown(current_node);
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
        }
      }
      if (dir == 4) {
        std::pair<int, int> cu_pose = this->current_node.position_;
        cu_pose.second = cu_pose.second + 1;
        if (!isObstacle(cu_pose)) {
          Node tempNode = this->moveRight(current_node);
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
        }
      }
    }
    // break the loop before hand if we reach goal
    if (this->current_node.position_ == this->goal_node.position_) {
      break;
    }
  }
  // if we cannot reach goal in required number of steps then print the
  // following
  if (this->current_node.position_ != this->goal_node.position_) {
    std::cout << "goal not found" << std::endl;
  }

  // printing path if found
  if (this->current_node.position_ == this->goal_node.position_) {
    for (int i = 0; i < int(path_.size()); i++) {
      if (i != int(path_.size()) - 1) {
        std::cout << "(" << path_[i].first << "," << path_[i].second << ") "
                  << ",";
      } else {
        std::cout << "(" << path_[i].first << "," << path_[i].second << ")"
                  << std::endl;
      }
    }
  }
  return path_;
}
