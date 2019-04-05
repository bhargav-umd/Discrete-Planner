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
 * @file optimalPlannerTest.cpp
 * @brief  Test Optimal Test functions
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-04-05
 */
#include "../include/optimalPlanner.h"
#include "../include/node.h"
#include <gtest/gtest.h>

std::vector<std::vector<int>> world_state{
    {0, 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 1, 0}, {0, 0, 1, 1, 1, 0}, {0, 0, 0, 0, 0, 0}};

std::pair<int, int> robot_pose(2, 0);
std::pair<int, int> goal_pose(5, 5);

optimalPlanner testObj(world_state, robot_pose, goal_pose);

TEST(test, invalidityTest) {
  std::pair<int, int> point = std::make_pair(99, 4);
  bool valid = testObj.isValid(point);
  ASSERT_FALSE(valid);
}

TEST(test, OnBoundaryValidityTest) {
  std::pair<int, int> point = std::make_pair(0, 0);
  bool valid = testObj.isValid(point);
  ASSERT_TRUE(valid);
}

TEST(test, validityTest) {
  std::pair<int, int> point = std::make_pair(3, 4);
  bool valid = testObj.isValid(point);
  ASSERT_TRUE(valid);
}

TEST(test, goalPointRegistering) {
  //    std::pair<int,int> goal= testObj.getGoalPoint();
  EXPECT_EQ(testObj.getGoalPoint(), goal_pose);
}

TEST(test, startingPointRegistering) {
  EXPECT_EQ(testObj.getStartingPoint(), robot_pose);
}
TEST(test, ylengthTest) {
  EXPECT_EQ(testObj.getYLength(), int(world_state.size()) - 1);
}
TEST(test, xlengthTest) {
  EXPECT_EQ(testObj.getYLength(), int(world_state[0].size()) - 1);
}

TEST(test, isBlockedTest) {
  std::pair<int, int> point = std::make_pair(0, 2);
  ASSERT_TRUE(testObj.isBlocked(point));
}

TEST(test, isItGoalYetTest) {
  std::pair<int, int> point = std::make_pair(0, 2);
  ASSERT_FALSE(testObj.isItGoalYet(point));
}

TEST(test, isGoalYetTest) {
  std::pair<int, int> point = goal_pose;
  ASSERT_TRUE(testObj.isItGoalYet(point));
}

TEST(test, calculateHCostTest) {
  std::pair<int, int> point = std::make_pair(0, 0);
  EXPECT_EQ(testObj.calculateHCost(point), 10.0);
}

TEST(test, goTopTest) {
  EXPECT_EQ(testObj.goTop(std::make_pair(1, 2)), std::make_pair(0, 2));
}
TEST(test, goDownTest) {
  EXPECT_EQ(testObj.goBottom(std::make_pair(1, 2)), std::make_pair(2, 2));
}
TEST(test, goLeftTest) {
  EXPECT_EQ(testObj.goLeft(std::make_pair(1, 2)), std::make_pair(1, 1));
}
TEST(test, goRightTest) {
  EXPECT_EQ(testObj.goRight(std::make_pair(1, 2)), std::make_pair(1, 3));
}

TEST(test, moveAndUpdateNodesTest) {
  std::pair<int, int> child = std::make_pair(1, 0);
  std::pair<int, int> parent = std::make_pair(2, 0);

  testObj.search(world_state, robot_pose, goal_pose);
  testObj.moveAndUpdateNodes(child, parent);
  double fc = testObj.getNodeInformation(child).f_cost;
  EXPECT_EQ(fc, 10.0);
}
TEST(test, moveAndUpdateNodesHCTest) {
  std::pair<int, int> child = std::make_pair(1, 0);
  std::pair<int, int> parent = std::make_pair(2, 0);

  testObj.search(world_state, robot_pose, goal_pose);
  testObj.moveAndUpdateNodes(child, parent);
  double hc = testObj.getNodeInformation(child).h_cost;
  EXPECT_EQ(hc, 9.0);
}
TEST(test, moveAndUpdateNodesGCTest) {
  std::pair<int, int> child = std::make_pair(1, 0);
  std::pair<int, int> parent = std::make_pair(2, 0);

  testObj.search(world_state, robot_pose, goal_pose);
  testObj.moveAndUpdateNodes(child, parent);
  double gc = testObj.getNodeInformation(child).g_cost;
  EXPECT_EQ(gc, 1.0);
}
TEST(test, moveAndUpdateNodesParentTest) {
  std::pair<int, int> child = std::make_pair(1, 0);
  std::pair<int, int> parent = std::make_pair(2, 0);

  testObj.search(world_state, robot_pose, goal_pose);
  testObj.moveAndUpdateNodes(child, parent);
  std::pair<int, int> par = testObj.getNodeInformation(child).parent;
  EXPECT_EQ(par, std::make_pair(2, 0));
}

TEST(test, pathTest) {
  std::stack<std::pair<int, int>> pathT =
      testObj.search(world_state, robot_pose, goal_pose);
  pathT.pop();
  pathT.pop();
  std::pair<int, int> path_third_element = pathT.top();
  EXPECT_EQ(path_third_element, std::make_pair(3, 1));
}
