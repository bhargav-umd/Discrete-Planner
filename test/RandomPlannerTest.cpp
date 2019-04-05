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
 * @file RandomPlannerTest.cpp
 * @brief  To test all functions in Random PLanner
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-04-05
 */
#include "../include/RandomPlanner.h"
#include "../include/Node.h"
#include <gtest/gtest.h>

std::vector<std::vector<int>> world{{0, 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
                                    {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 1, 0},
                                    {0, 0, 1, 1, 1, 0}, {0, 0, 0, 0, 0, 0}};

std::pair<int, int> robot_pose_rp(2, 0);
std::pair<int, int> goal_pose_rp(5, 5);

RandomPlanner rp_test(world, robot_pose_rp, goal_pose_rp);

TEST(randomTest, goalPointSetTEst) {
  EXPECT_EQ(rp_test.getGoal(), goal_pose_rp);
}

TEST(randomTest, startPointTest) {
  EXPECT_EQ(rp_test.getStart(), robot_pose_rp);
}

TEST(randomTest, IsObstacleTest) {
  bool inside = rp_test.isObstacle(std::make_pair(0, 0));
  ASSERT_FALSE(inside);
}
TEST(randomTest, IsObstacleinsideTest) {
  bool obs_inside = rp_test.isObstacle(std::make_pair(0, 2));
  ASSERT_TRUE(obs_inside);
}
TEST(randomTest, IsObstacleOutsideTest) {
  bool outside = rp_test.isObstacle(std::make_pair(99, 222));
  ASSERT_TRUE(outside);
}

TEST(randomTest, XlengthTEST_Rp) {
  EXPECT_EQ(rp_test.getXLength(), int(world[0].size() - 1));
}
TEST(randomTest, YlengthTEST_Rp) {
  EXPECT_EQ(rp_test.getXLength(), int(world.size() - 1));
}

TEST(randomTest, MoveTop) {
  Node test;
  test.value_ = 0;
  test.position_.first = 1;
  test.position_.second = 2;
  EXPECT_EQ(rp_test.moveUp(test).position_.first, test.position_.first - 1);
}
TEST(randomTest, MoveLeft) {
  Node test;
  test.value_ = 0;
  test.position_.first = 1;
  test.position_.second = 2;
  EXPECT_EQ(rp_test.moveLeft(test).position_.second, test.position_.second - 1);
}
TEST(randomTest, MoveDown) {
  Node test;
  test.value_ = 0;
  test.position_.first = 1;
  test.position_.second = 2;
  EXPECT_EQ(rp_test.moveDown(test).position_.first, test.position_.first + 1);
}
TEST(randomTest, MoveRight) {
  Node test;
  test.value_ = 0;
  test.position_.first = 1;
  test.position_.second = 2;
  EXPECT_EQ(rp_test.moveRight(test).position_.second,
            test.position_.second + 1);
}
TEST(randomTest, findNeighborsTest) {
  Node test;
  test.value_ = 0;
  test.position_.first = 0;
  test.position_.second = 0;
  std::vector<std::pair<int, int>> neighbor_t;
  neighbor_t.push_back(std::make_pair(1, 0));
  neighbor_t.push_back(std::make_pair(0, 1));

  EXPECT_EQ(rp_test.findNeighbors(test.position_), neighbor_t);
}
