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
 * @file Node.h
 * @brief  A structure to denote each block in map for RandomPlanner
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-04-03
 */
#pragma once
#include <iostream>
#include <utility>

/* ----------------------------------------------------------------*/
/**
 * @brief  Basic Node Structure for each block
 */
/* ----------------------------------------------------------------*/
struct Node {
  Node();                         // Default constructor
  Node(int, std::pair<int, int>); // second constructor to initialise with
                                  // data members
  std::pair<int, int> position_;  // location of the node (y,x) coordinate
  int value_; // this tells if its obstacle(1) or free space (0)
};
