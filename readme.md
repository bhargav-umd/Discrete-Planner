# Motion Planning: Discrete Planner 
[![Build Status](https://travis-ci.org/dpiet/cpp-boilerplate.svg?branch=master)](https://travis-ci.org/dpiet/cpp-boilerplate)
[![Coverage Status](https://coveralls.io/repos/github/dpiet/cpp-boilerplate/badge.svg?branch=master)](https://coveralls.io/github/dpiet/cpp-boilerplate?branch=master)
---

## Overview
To extend a library of discrete motion planners for an holonomic robot. The robot
moves in a static and flat environment and must find a path to a goal. 
There are mainly two planners implemented in the program.
1. Random Planner 
2. Optimal Planner 

● Random planner
The random planner tries to find a path to the goal by randomly moving in the environment (only
orthogonal moves are legal). If the planner can not find an acceptable solution in less than
max_step_number, the search should fail. The random planner, while being erratic, has a short memory,
and it will never attempt to visit a cell that was visited in the last sqrt(max_step_number)​ steps except
if this is the only available option.
    -> Move randomly
    -> If all neighbors are visited already and only option to move is to move onto
        already visited block,then visit the block which has been visited already.
Random planners try to make a move randomly to figure out a path for the goal pose. 
The random planner has constraints of memory and finite time. 

● Optimal planner:A planner that goes to the goal with the shortest (non-colliding) path. Again,
only orthogonal moves are legal.Implemented Astar algorithm, as it gives the optimal
path. 

Astar Working(also explained well in code):
    
1. Add the starting node to the open list.

2. Repeat the following:

A) Look for the lowest F cost square on the open list. We refer to this as the current square.

B). Switch it to the closed list.

C) For each of the 4 nodes adjacent to this current node …

If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following.
If it isn’t on the open list, add it to the open list. Make the current square the parent of this square. Record the F, G, and H costs of the square.
If it is on the open list already, check to see if this path to that square is better, using G cost as the measure. A lower G cost means that this is a better path. If so, change the parent of the square to the current square, and recalculate the G and F scores of the square. If you are keeping your open list sorted by F score, you may need to resort the list to account for the change.  

D) Stop when you:  
Add the goal node to the closed list, in which case the path has been found, or
Fail to find the goal node , and the open list is empty. In this case, there is no path.  

3.Save the path. Working backwards from the goal node , go from each node to its parent node until you reach the starting node. That is your path.

## Performance Comparision

    -> Random PLanner is slower to run compared to optimal Planner. As there is time and 
        memory constraint on it.
Time Complexity of Random Planner depends on random number generation, world size and number of steps to be taken.
It takes time based on world size (possibly bigger than n i.e, number of steps aken), we check sqrt(n) memor.
We can possibly conclude the order to be O(n^(3/2)) 

    -> Optimal Planner (Astar here in case) is lot faster than the random Planner. 
Time Complexity of optimal planner depends on heuristics which is chosen as Manhattan distance.
Manhattan distance is sum of absolute difference of x coordinates and y coordinates of 
current point and goal point. Here its O(Manhattan distance from start to goal).


## Standard install via command-line
```
cd <path to repository>
mkdir build
cd build
cmake ..
make
```
### Run tests:
```
./test/cpp-test
```
### Run program:
```
 ./app/shell-app
```

## Building for code coverage (for assignments beginning in Week 4)
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.


## Doxygen Documentation
To generate Doxygen Documentation in HTML and LaTEX, follow the next steps:
```
cd <path to repository>
mkdir <documentation_folder_name>
cd <documentation_folder_name>
doxygen -g <config_file_name>
```
Inside the configuration file, update:
```
PROJECT_NAME = 'Discrete Planner'
INPUT = ../app ../include ../test ../
```
Run and generate the documents by running the next command:
```
doxygen <config_file_name>
`````````
Use file Explorer and Open html/index.html file to see the doxygen documentation in web browser.
