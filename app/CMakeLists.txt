add_executable(shell-app main.cpp 
                        RandomPlanner.cpp 
                        Node.cpp
                        optimalPlanner.cpp
                        node.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
