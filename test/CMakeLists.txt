set(GTEST_SHUFFLE 1)

add_executable( cpp-test main.cpp
                 optimalPlannerTest.cpp
                 RandomPlannerTest.cpp
                 ../app/optimalPlanner.cpp 
                 ../app/node.cpp
                 ../app/RandomPlanner.cpp
                 ../app/Node.cpp) 
#)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
