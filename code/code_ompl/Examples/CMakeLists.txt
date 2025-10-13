cmake_minimum_required(VERSION 3.10.1)
project(Project0)

set(CMAKE_CXX_STANDARD 14)

find_package(ompl REQUIRED)


add_executable(BouncingBallPlanning BouncingBallPlanning.cpp)
#If you install from master you will have to link the new way by uncommentting the following line and commenting the rest.
target_link_libraries(BouncingBallPlanning ompl::ompl)
#target_link_libraries(BouncingBallPlanning PRIVATE ${OMPL_LIBRARIES})
#target_include_directories(BouncingBallPlanning PRIVATE ${OMPL_INCLUDE_DIRS})
#target_link_directories(BouncingBallPlanning PRIVATE ${OMPL_INCLUDE_DIRS})