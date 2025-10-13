cmake_minimum_required(VERSION 3.10.1)
project(Project0)

set(CMAKE_CXX_STANDARD 11)

# Set paths to help CMake find dependencies
set(CMAKE_PREFIX_PATH "/usr/local;/opt/homebrew")
set(BOOST_ROOT "/opt/homebrew")
set(Boost_NO_SYSTEM_PATHS ON)

# Find OMPL (which will handle Boost dependencies)
find_package(ompl REQUIRED)


add_executable(Point2DPlanning Point2DPlanning.cpp)

# Link all required libraries
target_link_libraries(Point2DPlanning PRIVATE 
    ${OMPL_LIBRARIES}
    /opt/homebrew/lib/libboost_serialization.dylib
    /opt/homebrew/lib/libboost_filesystem.dylib
)
target_include_directories(Point2DPlanning PRIVATE ${OMPL_INCLUDE_DIRS})
