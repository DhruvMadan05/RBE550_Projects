#include "CollisionChecking.h"
#include "RTP.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

#include <iostream> // For console output
#include <fstream> // For file output
#include <vector> // For std::vector

namespace ob = ompl::base; //
namespace og = ompl::geometric;

static constexpr double HBOUND = 5.0; // boundary in the x direction
static constexpr double LBOUND = 5.0; // boundary in the y direction

static constexpr double BOX_SIZE = 0.3; // side length of the box robot

// --- State validity checkers ---
/*
* Check validity of a point robot state
* A state is valid if it is within bounds and does not collide with any obstacles
*
* @param state: The state to check (expects RealVectorStateSpace::StateType)
* @param obstacles: List of rectangular obstacles in the environment
* @return: true if the state is valid (no collision), false otherwise
*/
bool isStateValidPoint(const ob::State *state,
                       const std::vector<Rectangle> &obstacles)
{
    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
    double x = pos->values[0];
    double y = pos->values[1];

    // Check bounds
    if (x < 0 || x > HBOUND || y < 0 || y > LBOUND)
        return false;

    return isValidPoint(x, y, obstacles);
}


/*
* Check validity of a box robot state
* A state is valid if the box does not collide with any obstacles
* 
* @param state: The state to check (expects SE2StateSpace::StateType)
* @param obstacles: List of rectangular obstacles in the environment
* @return: true if the state is valid (no collision), false otherwise
*/
bool isStateValidBox(const ob::State *state, const std::vector<Rectangle> &obstacles)
{
    const auto *se2 = state->as<ob::SE2StateSpace::StateType>();
    return isValidSquare(se2->getX(), se2->getY(), se2->getYaw(), BOX_SIZE, obstacles, HBOUND, LBOUND);
}


// --- Planning functions ---
/*
* Plan a path for a point robot in 2D using RTP
* 
* @param obstacles: List of rectangular obstacles in the environment
*/
void planPoint(const std::vector<Rectangle> &obstacles)
{
    // Create the state space (R^2 for point robot)
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, 0.0); bounds.setHigh(0, HBOUND);
    bounds.setLow(1, 0.0); bounds.setHigh(1, LBOUND);
    space->setBounds(bounds);

    // Create SpaceInformation with obstacle checker
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(
        [&obstacles](const ob::State *s) {
            return isStateValidPoint(s, obstacles);
        });

    // Define start and goal states
    ob::ScopedState<> start(space);
    start[0] = 0.4; start[1] = 0.4;
    ob::ScopedState<> goal(space);
    goal[0] = 3.6; goal[1] = 3.6;

    // Create ProblemDefinition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);

    // Create the planner
    auto planner(std::make_shared<og::RTP>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve the problem within 10 seconds
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

    // Output the result if solved
    if (solved)
    {
        auto path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        
        // // Smooth the path to improve quality
        // og::PathSimplifier pathSimplifier(si);
        // pathSimplifier.simplify(*path, 5.0);  // 5 seconds for smoothing
        
        std::cout << "Found solution for point robot:\n";
        path->print(std::cout);

        // Dump to file
        std::ofstream out("point_solution.txt");
        path->printAsMatrix(out);
        out.close();  // Ensure file is properly closed
        std::cout << "Saved solution path to point_solution.txt\n";
    }
    else
        // If no solution found
        std::cout << "No solution found for point robot.\n";
}

/*
* Plan a path for a box robot in 2D using RTP
*
* @param obstacles: List of rectangular obstacles in the environment
*/
void planBox(const std::vector<Rectangle> &obstacles)
{
    // Create the state space (SE(2) for box robot)
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, 0.0); bounds.setHigh(0, HBOUND);
    bounds.setLow(1, 0.0); bounds.setHigh(1, LBOUND);
    space->setBounds(bounds);

    // Create SpaceInformation with obstacle checker
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(
        [&obstacles](const ob::State *s) {
            return isStateValidBox(s, obstacles);
        });

    // Define start and goal states
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.6); start->setY(0.6); start->setYaw(0.0);
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(3.4); goal->setY(3.4); goal->setYaw(0.0);

    // Create ProblemDefinition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);

    // Create the planner
    auto planner(std::make_shared<og::RTP>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve the problem within 10 seconds
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

    // Output the result if solved
    if (solved)
    {
        auto path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        
        // // Smooth the path to improve quality
        // og::PathSimplifier pathSimplifier(si);
        // pathSimplifier.simplify(*path, 5.0);  // 5 seconds for smoothing
        
        std::cout << "Found solution for box robot:\n";
        path->print(std::cout);

        // Dump to file
        std::ofstream out("box_solution.txt");
        path->printAsMatrix(out);
        std::cout << "Saved solution path to box_solution.txt\n";
    }
    else
        std::cout << "No solution found for box robot.\n";
}


/*
* Create Environment 1 by loading obstacles from a file
* 
* @param obstacles: Vector to populate with rectangular obstacles
*/
void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    obstacles.clear();
    std::ifstream infile("obstacles1.txt");
    double x, y, w, h;
    while (infile >> x >> y >> w >> h)
        obstacles.push_back(Rectangle{x, y, w, h});
    std::cout << "Loaded environment 1 with " << obstacles.size() << " obstacles\n";
}

/*
* Create Environment 2 by loading obstacles from a file
* 
* @param obstacles: Vector to populate with rectangular obstacles
*/
void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    obstacles.clear();
    std::ifstream infile("obstacles2.txt");
    double x, y, w, h;
    while (infile >> x >> y >> w >> h)
        obstacles.push_back(Rectangle{x, y, w, h});
    std::cout << "Loaded environment 2 with " << obstacles.size() << " obstacles\n";

}

// --- Main function ---
int main(int /* argc */, char ** /* argv */)
{
    // User selects robot type and environment
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        // Prompt user for robot type
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        // Prompt user for environment choice
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Environment 1" << std::endl;
        std::cout << " (2) Environment 2" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}