#include "KinematicChain.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>


void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2 ; 
    goal[1] = 2 ; 
    goal[2] = 0; 
    goal[4] = -1.57079;

    //Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    //Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);

}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -4;
    start[1] = -4;
    start[2] = 0;
    goal[0] = 3; 
    goal[1] = 3; 
    goal[2] = 0; 
    goal[4] = -1.57079;

    //Obstacle 1
    env.emplace_back(-1, -1, 1, -1);
    env.emplace_back(1, -1, 1, 1);
    env.emplace_back(1, 1, -1, 1);
    env.emplace_back(-1, 1, -1, -1);
}


/**
 * Plans for scenario 1 and stores the path in path1.txt
 * Choose the most efficient planner from rrt, prm, or rrtconnect, to solve this problem. In
 * your report, explain which planner you selected and why you believe it is the fastest for this
 * scenario.
 *
 * @param ss the SimpleSetup instance to use for planning
 * 
 */
void planScenario1(ompl::geometric::SimpleSetup &ss)
{
    // TODO: Plan for chain_box in the plane, and store the path in path1.txt. 

    auto planner = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation());

    ss.setPlanner(planner);
    ompl::base::PlannerStatus solved = ss.solve(10.0);
    std::cout << "Planning complete" << std::endl;
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().printAsMatrix(std::cout);
        std::ofstream outFile("path1.txt");
        ss.getSolutionPath().printAsMatrix(outFile);
        outFile.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}


/**
 * Benchmark different PRM sampling strategies for Scenario1:
 * Benchmark (Uniform, Gaussian, Bridge, and Obstacle)-based sampling for scenario1.
 * The example in this benchmarking demo could help.
 * In your report, explain which sampling strategy performed the best and why. Include
 * supporting figures from PlannerArena.
 *
 * @param ss the SimpleSetup instance to use for planning
 * 
 */
void benchScenario1(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark PRM with uniform, bridge, gaussian, and obstacle-based Sampling. Do 20 trials with 20 seconds each 
    using namespace ompl;

    // Benchmark settings
    double runtime_limit = 20.0;
    double memory_limit = 1024.0;
    int run_count = 20;

    // Create benchmark object
    tools::Benchmark b(ss, "ChainBox_Narrow");
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);

    // 1. Uniform Sampling
    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
            return std::make_shared<base::UniformValidStateSampler>(si);
        });
    b.addPlanner(std::make_shared<geometric::PRM>(ss.getSpaceInformation()));
    b.addExperimentParameter("sampler_type", "STRING", "Uniform");
    b.benchmark(request);
    b.saveResultsToFile("benchmark_uniform.log");

    // 2. Gaussian Sampling
    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
            return std::make_shared<base::GaussianValidStateSampler>(si);
        });
    b.addPlanner(std::make_shared<geometric::PRM>(ss.getSpaceInformation()));
    b.addExperimentParameter("sampler_type", "STRING", "Gaussian");
    b.benchmark(request);
    b.saveResultsToFile("benchmark_gaussian.log");

    // 3. Obstacle-based Sampling
    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
            return std::make_shared<base::ObstacleBasedValidStateSampler>(si);
        });
    b.addPlanner(std::make_shared<geometric::PRM>(ss.getSpaceInformation()));
    b.addExperimentParameter("sampler_type", "STRING", "ObstacleBased");
    b.benchmark(request);
    b.saveResultsToFile("benchmark_obstacle.log");

    // 4. Bridge-test Sampling
    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
            return std::make_shared<base::BridgeTestValidStateSampler>(si);
        });
    b.addPlanner(std::make_shared<geometric::PRM>(ss.getSpaceInformation()));
    b.addExperimentParameter("sampler_type", "STRING", "BridgeTest");
    b.benchmark(request);
    b.saveResultsToFile("benchmark_bridge.log");

    std::cout << "Benchmarking complete. Results saved to benchmark_*.log" << std::endl;
}



/**
 * Solve Scenario2 by finding a path with maximum workspace clearance.
 * The environment obstacles, start, and goal are already provided to you in makeScenario2.
 * Implement a clearance optimization function. This tutorial could be helpful optimal planning
 * tutorial.
 * Calculating the true c-space clearance is impractical in most situations. Simply approximat-
 * ing a workspace clearance by calculating the distance from the box center to the obstacle
 * corners. will suffice for this homework.
 * Choose an asymptotically optimal planner to solve the problem and submit your solutions as
 * clear.txt and clear.gif from the visualizer.
 */
void planScenario2(ompl::geometric::SimpleSetup &ss)
{
    // TODO: Plan for chain_box in the plane, with a clearance optimization objective, with an Asymptoticallly optimal planner of your choice and store the path in path2.txt
    // Objective: maximize the minimum workspace clearance
    auto clearanceObj = std::make_shared<ompl::base::MaximizeMinClearanceObjective>(ss.getSpaceInformation());
    ss.setOptimizationObjective(clearanceObj);

    // Use an asymptotically optimal planner (RRT*)
    auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
    planner->setGoalBias(0.2);   // Bias towards goal (helps convergence)
    planner->setRange(0.7);       // Step size

    ss.setPlanner(planner);

    std::cout << "Solving Scenario 2 (Maximize Clearance)\n";
    //ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01);  // check every 1% of edge length
    ompl::base::PlannerStatus solved = ss.solve(60.0);

    if (solved)
    {
        std::cout << "Solution found with clearance optimization!\n";
        //ss.simplifySolution();

        // Save the result
        std::ofstream outFile("clear.txt");
        ss.getSolutionPath().printAsMatrix(outFile);
        outFile.close();

        std::cout << "Path saved to clear.txt\n";

        // Optional: compute min clearance for reporting
        auto path = ss.getSolutionPath();
        double minClearance = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < path.getStateCount(); ++i)
        {
            double c = ss.getSpaceInformation()->getStateValidityChecker()->clearance(path.getState(i));
            if (c < minClearance)
                minClearance = c;
        }
        std::cout << "Minimum workspace clearance along path: " << minClearance << std::endl;
    }
    else
    {
        std::cout << "No solution found.\n";
    }
}


/**
 * Benchmark rrt*, prm*, and rrt# using your custom clearance objective. In your report, 
 * identify the most effective planner and explain the results with figures from PlannerArena.
 * Include the benchmarking figures from plannerarena in your submission.
 * 
 * @param ss the SimpleSetup instance to use for planning
 */
void benchScenario2(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark RRT*, PRM*, RRT# for 10 trials with 60 secounds timeout.

    using namespace ompl;

    // --- Use the same clearance objective
    auto clearanceObj = std::make_shared<ompl::base::MaximizeMinClearanceObjective>(ss.getSpaceInformation());
    ss.setOptimizationObjective(clearanceObj);

    //
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.02);

    // --- Benchmark config: 10 trials, 60s each,---
    double runtime_limit = 60.0, memory_limit = 1024.0;
    int run_count = 10;
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(ss, "ChainBox_Clearance");

    // --- RRT* ---
    {
        auto p = std::make_shared<geometric::RRTstar>(ss.getSpaceInformation());
        p->setRange(0.7);
        p->setGoalBias(0.2);
        b.addPlanner(p);
    }

    // --- PRM* ---
    {
        auto p = std::make_shared<geometric::PRMstar>(ss.getSpaceInformation());
        b.addPlanner(p);
    }

    // --- RRT# (RRTsharp) ---
    {
        auto p = std::make_shared<geometric::RRTsharp>(ss.getSpaceInformation());
        p->setRange(0.7);
        p->setGoalBias(0.2);
        b.addPlanner(p);
    }

    // --- Run and save ---
    b.benchmark(request);

    b.saveResultsToFile("clearance_benchmark.log");

    std::cout << "Benchmarking complete. Results written to clearance_benchmark.log\n";

}


/**
 * Implements the state space for the chainbox. 
 * 
 * Take the following into account:
 * The chainbox robot has a square base where each side is 1 and a 4 link chain each link size
 * 1, with the first joint at the center of the box. See included gifs for a visualization.
 * The box center of the robot must remain within a [5,-5], boundary at all times.
 * For guidance on custom state spaces, refer to this tutorial.
 * In your report provide the topological space of this robot.
 * Pro Tip: You can use functions from {KinematicChain.h} to implement this composite
 * space
 * 
 * @return std::shared_ptr<ompl::base::CompoundStateSpace> the created state space
 * 
 */

std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace()
{   //TODO Create the Chainbox ConfigurationSpace
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    // Create the 2D position space for the box
    auto boxPosition = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    boxPosition->setBounds(-5, 5);
    space->addSubspace(boxPosition, 1.0);

    // Create the orientation space for the box
    auto boxOrientation = std::make_shared<ompl::base::RealVectorStateSpace>(1);
    boxOrientation->setBounds(-M_PI, M_PI);
    space->addSubspace(boxOrientation, 1.0);

    // Create the chain space
    auto chain = std::make_shared<KinematicChainSpace>(4, 1.0);
    space->addSubspace(chain, 1.0);

    return space;
}

/**
 * Implements the collision checking for the chainbox robot. 
 * 
 * Take the following into account:
 * The robot base center must remain within [5,-5] at all times. 
 * The chain should not self-intersect with itself or the box (except for the first link).
 * Pro Tip: You can heavily leverage and modify KinematicChain.h to achieve this collision
 * checking. One approach is to implement everything by modifying KinematicChain.h.
 * Then implementing setupCollisionChecker will be only 1 line.
 * 
 * @param ss the SimpleSetup instance to which the state validity checker is to be set
 * @param env the environment in which the robot is to be checked for collisions
 */
void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env)
{
    auto space = ss.getSpaceInformation()->getStateSpace()->as<ompl::base::CompoundStateSpace>();
    auto chain = space->getSubspace(2)->as<KinematicChainSpace>();
    chain->setEnvironment(&env);
    ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ss.getSpaceInformation()));
}

    
int main(int argc, char **argv)
{
    int scenario; 
    Environment env;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 3);

    switch (scenario)
    {
        case 1:
            makeScenario1(env, startVec, goalVec);
            break;
        case 2:
            makeScenario2(env, startVec, goalVec);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    auto space = createChainBoxSpace();
    ompl::geometric::SimpleSetup ss(space);

    setupCollisionChecker(ss, env);

    //setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    switch (scenario)
    {
        case 1:
            planScenario1(ss);
            benchScenario1(ss);
            break;
        case 2:
            planScenario2(ss);
            benchScenario2(ss);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }
}
