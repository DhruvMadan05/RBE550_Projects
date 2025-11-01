///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Dhruv Madan
//////////////////////////////////////

#include <iostream>
#include <cmath>
#include <fstream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/StateValidityChecker.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/sst/SST.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "AO-RRT.h"

// Your projection for the car
/**
 * Projection for the car state space.
 * The car state is represented as (x, y, theta, v), where
 * - x, y are the position coordinates
 * - theta is the heading angle
 * - v is the velocity
 * 
 * The projection maps the state to a 2D space defined by (x, y).
 */
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    /** Get the dimension of the projection space. 
     * @return The dimension of the projection space (2 for car).
    */
    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    /** Project the given state to the projection space.
     * @param state The state to project.
     * @param projection The resulting projection vector.
    */
    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection(0) = s->values[0];  // x
        projection(1) = s->values[1];  // y
    }
};

/** ODE defining the car dynamics.
 * State vector: (x, y, theta, v)
 * Control vector: (omega, alpha)
 * The dynamics are defined as:
 * dx/dt = v * cos(theta)
 * dy/dt = v * sin(theta)
 * dtheta/dt = omega
 * dv/dt = alpha
 * @param q The current state vector.
 * @param control The control input.
 * @param qdot The resulting state derivative.
 */
void carODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    // q has size 4
    double x = q[0];
    double y = q[1];
    double theta = q[2];
    double v = q[3];

    // control is 2D: (omega, alpha)
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    double omega = u[0];
    double alpha = u[1];

    // Compute the state derivatives
    qdot.resize(4);
    qdot[0] = v * std::cos(theta);
    qdot[1] = v * std::sin(theta);
    qdot[2] = omega;
    qdot[3] = alpha;
}

/**
 * Post-integration event to normalize the heading angle theta.
 * Ensures that theta remains within the range [-pi, pi].
 * @param state The initial state before integration.
 * @param control The control input applied.
 * @param duration The duration of integration.
 * @param result The resulting state after integration.
 */
void carPostIntegration(const ompl::base::State *state, const ompl::control::Control *control,
                        double duration, ompl::base::State *result)
{
    auto *s = result->as<ompl::base::RealVectorStateSpace::StateType>();
    double &theta = s->values[2];
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
}

/** Create a street environment with predefined obstacles. */
void makeStreet(std::vector<Rectangle> &  obstacles )
{
    // Do not change the obstacles here. These are the same obstacles used for grading.
    obstacles.emplace_back(Rectangle{5.0, -2.0, 7, 5});
    obstacles.emplace_back(Rectangle{-4, 5, 16, 2});
    obstacles.emplace_back(Rectangle{-4, -2, 7, 4});
    obstacles.emplace_back(Rectangle{8, 3, 4, 2});
}

/** Validity checker for the car state space. 
 * The car is represented as a point with a small radius for collision checking. 
*/
class CarValidityChecker : public ompl::base::StateValidityChecker
{
public:
    CarValidityChecker(const ompl::base::SpaceInformationPtr &si, const std::vector<Rectangle> &obs) : ompl::base::StateValidityChecker(si), obstacles_(obs)
    {
    }

    /** Check if the given state is valid (collision-free and within bounds).
     * @param state The state to be checked.
     * @return True if the state is valid, false otherwise.
    */
    bool isValid(const ompl::base::State *state) const override
    {
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        double x = s->values[0];
        double y = s->values[1];
        double theta = s->values[2];
        double v = s->values[3];

        // workspace bounds check
        if (x < -10 || x > 10 || y < -10 || y > 10)
            return false;

        // velocity bounds check
        if (v < -1.0 || v > 1.0)
            return false;

        // simple collision check (treat car as small circle)
        return isValidCircle(x, y, 0.2, obstacles_);
    }

private:
    std::vector<Rectangle> obstacles_;
};

/** Create and setup the car's planning environment.
 * @param obstacles The vector to store the obstacles in the environment.
 * @return The SimpleSetup object configured for the car.
*/
ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    
    // State space: (x, y, theta, v)
    auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(4);
    ompl::base::RealVectorBounds stateBounds(4);
    stateBounds.setLow(0, -10.0);
    stateBounds.setHigh(0, 10.0);
    stateBounds.setLow(1, -10.0);
    stateBounds.setHigh(1, 10.0);
    stateBounds.setLow(2, -M_PI);
    stateBounds.setHigh(2, M_PI);
    stateBounds.setLow(3, -1.0);
    stateBounds.setHigh(3, 1.0);
    stateSpace->setBounds(stateBounds);

    // Register projection for KPIECE1
    stateSpace->registerProjection("CarProjection", std::make_shared<CarProjection>(stateSpace.get()));

    // Control space: (omega, alpha)
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(stateSpace, 2);
    ompl::base::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -0.3);
    controlBounds.setHigh(0, 0.3);
    controlBounds.setLow(1, -0.3);
    controlBounds.setHigh(1, 0.3);
    controlSpace->setBounds(controlBounds);

    // SimpleSetup object
    ompl::control::SimpleSetupPtr ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);

    // ODE solver and propagator
    ompl::control::ODESolverPtr odeSolver =
        std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &carPostIntegration));

    // Validity checker
    ss->setStateValidityChecker(std::make_shared<CarValidityChecker>(ss->getSpaceInformation(), obstacles));

    // Start and goal states
    ompl::base::ScopedState<> start(stateSpace);
    start[0] = 1.0;  // x
    start[1] = -5.0; // y
    start[2] = 0.0;  // heading
    start[3] = 0.0;  // velocity

    // Goal state
    ompl::base::ScopedState<> goal(stateSpace);
    goal[0] = 7.0;
    goal[1] = 4.0;
    goal[2] = 3.14;
    goal[3] = 0.0;

    ss->setStartAndGoalStates(start, goal, 0.5);

    // Optimization objective (path length)
    ss->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss->getSpaceInformation()));

    ss->setup();
    std::cout << "Car setup complete.\n";
    return ss;
}

/**
 * Plan a path for the car using the specified planner.
 * @param ss The SimpleSetup object configured for the car.
 * @param choice The planner choice (1: KPIECE1, 2: SST, 3: AO-RRT).
 */
void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    ompl::base::PlannerPtr planner;
    auto si = ss->getSpaceInformation();

    if (choice == 1)
    {
        std::cout << "Using KPIECE1...\n";
        planner = std::make_shared<ompl::control::KPIECE1>(si);
    }
    else if (choice == 2)
    {
        std::cout << "Using SST...\n";
        planner = std::make_shared<ompl::control::SST>(si);
    }
    else
    {
        std::cout << "Using AO-RRT...\n";
        planner = std::make_shared<ompl::control::AORRT>(si);
    }

    ss->setPlanner(planner);
    ompl::base::PlannerStatus solved = ss->solve(1000.0);

    if (solved)
    {
        std::cout << "Planner found a solution!\n";
        auto path = ss->getSolutionPath().asGeometric();
        path.printAsMatrix(std::cout);

        std::ofstream out("car_path.txt");
        path.printAsMatrix(out);
        out.close();
        std::cout << "Saved path to car_path.txt\n";
    }
    else
    {
        std::cout << "No solution found.\n";
    }
}

/** Benchmark the car planning problem using different planners. 
 * @param ss The SimpleSetup object configured for the car.
*/
void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // implement benchmarking logic for 3 planners
    // KPIECE1, SST, AO-RRT
    // Each planner must run 20 trials for 200 seconds each
    // use the benchmark class of OMPL

    // --- Configuration ---
    unsigned int runCount = 20; // repeat each planner 20 times
    double timeout = 200.0; // seconds per planner
    double memoryLimit = 1024.0; // MB
    std::string outputFile = "car_benchmark.log";

    // --- Benchmark setup ---
    ompl::tools::Benchmark benchmark(*ss, "CarBenchmark");

    // Add planners
    // benchmark.addPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<ompl::control::SST>(ss->getSpaceInformation()));
    // benchmark.addPlanner(std::make_shared<ompl::control::AORRT>(ss->getSpaceInformation()));

    // --- Experiment settings ---
    ompl::tools::Benchmark::Request request;
    request.maxTime = timeout;       // per run
    request.maxMem = memoryLimit;    // memory limit
    request.runCount = runCount;     // number of repetitions
    request.displayProgress = true;  // show progress in console

    std::cout << "\nRunning benchmark: " << runCount << " runs per planner, "
              << timeout << " seconds each...\n";

    // --- Run benchmark ---
    benchmark.benchmark(request);

    // --- Save results ---
    benchmark.saveResultsToFile(outputFile.c_str());
    std::cout << "\nBenchmark results saved to: " << outputFile << std::endl;
}

int main(int argc, char **argv)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) KPIECE1" << std::endl;
            std::cout << " (2) SST" << std::endl;
            std::cout << " (3) AO-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
