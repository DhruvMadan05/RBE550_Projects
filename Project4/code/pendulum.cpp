///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Dhruv Madan
//////////////////////////////////////

// include library headers
#include <iostream>
#include <cmath>
#include <fstream>

// Base headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

// Control headers
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/tools/benchmark/Benchmark.h>

// 3 planners to choose from
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

#include "CollisionChecking.h"

static const double g = 9.81;  // acceleratio ndue to gravity 9.81 m/s^2


// Your projection for the pendulum
/**
 * Projection for the pendulum state space.
 * The pendulum state is represented as (theta, omega), where
 * - theta is the angular position
 * - omega is the angular velocity
 * 
 * The projection maps the state to a 2D space defined by (theta, omega).
 */
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    /**
     * Get the dimension of the projection space.
     * @return The dimension of the projection space (2 for pendulum).
     */
    unsigned int getDimension() const override
    {
        // The dimension of the pendulum projection is 2
        // - angular position, theta
        // - angular velocity, omega
        return 2;
    }

    /**
     * Project the given state to the projection space.
     * @param state The state to be projected.
     * @param projection The resulting projection vector.
     */
    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection(0) = s->values[0];  // theta
        projection(1) = s->values[1];  // omega
    }
};


/**
 * ODE function for the pendulum dynamics.
 * 
 * @param q The current state vector (theta, omega).
 * @param control The control input (torque).
 * @param qdot The resulting state derivative vector (dtheta/dt, domega/dt).
 */
void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // q has size 2
    // unpack q into theta and omega
    double theta = q[0];
    double omega = q[1];

    // control is 1D: torque
    // get the torque value from the control
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    double tau = u[0];

    qdot.resize(2);
    qdot[0] = omega;
    qdot[1] = -g * std::cos(theta) + tau;
}


/**
 * Ensure theta remains within [-pi, pi].
 * 
 * @param state The initial state before integration.
 * @param control The control input applied during integration.
 * @param duration The duration of the integration.
 * @param result The resulting state after integration (to be modified).
 */
void pendulumPostIntegration(const ompl::base::State *state, const ompl::control::Control *control,
                               double duration, ompl::base::State *result)
{
    // Ensure theta remains within [-pi, pi] after integration
    auto *s = result->as<ompl::base::RealVectorStateSpace::StateType>();
    double &theta = s->values[0];
    // Wrap theta to be within [-pi, pi]
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
}


/**
 * State validity checker for the pendulum.
 * Considers all states valid as long as omega is within bounds.
 */
class PendulumValidityChecker : public ompl::base::StateValidityChecker
{
public:
    PendulumValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    /**
     * Check if the given state is valid.
     * @param state The state to be checked.
     * @return True if the state is valid, false otherwise.
     */
    bool isValid(const ompl::base::State *state) const override
    {
        // For the pendulum, we can consider all states valid as long as omega is within bounds
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        double omega = s->values[1];
        return (omega >= -10.0 && omega <= 10.0);
    }
};


/**
 * Create and setup the pendulum's state space, control space, validity checker.
 * @param torque The maximum torque that can be applied to the pendulum.
 * @return A pointer to the SimpleSetup object for the pendulum.
 */
ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // Create the state space for the pendulum
    auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // First component: SO(2) for theta (continuous circle)
    // Second component: R(1) for omega
    // auto thetaSpace = std::make_shared<ompl::base::SO2StateSpace>();
    // auto omegaSpace = std::make_shared<ompl::base::RealVectorStateSpace>(1);

    // auto stateSpace = thetaSpace + omegaSpace;

    // set omega bounds
    ompl::base::RealVectorBounds omegaBounds(1);
    omegaBounds.setLow(-10);
    omegaBounds.setHigh(10);
    //stateSpace->as<ompl::base::CompoundStateSpace>()->getSubspace(1)->as<ompl::base::RealVectorStateSpace>()->setBounds(omegaBounds);
    //omegaSpace->setBounds(omegaBounds);

    // Set the bounds for theta and omega
    ompl::base::RealVectorBounds stateBounds(2);
    stateBounds.setLow(0, -M_PI);   // theta lower bound
    stateBounds.setHigh(0, M_PI);   // theta upper bound
    stateBounds.setLow(1, -10.0);   // omega lower bound
    stateBounds.setHigh(1, 10.0);   // omega upper bound
    stateSpace->setBounds(stateBounds); 

    // Register the projection for KPIECE planner
    stateSpace->registerProjection("PendulumProjection", std::make_shared<PendulumProjection>(stateSpace.get()));

    // Create the control space for the pendulum
    // 1D control space for torque
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(stateSpace, 1);
    // Set the bounds for the control (torque)
    ompl::base::RealVectorBounds controlBounds(1);
    controlBounds.setLow(0, -torque);  // torque lower bound
    controlBounds.setHigh(0, torque); // torque upper bound
    controlSpace->setBounds(controlBounds); 

    // Create the SimpleSetup object
    ompl::control::SimpleSetupPtr ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace); 

    // set the ode solver
    ompl::control::ODESolverPtr odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE);
    //ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &pendulumPostIntegration));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // Set the state validity checker
    ss->setStateValidityChecker(std::make_shared<PendulumValidityChecker>(ss->getSpaceInformation()));

    // set the start and goal states
    ompl::base::ScopedState<> start(stateSpace);
    start[0] = -M_PI / 2.0;  // initial theta
    start[1] = 0.0;  // initial omega 
    ompl::base::ScopedState<> goal(stateSpace);
    goal[0] = M_PI / 2.0;  // goal theta
    goal[1] = 0.0;   // goal omega

    // Set the start and goal states with a tolerance
    ss->setStartAndGoalStates(start, goal, 0.1); // 0.1 tolerance

    ss->setup();
    std::cout << "Pendulum setup complete." << std::endl;

    return ss;
}

/**
 * Plan for the pendulum using the specified planner.
 * @param ss The SimpleSetup object for the pendulum.
 * @param choice The planner choice (1: RRT, 2: EST, 3: KPIECE1).
 */
void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    ompl::base::PlannerPtr planner;

    if (choice == 1)
        planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
    else if (choice == 2)
        planner = std::make_shared<ompl::control::EST>(ss->getSpaceInformation());
    else
        planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());

    ss->setPlanner(planner);
    ompl::base::PlannerStatus solved = ss->solve(5.0);

    if (solved)
    {
        std::cout << "Planner found a solution!\n";
        auto path = ss->getSolutionPath().asGeometric();
        //path.interpolate(400);  // interpolate the path for smoother output
        path.printAsMatrix(std::cout);
        // Add the path to a file
        std::ofstream outFile("pendulum_path.txt");
        path.printAsMatrix(outFile);
        outFile.close();
        std::cout << "Path saved to pendulum_path.txt\n"; 

    }
    else
        std::cout << "No solution found.\n";

}

/**
 * Benchmarking for the pendulum.
 * Using planners RRT, EST, KPIECE1. all with torque limits of 3
 * @param ss The SimpleSetup object for the pendulum.
 */
void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // Always use a torque limit of 3 for benchmarking
    double benchmarkTorque = 3.0;

    // Create a new pendulum setup with fixed torque = 3
    ompl::control::SimpleSetupPtr benchSS = createPendulum(benchmarkTorque);

    // Create the benchmark object
    ompl::tools::Benchmark b(*benchSS, "PendulumBenchmark_Torque3");

    // Add planners to benchmark
    b.addPlanner(std::make_shared<ompl::control::RRT>(benchSS->getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::control::EST>(benchSS->getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::control::KPIECE1>(benchSS->getSpaceInformation()));

    // Configure benchmark parameters
    ompl::tools::Benchmark::Request req;
    req.maxTime = 30.0;          // seconds per planner run
    req.maxMem = 1024.0;         // MB memory limit
    req.runCount = 20;           // number of runs per planner
    req.displayProgress = true;  // show progress bar

    // Run the benchmark
    std::cout << "Running benchmark with torque = 3...\n";
    b.benchmark(req);

    // Save results
    b.saveResultsToFile("pendulum_benchmark_torque3.log");
    std::cout << "Benchmark results saved to pendulum_benchmark_torque3.log\n";
}

int main(int argc, char **argv)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) EST" << std::endl;
            std::cout << " (3) KPIECE1" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
