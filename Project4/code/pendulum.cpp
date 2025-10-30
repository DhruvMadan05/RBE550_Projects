///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Dhruv Madan
//////////////////////////////////////

// include library headers
#include <iostream>
#include <cmath>

// Base headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Control headers
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

// 3 planners to choose from
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

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
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    double tau = u[0];

    qdot.resize(2);
    qdot[0] = omega;
    qdot[1] = -g * std::cos(theta) + tau;
}

ompl::control::SimpleSetupPtr createPendulum(double /* torque */)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    return nullptr;
}

void planPendulum(ompl::control::SimpleSetupPtr &/* ss */, int /* choice */)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */)
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
