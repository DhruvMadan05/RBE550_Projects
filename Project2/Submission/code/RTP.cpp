#include "RTP.h"
#include "CollisionChecking.h"

#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/tools/config/SelfConfig.h"

using namespace ompl;
using namespace ompl::base;
using namespace ompl::geometric;

/*
* Constructor
* @param si: Space information
* @param addIntermediateStates: Whether to add intermediate states along edges for smoother paths
*                              (set to false for faster planning)
*/
RTP::RTP(const SpaceInformationPtr &si, bool addIntermediateStates)
    : Planner(si, addIntermediateStates ? "RTPintermediate" : "RTP")
{
    specs_.approximateSolutions = true; // we may return an approximate path
    specs_.directed = true;

    addIntermediateStates_ = addIntermediateStates;

    // Declare params for RTP
    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RTP::setIntermediateStates, &RTP::getIntermediateStates, "0,1");
}

RTP::~RTP()
{
    freeMemory();
}

// Reset the planner
void RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

// Prepare the planner for use
void RTP::setup()
{
    Planner::setup();

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

// Free all allocated memory
void RTP::freeMemory()
{
    if (!nn_) return;
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (auto *m : motions)
    {
        if (m->state)
            si_->freeState(m->state);
        delete m;
    }
}

// Main entry: returns exact path if found, else approximate path to closest-to-goal state
PlannerStatus RTP::solve(const PlannerTerminationCondition &ptc)
{
    checkValidity(); // Check if setup and problem definition are valid

    Goal *goal = pdef_->getGoal().get();
    auto *goalRegion = dynamic_cast<GoalSampleableRegion *>(goal);

    // Seed tree with all valid starts
    while (const State *st = pis_.nextStart())
    {
        auto *m = new Motion(si_);
        si_->copyState(m->state, st);
        nn_->add(m);
    }

    // Check if we have any valid initial states
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return PlannerStatus::INVALID_START;
    }

    // Initialize the sampler if needed
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // Inform about the number of start states
    OMPL_INFORM("%s: starting with %u start states", getName().c_str(), nn_->size());

    // Main loop
    Motion *exactSolution = nullptr;    // will hold solution when goal satisfied
    Motion *approxSolution = nullptr;   // best-so-far w.r.t. goal distance
    double approxDist = std::numeric_limits<double>::infinity();

    // Temporary motion used for sampling
    auto *randMotion = new Motion(si_);
    State *rstate = randMotion->state;

    // Iterate until termination condition is met
    while (!ptc)
    {
        // 1) Select a random configuration qa from the existing Random Tree
        std::vector<Motion *> treeNodes;
        nn_->list(treeNodes);
        if (treeNodes.empty()) break; // Safety check
        
        Motion *randomTreeNode = treeNodes[rng_.uniformInt(0, treeNodes.size() - 1)];

        // 2) Sample a random configuration qb from the configuration space
        // With a small probability, select the goal configuration as qb instead
        if (goalRegion && rng_.uniform01() < goalBias_ && goalRegion->canSample())
            goalRegion->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // 3) First check if the sampled state itself is valid
        if (!si_->isValid(rstate))
            continue;
            
        // 4) Check whether the straight-line path between qa and qb is valid
        // If valid, add the path from qa to qb to the tree (TRUE RTP: no steering!)
        if (si_->checkMotion(randomTreeNode->state, rstate))
        {
            Motion *newNode = nullptr;

            if (addIntermediateStates_)
            {
                std::vector<State *> states;
                const unsigned int segCount = si_->getStateSpace()->validSegmentCount(randomTreeNode->state, rstate);

                // getMotionStates returns a sequence including both endpoints if requested
                if (si_->getMotionStates(randomTreeNode->state, rstate, states, segCount, true, true))
                    si_->freeState(states[0]); // we already have randomTreeNode->state

                for (std::size_t i = 1; i < states.size(); ++i)
                {
                    // Verify each intermediate state is valid before adding to tree
                    if (!si_->isValid(states[i]))
                    {
                        // Free remaining states if we hit an invalid one
                        for (std::size_t j = i; j < states.size(); ++j)
                            si_->freeState(states[j]);
                        break;
                    }
                    
                    auto *m = new Motion;
                    m->state = states[i];
                    m->parent = randomTreeNode;
                    nn_->add(m);
                    randomTreeNode = m;
                }
                newNode = randomTreeNode;
            }
            else
            {
                auto *m = new Motion(si_);
                si_->copyState(m->state, rstate);  // Use original sample qb, not interpolated target
                m->parent = randomTreeNode;
                nn_->add(m);
                newNode = m;
            }

            // 5) Check goal satisfaction & track approximate best
            double d2goal = 0.0;
            const bool satisfied = goal->isSatisfied(newNode->state, &d2goal);
            if (satisfied)
            {
                approxDist = d2goal;
                exactSolution = newNode;
                break; // exact solution found
            }
            if (d2goal < approxDist)
            {
                approxDist = d2goal;
                approxSolution = newNode;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    Motion *solution = exactSolution;

    // If no exact solution, return approximate solution if available
    if (!solution)
    {
        solution = approxSolution;
        approximate = (solution != nullptr);
    }

    // Construct the solution path
    if (solution)
    {
        lastGoalMotion_ = solution;

        // Reconstruct path by backtracking parents
        std::vector<Motion *> pathMotions;
        for (Motion *m = solution; m != nullptr; m = m->parent)
            pathMotions.push_back(m);

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = static_cast<int>(pathMotions.size()) - 1; i >= 0; --i)
            path->append(pathMotions[i]->state);

        pdef_->addSolutionPath(path, approximate, approxDist, getName());
        solved = true;
    }

    // Free the temporary random motion
    if (randMotion->state) si_->freeState(randMotion->state);
    delete randMotion;

    OMPL_INFORM("%s: tree has %u states", getName().c_str(), nn_->size());
    return {solved, approximate};
}

// Export vertices/edges for inspection
void RTP::getPlannerData(PlannerData &data) const
{
    Planner::getPlannerData(data);
    std::vector<Motion *> motions;
    if (nn_) nn_->list(motions);

    // If the solution is approximate, we cannot identify which motion is the goal
    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(PlannerDataVertex(lastGoalMotion_->state));

    // Add the start and goal vertices and the edges between them
    for (auto *m : motions)
    {
        if (m->parent == nullptr)
            data.addStartVertex(PlannerDataVertex(m->state));
        else
            data.addEdge(PlannerDataVertex(m->parent->state), PlannerDataVertex(m->state));
    }
}
