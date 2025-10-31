///////////////////////////////////////
// RBE550
// Project 4
// Authors: Dhruv Madan
//////////////////////////////////////

#include "AO-RRT.h"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/Path.h>
#include <ompl/control/PathControl.h>

using namespace ompl;
using namespace ompl::control;

AORRT::AORRT(const SpaceInformationPtr &si)
    : base::Planner(si, "AORRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    // expose goal bias as parameter (like RRT/SST)
    Planner::declareParam<double>("goal_bias", this, &AORRT::setGoalBias, &AORRT::getGoalBias, "0.:.05:1.");
}

AORRT::~AORRT()
{
    freeMemory();
}

void AORRT::setup()
{
    base::Planner::setup();
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }

    // get or create optimization objective
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
        }
        else
        {
            // default to path length objective
            OMPL_WARN("%s: No optimization objective specified. Using path length.", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }

    // start with infinite bound (no pruning yet)
    bestCost_ = opt_->infiniteCost();
}

void AORRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    bestCost_ = opt_ ? opt_->infiniteCost() : base::Cost(std::numeric_limits<double>::infinity());

    // clear cached solution
    for (auto *s : bestPathStates_)
        si_->freeState(s);
    bestPathStates_.clear();

    for (auto *c : bestPathControls_)
        siC_->freeControl(c);
    bestPathControls_.clear();
    bestPathSteps_.clear();
}

void AORRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &m : motions)
        {
            if (m->state_)
                si_->freeState(m->state_);
            if (m->control_)
                siC_->freeControl(m->control_);
            delete m;
        }
    }
}

void AORRT::buildSolutionPath(Motion *solution)
{
    // clear old cached path
    for (auto *s : bestPathStates_)
        si_->freeState(s);
    bestPathStates_.clear();
    for (auto *c : bestPathControls_)
        siC_->freeControl(c);
    bestPathControls_.clear();
    bestPathSteps_.clear();

    Motion *m = solution;
    while (m != nullptr)
    {
        bestPathStates_.push_back(si_->cloneState(m->state_));
        if (m->parent_ != nullptr)
        {
            bestPathControls_.push_back(siC_->cloneControl(m->control_));
            bestPathSteps_.push_back(m->steps_);
        }
        m = m->parent_;
    }

    // path is from goal -> start, so we'll reverse when adding to PathControl in solve()
}

base::PlannerStatus AORRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    base::Goal *goal = pdef_->getGoal().get();
    auto *goalRegion = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // add all start states
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state_, st);
        siC_->nullControl(motion->control_);
        motion->parent_ = nullptr;
        motion->steps_ = 0;
        motion->accCost_ = opt_->identityCost();  // cost-to-come = 0
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();

    const base::ReportIntermediateSolutionFn intermediateCallback = pdef_->getIntermediateSolutionCallback();

    // reusable samples
    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state_;
    control::Control *rctrl = rmotion->control_;
    base::State *xstate = si_->allocState();

    Motion *solution = nullptr;
    Motion *approxSolution = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    unsigned int iterations = 0;

    OMPL_INFORM("%s: starting planning with %u states", getName().c_str(), nn_->size());

    while (ptc == false)
    {
        // 1) sample a state, possibly from goal
        if (goalRegion && rng_.uniform01() < goalBias_ && goalRegion->canSample())
            goalRegion->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // 2) find nearest motion in the tree
        Motion *nmotion = nearest(rmotion);

        // 3) sample a control and duration
        controlSampler_->sample(rctrl);
        unsigned int cd = rng_.uniformInt(siC_->getMinControlDuration(), siC_->getMaxControlDuration());

        // 4) propagate
        unsigned int propCd = siC_->propagateWhileValid(nmotion->state_, rctrl, cd, rstate);

        if (propCd == cd)
        {
            // compute incremental cost (motion + control)
            base::Cost incMotionCost = opt_->motionCost(nmotion->state_, rstate);
            base::Cost incControlCost = opt_->controlCost(rctrl, cd);
            base::Cost incCost = opt_->combineCosts(incMotionCost, incControlCost);

            // new cost-to-come
            base::Cost newCost = opt_->combineCosts(nmotion->accCost_, incCost);

            // AO-RRT pruning: if we already have a solution, only accept better-than-best
            // before first solution: bestCost_ == inf, so always true
            bool accept = opt_->isCostBetterThan(newCost, bestCost_);

            if (accept)
            {
                // create and insert new motion
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state_, rstate);
                siC_->copyControl(motion->control_, rctrl);
                motion->parent_ = nmotion;
                motion->steps_ = cd;
                motion->accCost_ = newCost;

                nn_->add(motion);

                // check goal
                double dist = 0.0;
                bool reach = goal->isSatisfied(motion->state_, &dist);
                if (reach)
                {
                    // better than any seen so far -> tighten bound
                    if (opt_->isCostBetterThan(motion->accCost_, bestCost_))
                    {
                        bestCost_ = motion->accCost_;
                        solution = motion;
                        approxDist = dist;

                        // rebuild cached path for intermediate callbacks / final output
                        buildSolutionPath(solution);

                        OMPL_INFORM("%s: found solution with cost %.6f", getName().c_str(),
                                    solution->accCost_.value());

                        if (intermediateCallback)
                        {
                            // create const vector for callback
                            std::vector<const base::State *> constStates;
                            constStates.reserve(bestPathStates_.size());
                            // bestPathStates_ is goal->start, need to push reversed
                            for (int i = (int)bestPathStates_.size() - 1; i >= 0; --i)
                                constStates.push_back(bestPathStates_[i]);
                            intermediateCallback(this, constStates, bestCost_);
                        }

                        // we do NOT break: AO-RRT keeps going to improve the solution
                    }
                }
                else
                {
                    // if we don't have a solution yet, keep track of best approximate
                    if (!solution && dist < approxDist)
                    {
                        approxDist = dist;
                        approxSolution = motion;
                    }
                }
            }
        }

        iterations++;
    }

    bool solved = false;
    bool approximate = false;

    Motion *finalSol = solution ? solution : approxSolution;

    if (finalSol)
    {
        // if we reached here through approximate path (no exact goal)
        approximate = (solution == nullptr);

        // if we solved just now but don't have cached path (e.g. approximate)
        if (bestPathStates_.empty())
            buildSolutionPath(finalSol);

        // build PathControl to hand to OMPL
        auto path(std::make_shared<control::PathControl>(si_));
        double dt = siC_->getPropagationStepSize();

        // bestPathStates_ is stored from goal -> start, so we append reversed
        int N = static_cast<int>(bestPathStates_.size());
        for (int i = N - 1; i >= 1; --i)
        {
            path->append(bestPathStates_[i], bestPathControls_[i - 1],
                         bestPathSteps_[i - 1] * dt);
        }
        path->append(bestPathStates_[0]);

        pdef_->addSolutionPath(path, approximate, approxDist, getName());
        solved = !approximate;
    }

    si_->freeState(xstate);
    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    if (rmotion->control_)
        siC_->freeControl(rmotion->control_);
    delete rmotion;

    OMPL_INFORM("%s: created %u states in %u iterations", getName().c_str(), nn_->size(), iterations);

    return {solved, approximate};
}

void AORRT::getPlannerData(base::PlannerData &data) const
{
    // Add basic planner metadata
    Planner::getPlannerData(data);

    if (!nn_)
        return;

    // Collect all motions from the tree
    std::vector<Motion *> motions;
    nn_->list(motions);

    // Mark the goal vertex if we have a saved solution
    if (!bestPathStates_.empty())
        data.addGoalVertex(base::PlannerDataVertex(bestPathStates_.front()));

    // Add all edges between parent and child
    for (auto *m : motions)
    {
        if (m->parent_ != nullptr)
        {
            data.addEdge(
                base::PlannerDataVertex(m->parent_->state_),
                base::PlannerDataVertex(m->state_));
        }
        else
        {
            // root node (start state)
            data.addStartVertex(base::PlannerDataVertex(m->state_));
        }
    }
}


