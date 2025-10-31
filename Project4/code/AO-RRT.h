///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Dhruv Madan
//////////////////////////////////////

#ifndef AORRT_H
#define AORRT_H

#include <ompl/base/Planner.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/control/PathControl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>
#include <vector>

namespace ompl
{
    namespace control
    {
        /** \brief Asymptotically Optimal RRT (AO-RRT)
         *
         *  Based on:
         *  K. Hauser and Y. Zhou, "Asymptotically optimal planning by feasible
         *  kinodynamic planning in a state–cost space," IEEE TRO, 2016.
         *
         *  Idea: run a normal kinodynamic RRT, but once we have a first solution
         *  with cost C*, we reject any newly proposed motion whose cost-to-come
         *  is >= C*. Whenever we find a better solution, we tighten C*.
         */
        class AORRT : public base::Planner
        {
        public:
            AORRT(const control::SpaceInformationPtr &si);

            ~AORRT() override;

            void setup() override;

            void clear() override;

            /** \brief Set the probability of sampling the goal directly. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the probability of sampling the goal directly. */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Solve the planning problem */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Get planner data */
            void getPlannerData(base::PlannerData &data) const override;

        protected:
            /** \brief Representation of a tree motion */
            struct Motion
            {
                Motion(const control::SpaceInformation *si)
                    : state_(si->allocState())
                    , control_(si->allocControl())
                    , parent_(nullptr)
                    , steps_(0)
                    , accCost_(base::Cost(std::numeric_limits<double>::infinity()))
                {
                }

                ~Motion() = default;

                base::State *state_;
                control::Control *control_;
                Motion *parent_;
                unsigned int steps_;
                base::Cost accCost_;
            };

            /** \brief Free all the memory allocated by this planner */
            void freeMemory();

            /** \brief Distance function for NN structure */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state_, b->state_);
            }

            /** \brief Select nearest node in the tree to the given state */
            Motion *nearest(const Motion *motion) const
            {
                return nn_->nearest(const_cast<Motion *>(motion));
            }

            /** \brief Add a new motion to the tree */
            void addMotion(Motion *motion)
            {
                nn_->add(motion);
            }

            /** \brief Build the final path from a solution motion */
            void buildSolutionPath(Motion *solution);

            control::SpaceInformation *siC_{nullptr};

            /** \brief Tree of motions (RRT) */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            control::ControlSamplerPtr controlSampler_;

            /** \brief Optimization objective (for cost-to-come & pruning) */
            base::OptimizationObjectivePtr opt_;

            /** \brief Global best (tightening) cost bound C* */
            base::Cost bestCost_;

            /** \brief Probability of sampling from goal */
            double goalBias_{0.05};

            /** \brief Random number generator */
            RNG rng_;

            /** \brief Cached solution path (to support intermediate solution callbacks) */
            std::vector<base::State *> bestPathStates_;
            std::vector<control::Control *> bestPathControls_;
            std::vector<unsigned int> bestPathSteps_;
        };
    }  // namespace control
}  // namespace ompl

#endif

