#ifndef OMPL_GEOMETRIC_PLANNERS_RTP_RTP_
#define OMPL_GEOMETRIC_PLANNERS_RTP_RTP_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        class RTP : public base::Planner
        {
        public:
            RTP(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

            ~RTP() override;

            void clear() override; // Reset the planner

            // Main entry: returns exact path if found, else approximate path to closest-to-goal state
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            // Export vertices/edges for inspection
            void getPlannerData(base::PlannerData &data) const override;

            // Tunables
            void setGoalBias(double gb) { goalBias_ = gb; }
            double getGoalBias() const { return goalBias_; }

            // Whether to add intermediate states along edges
            void setIntermediateStates(bool b) {addIntermediateStates_ = b; }
            bool getIntermediateStates() const { return addIntermediateStates_; }

            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }
            
            void setup() override; // Prepare the planner for use

        protected:
            /** A node in the tree */
            class Motion
            {
            public:
                Motion() = default;
                explicit Motion(const base::SpaceInformationPtr &si) : state(si->allocState()) {}
                ~Motion() = default;

                base::State *state{nullptr};
                Motion *parent{nullptr};
            };

            void freeMemory();

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            // Data
            base::StateSamplerPtr sampler_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;
            RNG rng_;

            // Params
            double goalBias_{0.05};     // probability to sample from goal region (higher for better convergence)
            bool addIntermediateStates_{true};  // enable for smoother paths

            // For getPlannerData
            Motion *lastGoalMotion_{nullptr};
        };
    } // namespace geometric
} // namespace ompl

#endif // OMPL_GEOMETRIC_PLANNERS_RTP_RTP_
