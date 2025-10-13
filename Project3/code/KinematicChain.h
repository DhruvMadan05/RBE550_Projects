/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Bryant Gipson, Mark Moll */

#ifndef OMPL_DEMO_KINEMATIC_CHAIN_
#define OMPL_DEMO_KINEMATIC_CHAIN_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>


#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

// a 2D line segment
struct Segment
{
    Segment(double p0_x, double p0_y, double p1_x, double p1_y) : x0(p0_x), y0(p0_y), x1(p1_x), y1(p1_y)
    {
    }
    double x0, y0, x1, y1;
};

// the robot and environment are modeled both as a vector of segments.
using Environment = std::vector<Segment>;

// simply use a random projection
class KinematicChainProjector : public ompl::base::ProjectionEvaluator
{
public:
    KinematicChainProjector(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space)
    {
        int dimension = std::max(2, (int)ceil(log((double)space->getDimension())));
        projectionMatrix_.computeRandom(space->getDimension(), dimension);
    }
    unsigned int getDimension() const override
    {
        return projectionMatrix_.mat.rows();
    }
    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        std::vector<double> v(space_->getDimension());
        space_->copyToReals(v, state);
        projectionMatrix_.project(&v[0], projection);
    }

protected:
    ompl::base::ProjectionMatrix projectionMatrix_;
};

class KinematicChainSpace : public ompl::base::RealVectorStateSpace
{
public:
    KinematicChainSpace(unsigned int numLinks, double linkLength, Environment *env = nullptr)
      : ompl::base::RealVectorStateSpace(numLinks), linkLength_(linkLength), environment_(env)
    {
        ompl::base::RealVectorBounds bounds(numLinks);
        bounds.setLow(-boost::math::constants::pi<double>());
        bounds.setHigh(boost::math::constants::pi<double>());
        setBounds(bounds);
    }

    void registerProjections() override
    {
        registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    }

    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override
    {
        const auto *cstate1 = state1->as<StateType>();
        const auto *cstate2 = state2->as<StateType>();
        double theta1 = 0., theta2 = 0., dx = 0., dy = 0., dist = 0.;

        for (unsigned int i = 0; i < dimension_; ++i)
        {
            theta1 += cstate1->values[i];
            theta2 += cstate2->values[i];
            dx += cos(theta1) - cos(theta2);
            dy += sin(theta1) - sin(theta2);
            dist += sqrt(dx * dx + dy * dy);
        }

        return dist * linkLength_;
    }

    void enforceBounds(ompl::base::State *state) const override
    {
        auto *statet = state->as<StateType>();

        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double v = fmod(statet->values[i], 2.0 * boost::math::constants::pi<double>());
            if (v < -boost::math::constants::pi<double>())
                v += 2.0 * boost::math::constants::pi<double>();
            else if (v >= boost::math::constants::pi<double>())
                v -= 2.0 * boost::math::constants::pi<double>();
            statet->values[i] = v;
        }
    }

    bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override
    {
        bool flag = true;
        const auto *cstate1 = state1->as<StateType>();
        const auto *cstate2 = state2->as<StateType>();

        for (unsigned int i = 0; i < dimension_ && flag; ++i)
            flag &= fabs(cstate1->values[i] - cstate2->values[i]) < std::numeric_limits<double>::epsilon() * 2.0;

        return flag;
    }

    void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
                     ompl::base::State *state) const override
    {
        const auto *fromt = from->as<StateType>();
        const auto *tot = to->as<StateType>();
        auto *statet = state->as<StateType>();

        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double diff = tot->values[i] - fromt->values[i];
            if (fabs(diff) <= boost::math::constants::pi<double>())
                statet->values[i] = fromt->values[i] + diff * t;
            else
            {
                if (diff > 0.0)
                    diff = 2.0 * boost::math::constants::pi<double>() - diff;
                else
                    diff = -2.0 * boost::math::constants::pi<double>() - diff;

                statet->values[i] = fromt->values[i] - diff * t;
                if (statet->values[i] > boost::math::constants::pi<double>())
                    statet->values[i] -= 2.0 * boost::math::constants::pi<double>();
                else if (statet->values[i] < -boost::math::constants::pi<double>())
                    statet->values[i] += 2.0 * boost::math::constants::pi<double>();
            }
        }
    }

    double linkLength() const
    {
        return linkLength_;
    }

    const Environment *environment() const
    {
        return environment_;
    }

    // Allow external code to set the environment after construction
    void setEnvironment(Environment *env)
    {
        environment_ = env;
    }

protected:
    double linkLength_;
    Environment *environment_;
};

class KinematicChainValidityChecker : public ompl::base::StateValidityChecker
{
public:
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
        const auto *compound = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();

        // Extract components
        const auto *compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto *pos = compoundState->components[0]->as<ompl::base::RealVectorStateSpace::StateType>();
        const auto *orient = compoundState->components[1]->as<ompl::base::RealVectorStateSpace::StateType>();
        const auto *chainState = compoundState->components[2]->as<KinematicChainSpace::StateType>();
        const auto *chainSpace = compound->getSubspace(2)->as<KinematicChainSpace>();

        // Base position & orientation
        double base_x = pos->values[0];
        double base_y = pos->values[1];
        double base_theta = orient->values[0];

        return isValidImpl(chainSpace, chainState, base_x, base_y, base_theta);
    }

    /**
     * Compute the clearance of the state from obstacles.
     * This function approximates the clearance by calculating the distance
     * from the box center to the obstacle corners.
     */
    double clearance(const ompl::base::State *state) const override
    {
        // Extract state components
        const auto *compound = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();
        const auto *compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto *pos = compoundState->components[0]->as<ompl::base::RealVectorStateSpace::StateType>();

        // Box center
        double bx = pos->values[0];
        double by = pos->values[1];

        // Invalid states get 0 clearance
        if (!isValid(state))
            return 0.0;

        // Access environment
        const auto *chainSpace = compound->getSubspace(2)->as<KinematicChainSpace>();
        const Environment *env = chainSpace->environment();
        if (!env || env->empty())
            return 1e6;

        // Compute the minimum distance from the box center to all obstacle segment endpoints
        double minDist = std::numeric_limits<double>::infinity();
        for (const auto &seg : *env)
        {
            // distance from center (bx,by) to both segment endpoints
            double dx1 = bx - seg.x0, dy1 = by - seg.y0;
            double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            if (d1 < minDist) minDist = d1;

            double dx2 = bx - seg.x1, dy2 = by - seg.y1;
            double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (d2 < minDist) minDist = d2;
        }

        return minDist;
    }




protected:
    // bool isValidImpl(const KinematicChainSpace *space, const KinematicChainSpace::StateType *s, double base_x, double base_y, double base_theta) const
    // {
    //     unsigned int n = space->getDimension();
    //     Environment segments;
    //     double linkLength = space->linkLength();

    //     double theta = base_theta;
    //     double x = base_x;
    //     double y = base_y;
    //     double xN, yN;

    //     segments.reserve(n + 1);
    //     for (unsigned int i = 0; i < n; ++i)
    //     {
    //         theta += s->values[i];
    //         xN = x + cos(theta) * linkLength;
    //         yN = y + sin(theta) * linkLength;
    //         segments.emplace_back(x, y, xN, yN);
    //         x = xN;
    //         y = yN;
    //     }
    //     segments.emplace_back(x, y, x + cos(theta) * 0.001, y + sin(theta) * 0.001);

    //     return selfIntersectionTest(segments) &&
    //         environmentIntersectionTest(segments, *space->environment()) &&
    //         fabs(base_x) <= 5.0 && fabs(base_y) <= 5.0;
    // }


    bool isValidImpl(const KinematicChainSpace *space,
                 const KinematicChainSpace::StateType *s,
                 double base_x, double base_y, double base_theta) const
    {
        const Environment *env = space->environment();
        if (!env) return false;

        const unsigned int n = space->getDimension();
        const double L = space->linkLength();

        // 1) Build chain links as segments (center -> link1 -> ... -> linkN)
        Environment links;
        links.reserve(n);
        double theta = base_theta, x = base_x, y = base_y;
        for (unsigned int i = 0; i < n; ++i)
        {
            theta += s->values[i];
            double xN = x + std::cos(theta) * L;
            double yN = y + std::sin(theta) * L;
            links.emplace_back(x, y, xN, yN);
            x = xN; y = yN;
        }

        // 2) Make rotated box edges
        Environment boxEdges;
        makeBoxEdges(base_x, base_y, base_theta, boxEdges);

        // 3) Bounds for base center
        const bool withinBounds = (std::fabs(base_x) <= 5.0 && std::fabs(base_y) <= 5.0);
        if (!withinBounds) return false;

        // 4) Chain vs environment
        if (!environmentIntersectionTest(links, *env)) return false;

        // 5) Self-collision of chain (non-adjacent only)
        if (!selfIntersectionNonAdjacent(links)) return false;

        // 6) Box vs environment
        if (!environmentIntersectionTest(boxEdges, *env)) return false;

        // 7) Chain (except first link) vs box
        if (chainIntersectsBox(links, boxEdges)) return false;

        return true;
    }


    // return true iff env does *not* include a pair of intersecting segments
    bool selfIntersectionTest(const Environment &env) const
    {
        for (unsigned int i = 0; i < env.size(); ++i)
            for (unsigned int j = i + 1; j < env.size(); ++j)
                if (intersectionTest(env[i], env[j]))
                    return false;
        return true;
    }
    // return true iff no segment in env0 intersects any segment in env1
    bool environmentIntersectionTest(const Environment &env0, const Environment &env1) const
    {
        for (const auto &i : env0)
            for (const auto &j : env1)
                if (intersectionTest(i, j))
                    return false;
        return true;
    }
    // return true iff segment s0 intersects segment s1
    bool intersectionTest(const Segment &s0, const Segment &s1) const
    {
        // adopted from:
        // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/1201356#1201356
        double s10_x = s0.x1 - s0.x0;
        double s10_y = s0.y1 - s0.y0;
        double s32_x = s1.x1 - s1.x0;
        double s32_y = s1.y1 - s1.y0;
        double denom = s10_x * s32_y - s32_x * s10_y;
        if (fabs(denom) < std::numeric_limits<double>::epsilon())
            return false;  // Collinear
        bool denomPositive = denom > 0;

        double s02_x = s0.x0 - s1.x0;
        double s02_y = s0.y0 - s1.y0;
        double s_numer = s10_x * s02_y - s10_y * s02_x;
        if ((s_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
            return false;  // No collision
        double t_numer = s32_x * s02_y - s32_y * s02_x;
        if ((t_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
            return false;  // No collision
        if (((s_numer - denom > -std::numeric_limits<float>::epsilon()) == denomPositive) ||
            ((t_numer - denom > std::numeric_limits<float>::epsilon()) == denomPositive))
            return false;  // No collision
        return true;
    }

    // Build rotated 1x1 box edges centered at (bx,by) with orientation theta
    void makeBoxEdges(double bx, double by, double theta, Environment &boxEdges) const
    {
        const double h = 0.5;
        double cx[4], cy[4];

        // corners in local box frame: ( +h,+h ), ( +h,-h ), ( -h,-h ), ( -h,+h )
        double lx[4] = { h,  h, -h, -h};
        double ly[4] = { h, -h, -h,  h};

        for (int i = 0; i < 4; ++i)
        {
            cx[i] = bx + lx[i]*cos(theta) - ly[i]*sin(theta);
            cy[i] = by + lx[i]*sin(theta) + ly[i]*cos(theta);
        }
        boxEdges.clear();
        boxEdges.emplace_back(cx[0], cy[0], cx[1], cy[1]);
        boxEdges.emplace_back(cx[1], cy[1], cx[2], cy[2]);
        boxEdges.emplace_back(cx[2], cy[2], cx[3], cy[3]);
        boxEdges.emplace_back(cx[3], cy[3], cx[0], cy[0]);
    }

    // self-intersection: only check NON-adjacent links
    bool selfIntersectionNonAdjacent(const Environment &links) const
    {
        const std::size_t m = links.size();
        for (std::size_t i = 0; i < m; ++i)
            for (std::size_t j = i + 1; j < m; ++j)
            {
                if (j == i + 1) continue;     // adjacent links share a joint; allowed
                if (intersectionTest(links[i], links[j]))
                    return false;
            }
        return true;
    }

    // chain (except the first link) must NOT intersect the box edges
    bool chainIntersectsBox(const Environment &links, const Environment &boxEdges) const
    {
        for (std::size_t i = 0; i < links.size(); ++i)
        {
            if (i == 0) continue; // first link is allowed to overlap the box
            for (const auto &e : boxEdges)
                if (intersectionTest(links[i], e))
                    return true;
        }
        return false;
    }

};

Environment createHornEnvironment(unsigned int d, double eps)
{
    std::ofstream envFile(boost::str(boost::format("environment_%i.dat") % d));
    std::vector<Segment> env;
    double w = 1. / (double)d, x = w, y = -eps, xN, yN, theta = 0.,
           scale = w * (1. + boost::math::constants::pi<double>() * eps);

    envFile << x << " " << y << std::endl;
    for (unsigned int i = 0; i < d - 1; ++i)
    {
        theta += boost::math::constants::pi<double>() / (double)d;
        xN = x + cos(theta) * scale;
        yN = y + sin(theta) * scale;
        env.emplace_back(x, y, xN, yN);
        x = xN;
        y = yN;
        envFile << x << " " << y << std::endl;
    }

    theta = 0.;
    x = w;
    y = eps;
    envFile << x << " " << y << std::endl;
    scale = w * (1.0 - boost::math::constants::pi<double>() * eps);
    for (unsigned int i = 0; i < d - 1; ++i)
    {
        theta += boost::math::constants::pi<double>() / d;
        xN = x + cos(theta) * scale;
        yN = y + sin(theta) * scale;
        env.emplace_back(x, y, xN, yN);
        x = xN;
        y = yN;
        envFile << x << " " << y << std::endl;
    }
    envFile.close();
    return env;
}

Environment createEmptyEnvironment(unsigned int d)
{
    std::ofstream envFile(boost::str(boost::format("environment_%i.dat") % d));
    std::vector<Segment> env;
    envFile.close();
    return env;
}

#endif
