/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/rrt/RRTtask.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/multilevel/datastructures/TaskSpaceMotionValidator.h"

const bool debug = false;

ompl::geometric::RRTtask::RRTtask(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, "RRTtask")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRTtask::setRange, &RRTtask::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTtask::setGoalBias, &RRTtask::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRTtask::setIntermediateStates, &RRTtask::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;
    if(dynamic_pointer_cast<ompl::multilevel::TaskSpaceMotionValidator>(si_->getMotionValidator()) != nullptr) {
      OMPL_ERROR("Using Task Space Capabilities for Planner %s", getName().c_str());
        use_task_space_ = true;
    }else {
      OMPL_ERROR("Not Using Task Space Capabilities for Planner %s", getName().c_str());
    }
    lastValid.first = si_->allocState();
    xstate = si_->allocState();
    rmotion = new Motion(si_);
    rstate = rmotion->state;
}

ompl::geometric::RRTtask::~RRTtask()
{
    freeMemory();
    si_->freeState(lastValid.first);
    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

}

void ompl::geometric::RRTtask::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
    first_run_ = true;
}

void ompl::geometric::RRTtask::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    first_run_ = true;
}

void ompl::geometric::RRTtask::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RRTtask::solve(const base::PlannerTerminationCondition &ptc)
{
    if(first_run_) {
      first_run_ = false;
      checkValidity();
      while (const base::State *st = pis_.nextStart())
      {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, st);
          nn_->add(motion);
      }
      if (nn_->size() == 0)
      {
          OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
          return base::PlannerStatus::INVALID_START;
      }
    }

    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (!sampler_) {
        sampler_ = si_->allocStateSampler();
    }

    OMPL_DEBUG("%s: Start planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    if ((goal_s == nullptr) || !goal_s->canSample()) {
      OMPL_ERROR("Goal is not sampleable.");
    }

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            if(debug) {
              OMPL_WARN("Sampling goal");
            }
            goal_s->sampleGoal(rstate);
        }
        else
        {
            if(debug) {
              OMPL_DEBUG("Sample state");
            }
            sampler_->sampleUniform(rstate);
        }

        if(debug) {
          si_->printState(rstate);
        }
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        if(debug) {
        OMPL_DEBUG("Nearest state:");
        si_->printState(nmotion->state);
        }
        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d >= std::numeric_limits<double>::infinity()) {
          continue;
        }
        if(debug) {
          OMPL_DEBUG("Distance nearest to sample: %f", d);
        }

        if(use_task_space_) {

          auto motion_validator = static_pointer_cast<ompl::multilevel::TaskSpaceMotionValidator>(si_->getMotionValidator());
          auto states = motion_validator->propagateMotion(nmotion->state, dstate);
          if(states.size() <= 1) {
            continue;
          }

          for (std::size_t i = 1; i < states.size(); ++i)
          {
              auto *motion = new Motion;
              motion->state = states[i]; motion->parent = nmotion;
              nn_->add(motion);

              nmotion = motion;
          }
          if(debug) {
            OMPL_DEBUG("Reached new state:");
            si_->printState(nmotion->state);
          }
        } else {
          //Default checkMotion
          if (d > maxDistance_)
          {
              si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
              dstate = xstate;
          }
          // auto reached = si_->checkMotion(nmotion->state, dstate, lastValid);
          // if (reached || lastValid.second > 0.0) //si_->checkMotion(nmotion->state, dstate, lastValid))
          // {
          //     if(!reached) {
          //       dstate = lastValid.first;
          //     }
          //     auto *motion = new Motion(si_);
          //     si_->copyState(motion->state, dstate);
          //     motion->parent = nmotion;
          //     nn_->add(motion);

          //     nmotion = motion;
          // }
          if(debug) {
            OMPL_DEBUG("New state:");
            si_->printState(dstate);
          }
          if (si_->checkMotion(nmotion->state, dstate))
          {
              if(debug) {
                OMPL_DEBUG("Added valid connection:");
                si_->printState(nmotion->state);
                si_->printState(dstate);
              }
              // if(!reached) {
              //   dstate = lastValid.first;
              // }
              auto *motion = new Motion(si_);
              si_->copyState(motion->state, dstate);
              motion->parent = nmotion;
              nn_->add(motion);

              nmotion = motion;
          } else {
            continue;
          }
        }
        double dist = 0.0;
        bool sat = goal->isSatisfied(nmotion->state, &dist);
        if (sat)
        {
            approxdif = dist;
            solution = nmotion;
            break;
        }
        if (dist < approxdif)
        {
            approxdif = dist;
            approxsol = nmotion;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    OMPL_DEBUG("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::RRTtask::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
