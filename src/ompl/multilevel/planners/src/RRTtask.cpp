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

#include "ompl/multilevel/planners/RRTtask.h"
#include <limits>
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/multilevel/datastructures/TaskSpaceMotionValidator.h"

const bool kDebug = false;
using ompl::multilevel::TreeNode;

ompl::multilevel::RRTtask::RRTtask(const base::SpaceInformationPtr &si)
  : base::Planner(si, "RRTtask")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRTtask::setRange, &RRTtask::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTtask::setGoalBias, &RRTtask::getGoalBias, "0.:.05:1.");

    if(std::dynamic_pointer_cast<ompl::multilevel::TaskSpaceMotionValidator>(si_->getMotionValidator()) != nullptr) {
      OMPL_INFORM("Using Task Space Capabilities for Planner %s", getName().c_str());
        use_task_space_ = true;
    }else {
      OMPL_INFORM("Not Using Task Space Capabilities for Planner %s", getName().c_str());
    }
    random_node = new TreeNode(si_);
}

ompl::multilevel::RRTtask::~RRTtask()
{
    if (random_node->getState() != nullptr)
        si_->freeState(random_node->getState());
    delete random_node;

}

void ompl::multilevel::RRTtask::clear()
{
    Planner::clear();
    sampler_.reset();
    tree_->clear();
    if(tree_->size() != 0) {
      throw ompl::Exception("Could not clear tree");
    }
    lastGoalMotion_ = nullptr;
    first_run_ = true;
}

void ompl::multilevel::RRTtask::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxRange_);

    tree_ = std::make_shared<Tree>(si_);
    first_run_ = true;

    if (!pdef_)
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    } else {
      base::Goal *goal = pdef_->getGoal().get();
      auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
      if(goal_s ==nullptr) {
        OMPL_ERROR("Goal is not sampleable.");
        throw "RequiresSampleableGoal";
      }
    }
}

void ompl::multilevel::RRTtask::setGoalBias(double goalBias)
{
    goalBias_ = goalBias;
}

double ompl::multilevel::RRTtask::getGoalBias() const
{
    return goalBias_;
}

void ompl::multilevel::RRTtask::setRange(double range)
{
    maxRange_ = range;
}

double ompl::multilevel::RRTtask::getRange() const
{
    return maxRange_;
}

bool ompl::multilevel::RRTtask::shouldSampleGoal(const ompl::base::GoalSampleableRegion* goal, size_t iteration_counter) {
  if(goal == nullptr) {
    OMPL_WARN("Cannot sample goal.");
    return false;
  }
  if(!goal->canSample()) {
    OMPL_WARN("Cannot sample goal.");
    return false;
  }
  if(!(goalBias_ > 0.0)) {
    OMPL_WARN("No goal bias.");
    return false;
  }
  if(iteration_counter < 1) {
    return true;
  }
  if(rng_.uniform01() > goalBias_) {
    return false;
  }
  return true;
}

TreeNode* ompl::multilevel::RRTtask::makeRootNode() {
    while (const base::State *st = pis_.nextStart())
    {
        return tree_->addNodeAndParent(st, nullptr);
    }
    return nullptr;
}

ompl::base::PlannerStatus ompl::multilevel::RRTtask::solve(const base::PlannerTerminationCondition &ptc)
{
    if(first_run_) {
      first_run_ = false;
      checkValidity();
      makeRootNode();
    }

    if (tree_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = static_cast<base::GoalSampleableRegion *>(goal);

    if (!sampler_) {
        sampler_ = si_->allocStateSampler();
    }

    OMPL_DEBUG("%s: Start planning with %u states already in datastructure", getName().c_str(), tree_->size());

    TreeNode *solution = nullptr;
    TreeNode *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    if ((goal_s == nullptr) || !goal_s->canSample()) {
      OMPL_ERROR("Goal is not sampleable.");
    }

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if(shouldSampleGoal(goal_s, iteration_counter_))
        {
            if(kDebug) {
              OMPL_WARN("Sample goal");
            }
            goal_s->sampleGoal(random_node->getState());
        }
        else
        {
            if(kDebug) {
              OMPL_DEBUG("Sample state");
            }
            sampler_->sampleUniform(random_node->getState());
        }
        iteration_counter_++;

        if(kDebug) {
          si_->printState(random_node->getState());
        }
        /* find closest state in the tree */
        TreeNode *nearest_node = tree_->nearest(random_node);
        base::State *new_state = random_node->getState();

        if(kDebug) {
        OMPL_DEBUG("Nearest state:");
        si_->printState(nearest_node->getState());
        }
        /* find state to add */
        double d = si_->distance(nearest_node->getState(), random_node->getState());
        if (d >= std::numeric_limits<double>::infinity()) {
          continue;
        }
        if(kDebug) {
          OMPL_DEBUG("Distance nearest to sample: %f", d);
        }

        if(use_task_space_) {

          if(kDebug) {
            OMPL_DEBUG("Propagate motion");
          }
          auto motion_validator = std::static_pointer_cast<ompl::multilevel::TaskSpaceMotionValidator>(si_->getMotionValidator());
          auto states = motion_validator->propagateMotion(nearest_node->getState(), new_state);
          if(states.size() <= 1) {
            continue;
          }

          for (std::size_t i = 1; i < states.size(); ++i)
          {
              nearest_node = tree_->addNodeAndParent(states[i], nearest_node);
          }
          if(kDebug) {
            OMPL_DEBUG("Reached new state:");
            si_->printState(nearest_node->getState());
          }
        } else {
          //Default checkMotion
          if (d > maxRange_)
          {
              si_->getStateSpace()->interpolate(nearest_node->getState(), random_node->getState(), maxRange_ / d, new_state);
          }
          if(kDebug) {
            OMPL_DEBUG("New state:");
            si_->printState(new_state);
            OMPL_DEBUG("Check motion");
          }
          if (si_->checkMotion(nearest_node->getState(), new_state))
          {
              if(kDebug) {
                OMPL_DEBUG("Added valid connection:");
                si_->printState(nearest_node->getState());
                si_->printState(new_state);
              }
              nearest_node = tree_->addNodeAndParent(new_state, nearest_node);
          } else {
            if(kDebug) {
              OMPL_DEBUG("Invalid");
            }
            continue;
          }
        }
        double dist = 0.0;
        bool sat = goal->isSatisfied(nearest_node->getState(), &dist);
        if (sat)
        {
            approxdif = dist;
            solution = nearest_node;
            if(kDebug) {
              OMPL_DEBUG("Solution found and reached goal node");
              si_->printState(nearest_node->getState());
            }
            break;
        }
        if (dist < approxdif)
        {
            approxdif = dist;
            approxsol = nearest_node;
        }
    }

    //bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    } else {
        makeSolutionPath(solution, approximate, approxdif);
        solved_ = true;
    }

    OMPL_DEBUG("%s: Created %u states", getName().c_str(), tree_->size());

    return {solved_, approximate};
}

void ompl::multilevel::RRTtask::makeSolutionPath(TreeNode* last_node, bool approximate, double approxdif) {
    if (last_node == nullptr)
    {
      return;
    }

    lastGoalMotion_ = last_node;

    std::vector<TreeNode *> solution_path;
    auto current_node(last_node);
    while (current_node != nullptr)
    {
        solution_path.push_back(current_node);
        current_node = current_node->getParent();
    }

    auto path(std::make_shared<geometric::PathGeometric>(si_));

    for (int i = solution_path.size() - 1; i >= 0; --i) {
        auto state = solution_path.at(i)->getState();
        path->append(state);
    }
    pdef_->addSolutionPath(path, approximate, approxdif, getName());
}

void ompl::multilevel::RRTtask::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<TreeNode *> motions = tree_->getNodes();

    if (lastGoalMotion_ != nullptr)
    {
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->getState()));
    }

    for (auto &motion : motions)
    {
        if (motion->getParent() == nullptr)
        {
            data.addStartVertex(base::PlannerDataVertex(motion->getState()));
        } else {
            data.addEdge(base::PlannerDataVertex(motion->getParent()->getState()), base::PlannerDataVertex(motion->getState()));
        }
    }
}
