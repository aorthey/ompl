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

#ifndef OMPL_MULTILEVEL_PLANNERS_RRTTASK_
#define OMPL_MULTILEVEL_PLANNERS_RRTTASK_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/multilevel/datastructures/Tree.h"
#include "ompl/multilevel/datastructures/TreeNode.h"

namespace ompl
{
    namespace multilevel
    {
        class RRTtask : public base::Planner
        {
        public:
            RRTtask(const base::SpaceInformationPtr &si);

            ~RRTtask() override;
            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setGoalBias(double goalBias);
            double getGoalBias() const;
            void setRange(double range);
            double getRange() const;

            void setup() override;

        protected:
            void makeSolutionPath(TreeNode* last_node, bool approximate, double approxdif);

            TreeNode* makeRootNode();

            //void freeMemory();
            //double distance(const TreeNode *a, const TreeNode *b) const;
            bool shouldSampleGoal(const ompl::base::GoalSampleableRegion* goal, size_t iteration_counter);

            base::StateSamplerPtr sampler_;

            TreePtr tree_;

            double goalBias_{.05}; //was 0.05
            double maxRange_{0.};
            size_t iteration_counter_{0};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal node.  Used for PlannerData computation */
            TreeNode *lastGoalMotion_{nullptr};

            bool use_task_space_{false};

            bool first_run_{true};

            TreeNode *random_node{nullptr};
        };
    }
}

#endif
