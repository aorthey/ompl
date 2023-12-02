/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023 
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

/* Author: Andreas Orthey */

#ifndef OMPL_BASE_GOALS_FACTORED_GOAL_
#define OMPL_BASE_GOALS_FACTORED_GOAL_

#include <optional>

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/multilevel/datastructures/FactoredSpaceInformation.h"
#include "ompl/base/ScopedState.h"

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(GoalSampleableRegion);

        /** \brief Definition of a factored goal */
        class FactoredGoal : public GoalSampleableRegion
        {
        public:
            FactoredGoal(const multilevel::FactoredSpaceInformationPtr& si, const std::unordered_map<std::string, GoalSampleableRegionPtr>& goals);

            ~FactoredGoal() override;

            void sampleGoal(State *state) const override;
            unsigned int maxSampleCount() const override;
            double distanceGoal(const State *state) const override;

            std::optional<GoalSampleableRegionPtr> getFactorGoal(const std::string name) const;

        private:
            std::unordered_map<std::string, GoalSampleableRegionPtr> goals_;
            std::unordered_map<std::string, ompl::base::State*> tmp_goal_states_;
        };
    }
}

#endif

