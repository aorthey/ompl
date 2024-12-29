/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

#include <ompl/multilevel/datastructures/pathrestriction/FindSectionSideStep.h>

#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestrictionHelpers.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestrictionInterpolator.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>
#include <ompl/multilevel/datastructures/Tree.h>

#include <optional>

const bool kDebug = false;

namespace ompl
{
    namespace magic
    {
        // static const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 3;
        // static const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 10;
        static const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 5;
        static const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 2;
    }
}

using namespace ompl::multilevel;

FindSectionSideStep::FindSectionSideStep(const PathRestrictionPtr& restriction) : BaseT(restriction)
{
}

FindSectionSideStep::~FindSectionSideStep()
{
}

std::optional<PathSectionPtr> FindSectionSideStep::solve(const TreePtr& tree, const ompl::base::State* target)
{
    if (restriction_ == nullptr) {
        return std::nullopt;
    }

    const HeadPtr head = std::make_shared<Head>(restriction_, tree->getRoot(), target);

    auto maybe_fiber_last_section = recursiveSideStep(tree, head);
    if (maybe_fiber_last_section.has_value()) {
      return maybe_fiber_last_section;
    }

    std::stringstream buffer;
    buffer << *head;
    OMPL_DEVMSG1("Last head before termination: %s.", buffer.str().c_str());
    return std::nullopt;
}

std::optional<PathSectionPtr> FindSectionSideStep::recursiveSideStep(const TreePtr& tree, const HeadPtr& head, unsigned int depth)
{
    auto projection = restriction_->getProjection();
    auto bundle = projection->getBundle();
    auto base = projection->getBase();

    const double old_location_on_base_path = head->getLocationOnBasePath();
    if(kDebug) std::cout << ">>> Starting interpolate from " << old_location_on_base_path << std::endl;

    auto section = interpolateL1FiberLast(restriction_, head);
    auto resultAndNewHead = section->checkMotion(tree, head);
    if (resultAndNewHead.first)
    {
        OMPL_DEVMSG1("Found section on depth %d", depth);
        return section;
    }

    auto nextHead = resultAndNewHead.second;
    const double& new_location_on_base_path = nextHead->getLocationOnBasePath();

    if(kDebug) std::cout << "Head stopped at " << new_location_on_base_path << "/" << restriction_->getLengthBasePath() << std::endl;
    if(kDebug) bundle->printState(nextHead->getState());

    const double progress = std::abs(old_location_on_base_path - new_location_on_base_path);

    if(progress < 1e-4) {
        if(kDebug) std::cout << "TERMINATE: No Progress" << std::endl;
        return std::nullopt;
    }

    if (depth + 1 >= magic::PATH_SECTION_TREE_MAX_DEPTH)
    {
        if(kDebug) std::cout << "TERMINATE: Max depth" << std::endl;
        return std::nullopt;
    }

    for (unsigned int j = 0; j < magic::PATH_SECTION_TREE_MAX_BRANCHING; j++)
    {

        //Find a feasible fiber state to which we can sidestep 
        //(i.e. make a step exclusively on the fiber while keeping the base state constant)
        if(!findFeasibleStateOnFiber(restriction_, head, xBundleTmp_)) {
            continue;
        }

        if (restriction_->getSpaceInformation()->checkMotion(nextHead->getState(), xBundleTmp_))
        {
            auto xSideStep = tree->addNodeAndParent(xBundleTmp_, nextHead->getTreeNode());

            if(kDebug) std::cout << "New side step" << std::endl;
            if(kDebug) bundle->printState(xBundleTmp_);

            HeadPtr newHead(nextHead);
            newHead->setCurrent(xSideStep, new_location_on_base_path);

            auto maybe_feasible_section = recursiveSideStep(tree, newHead, depth + 1);

            if (maybe_feasible_section.has_value())
            {
                return maybe_feasible_section.value();
            }
        }
    }
    return std::nullopt;
}
