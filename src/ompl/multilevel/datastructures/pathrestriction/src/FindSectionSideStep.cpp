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
#include <ompl/multilevel/datastructures/pathrestriction/PathRestrictionInterpolator.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

#include <optional>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 3;
        static const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 10;
    }
}

using namespace ompl::multilevel;

FindSectionSideStep::FindSectionSideStep(const PathRestrictionPtr& restriction) : BaseT(restriction)
{
}

FindSectionSideStep::~FindSectionSideStep()
{
}

std::optional<PathSectionPtr> FindSectionSideStep::solve(const ompl::base::State* xStart, const ompl::base::State* xGoal)
{
    if (restriction_ == nullptr) {
        return std::nullopt;
    }

    HeadPtr head = std::make_shared<Head>(restriction_, xStart, xGoal);

    const ompl::base::State *q = head->getState();

    auto maybe_fiber_first_section = recursiveSideStep(head, true);
    if (maybe_fiber_first_section.has_value()) {
    std::stringstream buffer;
    buffer << *head;
    OMPL_DEVMSG1("Last head before termination: %s.", buffer.str().c_str());
      return maybe_fiber_first_section;
    }

    head->setCurrent(q, 0);
    auto maybe_fiber_last_section = recursiveSideStep(head, false);
    if (maybe_fiber_last_section.has_value()) {
    std::stringstream buffer;
    buffer << *head;
    OMPL_DEVMSG1("Last head before termination: %s.", buffer.str().c_str());
      return maybe_fiber_last_section;
    }
    std::stringstream buffer;
    buffer << *head;
    OMPL_DEVMSG1("Last head before termination: %s.", buffer.str().c_str());
    return std::nullopt;
}

PathSectionPtr MakeInterpolatedSection(const PathRestrictionPtr& restriction, const HeadPtr& head, bool fiber_first) {
    if (fiber_first)
    {
        return interpolateL1FiberFirst(restriction, head);
    }
    else
    {
        return interpolateL1FiberLast(restriction, head);
    }
}

std::optional<PathSectionPtr> FindSectionSideStep::recursiveSideStep(HeadPtr &head, bool interpolateFiberFirst, unsigned int depth)
{
    auto projection = restriction_->getProjection();
    auto bundle = projection->getBundle();
    auto base = projection->getBase();

    auto section = MakeInterpolatedSection(restriction_, head, interpolateFiberFirst);

    if (section->checkMotion(head))
    {
        OMPL_DEVMSG1("Found section on depth %d", depth);
        return section;
    }

    // static_cast<BundleSpaceGraph *>(projection->getChild())
    //     ->getGraphSampler()
    //     ->setPathBiasStartSegment(head->getLocationOnBasePath());

    //############################################################################
    // Get last valid state information
    //############################################################################

    if (depth + 1 >= magic::PATH_SECTION_TREE_MAX_DEPTH)
    {
        return std::nullopt;
    }

    double location = head->getLocationOnBasePath();

    base::State *xBase = base->allocState();

    restriction_->interpolateBasePath(location, xBase);

    for (unsigned int j = 0; j < magic::PATH_SECTION_TREE_MAX_BRANCHING; j++)
    {
        if (!findFeasibleStateOnFiber(xBase, xBundleTmp_))
        {
            continue;
        }

        if (restriction_->getSpaceInformation()->checkMotion(head->getState(), xBundleTmp_))
        {
            auto xSideStep = bundle->allocState();
            bundle->copyState(xSideStep, xBundleTmp_);

            OMPL_ERROR("NEED TO ADD CONFIG TO SECTION PATH");
            throw "NYI";
            // Configuration *xSideStep = new Configuration(bundle, xBundleTmp_);
            // graph->addConfiguration(xSideStep);
            // graph->addBundleEdge(head->getConfiguration(), xSideStep);

            HeadPtr newHead(head);

            newHead->setCurrent(xSideStep, location);

            auto maybe_feasible_section = recursiveSideStep(newHead, !interpolateFiberFirst, depth + 1);

            if (maybe_feasible_section.has_value())
            {
                head = newHead;
                base->freeState(xBase);
                return maybe_feasible_section.value();
            }
        }
    }
    base->freeState(xBase);
    return std::nullopt;
}
