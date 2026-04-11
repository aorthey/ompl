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

#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSection.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>
#include <ompl/multilevel/datastructures/Tree.h>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_MAX_FIBER_SAMPLING = 10;
    }
}

using namespace ompl::multilevel;

FindSection::FindSection(const PathRestrictionPtr& restriction) : restriction_(restriction)
{
    if (!restriction_->getProjection()->isFibered())
    {
        OMPL_ERROR("Finding section with non-fibered projection.");
        throw ompl::Exception("FindSection is only valid with a fibered projection");
    }

    auto projection = std::static_pointer_cast<FiberedProjection>(restriction_->getProjection());
    if (projection->getCoDimension() > 0)
    {
        base::StateSpacePtr fiber = projection->getFiberSpace();
        if(fiber == nullptr) {
          OMPL_ERROR("Fiber space is nullptr.");
        }
  
        xFiberStart_ = fiber->allocState();
        xFiberGoal_ = fiber->allocState();
        xFiberTmp_ = fiber->allocState();
        validFiberSpaceSegmentLength_ = fiber->getLongestValidSegmentLength();
    }
    if (projection->getBaseDimension() > 0)
    {
        auto base = projection->getBase();
        xBaseTmp_ = base->allocState();
        validBaseSpaceSegmentLength_ = base->getLongestValidSegmentLength();
    }
    auto bundle = projection->getBundle();
    xBundleTmp_ = bundle->allocState();

    validBundleSpaceSegmentLength_ = bundle->getLongestValidSegmentLength();

    neighborhoodRadiusBaseSpaceLambda_ = 1e-4;

    neighborhoodRadiusBaseSpace_.setLambda(neighborhoodRadiusBaseSpaceLambda_);
    neighborhoodRadiusBaseSpace_.setValueInit(0.0);
    neighborhoodRadiusBaseSpace_.setValueTarget(10 * validBaseSpaceSegmentLength_);
}

FindSection::~FindSection()
{
    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(restriction_->getProjection());

    if (projection->getCoDimension() > 0)
    {
        base::StateSpacePtr fiber = projection->getFiberSpace();
        fiber->freeState(xFiberStart_);
        fiber->freeState(xFiberGoal_);
        fiber->freeState(xFiberTmp_);
    }
    if (projection->getBaseDimension() > 0)
    {
        auto base = projection->getBase();
        base->freeState(xBaseTmp_);
    }
    auto bundle = projection->getBundle();
    bundle->freeState(xBundleTmp_);
}
