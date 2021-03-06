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

#include <ompl/multilevel/datastructures/graphsampler/VisibilityRegion.h>

using namespace ompl::multilevel;
using namespace ompl::base;

BundleSpaceGraphSamplerVisibilityRegion::BundleSpaceGraphSamplerVisibilityRegion(BundleSpaceGraph *bundleSpaceGraph)
  : BaseT(bundleSpaceGraph)
{
    epsilonGraphThickening_ = 0;
    bundleSpaceGraphSparse_ = dynamic_cast<BundleSpaceGraphSparse *>(bundleSpaceGraph);
    if (bundleSpaceGraphSparse_ == nullptr)
    {
        OMPL_ERROR("Visibility Region Sampler only valid with sparse graph.");
        throw ompl::Exception("Invalid Sampler");
    }
    regionBias_.setValueInit(0.0);
    regionBias_.setValueTarget(1.0);
    regionBias_.setCounterInit(0.0);
    regionBias_.setCounterTarget(1000);
}

void BundleSpaceGraphSamplerVisibilityRegion::clear()
{
    regionBias_.reset();
}

void BundleSpaceGraphSamplerVisibilityRegion::sampleImplementation(State *xRandom)
{
    BaseT::sampleImplementation(xRandom);

    double bias = regionBias_();

    // const double visibilityRadius = bias*bundleSpaceGraphSparse_->getSparseDelta();

    // bundleSpaceGraphSparse_->getBundleSamplerPtr()
    //   ->sampleUniformNear(xRandom, xRandom, visibilityRadius);

    double s = rng_.uniform01();
    if (s < bias)
    {
        const double visibilityRadius = bias * bundleSpaceGraphSparse_->getSparseDelta();

        bundleSpaceGraphSparse_->getBundleSamplerPtr()->sampleUniformNear(xRandom, xRandom, visibilityRadius);
    }
}
