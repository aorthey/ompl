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

#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

ompl::multilevel::BundleSpaceGraphSampler::BundleSpaceGraphSampler(BundleSpaceGraph *bundleSpaceGraph)
  : bundleSpaceGraph_(bundleSpaceGraph)
{
    double mu = bundleSpaceGraph_->getBundle()->getMaximumExtent();
    epsilonGraphThickening_ = mu * epsilonGraphThickeningFraction_;
    OMPL_DEBUG("Epsilon Graph Thickening constant set to %f", epsilonGraphThickening_);

    pathBiasDecay_.setLambda(exponentialDecayLambda_);
    pathBiasDecay_.setLowerBound(pathBiasFixed_);

    pathThickeningGrowth_.setLambda(exponentialDecayLambda_);
    pathThickeningGrowth_.setLowerBound(epsilonGraphThickening_);
    pathThickeningGrowth_.setUpperBound(0.0);

    graphThickeningGrowth_.setLambda(exponentialDecayLambda_);
    graphThickeningGrowth_.setLowerBound(epsilonGraphThickening_);
    graphThickeningGrowth_.setUpperBound(0.0);
}

void ompl::multilevel::BundleSpaceGraphSampler::reset()
{
    double mu = bundleSpaceGraph_->getBundle()->getMaximumExtent();
    epsilonGraphThickening_ = mu * epsilonGraphThickeningFraction_;

    pathBiasDecay_.reset();
    pathThickeningGrowth_.reset();
    graphThickeningGrowth_.reset();
}

void ompl::multilevel::BundleSpaceGraphSampler::disableSegmentBias()
{
    this->segmentBias_ = false;
}

void ompl::multilevel::BundleSpaceGraphSampler::disablePathBias()
{
    pathBiasDecay_.setLambda(0);
    pathBiasDecay_.setUpperBound(0);
    pathBiasDecay_.setLowerBound(0);
}

void ompl::multilevel::BundleSpaceGraphSampler::setPathBiasStartSegment(double s)
{
    if (!segmentBias_)
    {
        this->pathBiasStartSegment_ = 0;
    }
    else
    {
        if (s > this->pathBiasStartSegment_)
        {
            this->pathBiasStartSegment_ = s;
            geometric::PathGeometric &spath =
                static_cast<geometric::PathGeometric &>(*bundleSpaceGraph_->solutionPath_);
            OMPL_DEBUG("Set path bias: %f/%f", s, spath.length());
        }
    }
}

double ompl::multilevel::BundleSpaceGraphSampler::getPathBiasStartSegment()
{
    return this->pathBiasStartSegment_;
}

void ompl::multilevel::BundleSpaceGraphSampler::sample(base::State *xRandom)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    // EXP DECAY PATH BIAS.
    // from 1.0 down to lower limit pathbiasfixed_
    // const double pathBias =
    //   (1.0 - pathBiasFixed_) * exp(-exponentialDecayLambda_ * counterPathSampling_++)
    //   + pathBiasFixed_;

    double p = rng_.uniform01();
    double pd = pathBiasDecay_();

    if (p < pd && !bundleSpaceGraph_->isDynamic())
    {
        geometric::PathGeometric &spath = static_cast<geometric::PathGeometric &>(*bundleSpaceGraph_->solutionPath_);
        std::vector<base::State *> states = spath.getStates();

        if (states.size() < 2)
        {
            // empty solution returned, cannot use path bias sampling
            sampleImplementation(xRandom);
        }
        else
        {
            // First one works well for SE3->R3, second one works best for
            // SE3^k->SE3^{k-1}
            // double endLength = std::min( pathBiasStartSegment_ + 0.1*spath.length(),
            //     spath.length());

            double endLength = spath.length();
            double distStopping = pathBiasStartSegment_ + rng_.uniform01() * (endLength - pathBiasStartSegment_);

            // std::cout << "pathbias start:" << pathBiasStartSegment_ << std::endl;

            // std::cout << "pathSampling:" << distStopping << "/" << totalLength
            //   << " biasSegment:" << pathBiasStartSegment_ << "." << std::endl;

            base::State *s1 = nullptr;
            base::State *s2 = nullptr;

            int ctr = 0;
            double distLastSegment = 0;
            double distCountedSegments = 0;
            while (distCountedSegments < distStopping && (ctr < (int)states.size() - 1))
            {
                s1 = states.at(ctr);
                s2 = states.at(ctr + 1);
                distLastSegment = bundle->distance(s1, s2);
                distCountedSegments += distLastSegment;
                ctr++;
            }

            //          |---- d -----|
            //---O------O------------O
            //|--------- t ----------|
            //|--------- s ------|
            //          |d-(t-s) |
            double step = (distLastSegment - (distCountedSegments - distStopping)) / (distLastSegment);
            bundle->getStateSpace()->interpolate(s1, s2, step, xRandom);

            if (epsilonGraphThickening_ > 0)
            {
                double eps = pathThickeningGrowth_();
                bundleSpaceGraph_->getBundleSamplerPtr()->sampleUniformNear(xRandom, xRandom, eps);
            }
        }
    }
    else
    {
        sampleImplementation(xRandom);
    }

    // Perturbate sample in epsilon neighborhood
    //  Note: Alternatively, we can use sampleGaussian (but seems to give similar
    //  results)
    if (epsilonGraphThickening_ > 0)
    {
        // Decay on graph thickening (reflects or believe in the usefullness of
        // the graph for biasing our sampling)
        double eps = graphThickeningGrowth_();
        // std::cout << "Epsilon: " << eps << std::endl;
        bundleSpaceGraph_->getBundleSamplerPtr()->sampleUniformNear(xRandom, xRandom, eps);
    }
}