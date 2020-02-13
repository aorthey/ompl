/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
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

/* Author: Andreas Orthey, Sohaib Akbar */

#include <ompl/geometric/planners/quotientspace/algorithms/QMPStarImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#define foreach BOOST_FOREACH

ompl::geometric::QMPStarImpl::QMPStarImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("QMPStarImpl" + std::to_string(id_));
    Planner::declareParam<double>("range", this, &QMPStarImpl::setRange, &QMPStarImpl::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &QMPStarImpl::setGoalBias, &QMPStarImpl::getGoalBias, "0.:.1:1.");
    qRandom_ = new Configuration(Bundle);

    double d = (double)Bundle->getStateDimension();
    double e = boost::math::constants::e<double>();
    kPRMStarConstant_ = e + (e / d);
    
    randomWorkStates_.resize(5);
    Bundle->allocStates(randomWorkStates_);
}

ompl::geometric::QMPStarImpl::~QMPStarImpl()
{
    si_->freeStates(randomWorkStates_);
    deleteConfiguration(qRandom_);
}

void ompl::geometric::QMPStarImpl::setGoalBias(double goalBias)
{
    goalBias_ = goalBias;
}

double ompl::geometric::QMPStarImpl::getGoalBias() const
{
    return goalBias_;
}

void ompl::geometric::QMPStarImpl::setRange(double maxDistance)
{
    maxDistance_ = maxDistance;
}

double ompl::geometric::QMPStarImpl::getRange() const
{
    return maxDistance_;
}

void ompl::geometric::QMPStarImpl::setup()
{
    BaseT::setup();
    ompl::tools::SelfConfig sc(Bundle, getName());
    sc.configurePlannerRange(maxDistance_);
}

void ompl::geometric::QMPStarImpl::clear()
{
    BaseT::clear();
}

bool ompl::geometric::QMPStarImpl::getSolution(base::PathPtr &solution)
{
    if (hasSolution_)
    {
        bool baset_sol = BaseT::getSolution(solution);
        if (baset_sol)
        {
            shortestPathVertices_ = shortestVertexPath_;
        }
        return baset_sol;
    }
    else
    {
        return false;
    }
}

void ompl::geometric::QMPStarImpl::grow()
{
    if (firstRun_)
    {
        init();
        // add goal too
        vGoal_ = addConfiguration(qGoal_);

        firstRun_ = false;
    }

    if( ++growExpandCounter_ % 5 == 0)
    {
        expand();
        return;
    }

    if (hasSolution_)
    {
        // No Goal Biasing if we already found a solution on this bundle space
        sampleBundle(qRandom_->state);
    }
    else
    {
        double s = rng_.uniform01();
        if (s < goalBias_)
        {
            Bundle->copyState(qRandom_->state, qGoal_->state);
        }
        else
        {
            sampleBundle(qRandom_->state);
        }
    }
    addMileStone(qRandom_->state);
}

void ompl::geometric::QMPStarImpl::expand()
{
    PDF pdf;

    foreach (Vertex v, boost::vertices(graph_))
    {
        const unsigned long int t = graph_[v]->total_connection_attempts;
        pdf.add(graph_[v], (double)(t - graph_[v]->successful_connection_attempts) / (double)t);
    }

    if (pdf.empty())
        return;

    
    Configuration *q = pdf.sample(rng_.uniform01());

    int s = si_->randomBounceMotion(Bundle_sampler_, q->state, randomWorkStates_.size(), randomWorkStates_, false);
    if(s > 0)
    {
        Configuration *prev = q;
        Configuration *last = addMileStone(randomWorkStates_[--s]);
        for (int i = 0; i < s; i++)
        {
            Configuration *tmp = new Configuration(Bundle, randomWorkStates_[i]);
            addConfiguration(tmp);

            ompl::geometric::BundleSpaceGraph::addEdge(prev->index, tmp->index);
            prev = tmp;
        }
        if(!sameComponent(prev->index, last->index))
            ompl::geometric::BundleSpaceGraph::addEdge(prev->index, last->index);
    }
}

ompl::geometric::BundleSpaceGraph::Configuration *ompl::geometric::QMPStarImpl::addMileStone(ompl::base::State *q_state)
{
    // add sample to graph
    Configuration *q_next = new Configuration(Bundle, q_state);
    Vertex v_next = addConfiguration(q_next);

    // totalNumberOfSamples_++;
    // totalNumberOfFeasibleSamples_++;

    // Calculate K
    unsigned int k = static_cast<unsigned int>(ceil(kPRMStarConstant_ * log((double) boost::num_vertices(graph_))));

    // check for close neighbors
    std::vector<Configuration*> r_nearest_neighbors;
    BaseT::nearestDatastructure_->nearestK(q_next , k , r_nearest_neighbors);
    
    for(unsigned int i=0 ; i< r_nearest_neighbors.size(); i++)
    {
        Configuration* q_neighbor = r_nearest_neighbors.at(i);
        
        q_next->total_connection_attempts++;
        q_neighbor->total_connection_attempts++;
        
        if (Bundle->checkMotion(q_neighbor->state, q_next->state)) 
        {
            addEdge(q_neighbor->index, v_next);
            q_next->successful_connection_attempts++;
            q_neighbor->successful_connection_attempts++;

            if (/*q_neighbor->isGoal && */!hasSolution_)
            {
                bool same_component = sameComponent(vStart_, vGoal_);
                if (same_component)
                {
                    hasSolution_ = true;
                }
            }
        }

    }
    return q_next;
}

double ompl::geometric::QMPStarImpl::getImportance() const
{
    // Should depend on
    // (1) level : The higher the level, the more importance
    // (2) total samples: the more we already sampled, the less important it
    // becomes
    // (3) has solution: if it already has a solution, we should explore less
    // (only when nothing happens on other levels)
    // (4) vertices: the more vertices we have, the less important (let other
    // levels also explore)
    //
    // exponentially more samples on level i. Should depend on ALL levels.
    // const double base = 2;
    // const double normalizer = powf(base, level);
    // double N = (double)GetNumberOfVertices()/normalizer;
    double N = (double)getNumberOfVertices();
    return 1.0 / (N + 1);
}

// Make it faster by removing the validity check
bool ompl::geometric::QMPStarImpl::sampleBundle(base::State *q_random)
{
    if (parent_ == nullptr)
    {
        Bundle_sampler_->sampleUniform(q_random);
    }
    else
    {
        if (getFiberDimension() > 0)
        {
            Fiber_sampler_->sampleUniform(xFiberTmp_);
            parent_->sampleFromDatastructure(xBaseTmp_);
            mergeStates(xBaseTmp_, xFiberTmp_, q_random);
        }
        else
        {
            parent_->sampleFromDatastructure(q_random);
        }
    }
    return true;
}

bool ompl::geometric::QMPStarImpl::sampleFromDatastructure(base::State *q_random_graph)
{
    // RANDOM VERTEX SAMPLING
    const Vertex v = boost::random_vertex(graph_, rng_boost);
    Bundle->getStateSpace()->copyState(q_random_graph, graph_[v]->state);
    return true;
}
