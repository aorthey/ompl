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

#include <ompl/geometric/planners/quotientspace/algorithms/QMPImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <ompl/datastructures/NearestNeighbors.h>

#define foreach BOOST_FOREACH

ompl::geometric::QMPImpl::QMPImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("QMPImpl" + std::to_string(id_));
    // setMetric("euclidean");
    setMetric("shortestpath");
    // epsilonGraphThickening_ = 0.01;
}

ompl::geometric::QMPImpl::~QMPImpl()
{
    deleteConfiguration(xRandom_);
}

void ompl::geometric::QMPImpl::grow()
{
    if (firstRun_)
    {
        init();
        firstRun_ = false;
    }

    sampleBundleGoalBias(xRandom_->state, goalBias_);

    std::vector<Configuration*> r_nearest_neighbors;
     
    //TODO: Why 7? Why not use 10 like PRM?
    BaseT::nearestDatastructure_->nearestK(xRandom_, 7, r_nearest_neighbors);

    bool foundFeasibleEdge = false;
    
    for(unsigned int i=0 ; i< r_nearest_neighbors.size(); i++)
    {
        Configuration* q_neighbor = r_nearest_neighbors.at(i);
        if (Bundle->checkMotion(q_neighbor->state, xRandom_->state)) 
        {
            Vertex v_next;
            Configuration *q_next;
            //TODO: why only add one edge?
            if(!foundFeasibleEdge)
            {
    
                double d = Bundle->distance(q_neighbor->state, xRandom_->state);
                if (d > maxDistance_)
                {
                    Bundle->getStateSpace()->interpolate(q_neighbor->state, xRandom_->state, maxDistance_ / d, xRandom_->state);
                }

                // totalNumberOfSamples_++;
                // totalNumberOfFeasibleSamples_++;
                q_next = new Configuration(Bundle, xRandom_->state);
                v_next = addConfiguration(q_next);
            
                
                foundFeasibleEdge = true;
            }
            if (!hasSolution_ && foundFeasibleEdge)
            {
                //TODO: What happens if this edge is infeasible, but there has
                //been one feasible edge before? (i.e. foundfeasibleedge is set)
                addEdge(q_neighbor->index, v_next);
                
                double dist = 0.0;
                bool satisfied = goal_->isSatisfied(q_next->state, &dist);
                if (satisfied)
                {
                    vGoal_ = addConfiguration(qGoal_);
                    addEdge(q_neighbor->index, vGoal_);
                    hasSolution_ = true;
                }
            }
        }

    }
}

void ompl::geometric::QMPImpl::sampleFromDatastructure(base::State *xRandom)
{
    double p = rng_.uniform01();
    if(lengthStartGoalVertexPath_ > 0 && p < pathBias_)
    {
        //(1) Sample randomly on shortest path
        double p = rng_.uniform01() * lengthStartGoalVertexPath_;

        double t = 0;
        int ctr = 0;
        while(t < p && (ctr < startGoalVertexPath_.size()-1))
        {
            t += lengthsStartGoalVertexPath_.at(ctr);
            ctr++;
        }
        // std::cout << ctr << "/" << startGoalVertexPath_.size() << std::endl;
        const Vertex v1 = startGoalVertexPath_.at(ctr-1);
        const Vertex v2 = startGoalVertexPath_.at(ctr);
        double d = lengthsStartGoalVertexPath_.at(ctr-1);

        double s = d - (t - p);
        Bundle->getStateSpace()->interpolate(graph_[v1]->state, graph_[v2]->state, s, xRandom);

    }else{
        //(2) Sample randomly on graph
        BaseT::sampleFromDatastructure(xRandom);
    }

    //(3) Perturbate sample in epsilon neighborhood
    if(epsilonGraphThickening_>0) 
    {
        getBundleSamplerPtr()->sampleUniformNear(xRandom, xRandom, epsilonGraphThickening_);
    }

}

  //Edge e;
  //double t = rng_.uniform01();
  //if(t<percentageSamplesOnShortestPath)
  //{
  //  //shortest path heuristic
  //  percentageSamplesOnShortestPath = exp(-pow(((double)samplesOnShortestPath++/1000.0),2));
  //  e = pdf_edges_on_shortest_path.sample(rng_.uniform01());
  //}else{
  //  e = boost::random_edge(G, rng_boost);
  //  while(!sameComponent(boost::source(e, G), v_start))
  //  {
  //    e = boost::random_edge(G, rng_boost);
  //  }
  //}

  //double s = rng_.uniform01();

  //const Vertex v1 = boost::source(e, G);
  //const Vertex v2 = boost::target(e, G);
  //const ob::State *from = G[v1]->state;
  //const ob::State *to = G[v2]->state;

  //Q1->getStateSpace()->interpolate(from, to, s, q_random_graph);
  ////Q1_sampler->sampleGaussian(q_random_graph, q_random_graph, epsilon);
  //if(epsilon>0) Q1_sampler->sampleUniformNear(q_random_graph, q_random_graph, epsilon);
  //return true;
