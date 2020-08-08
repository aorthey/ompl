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

/* Author: Andreas Orthey, Sohaib Akbar */

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_SPARSE_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_SPARSE_

#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathGeometric.h>

#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathSimplifier);
    }
    namespace multilevel
    {
        class BundleSpaceGraphSparse : public BundleSpaceGraph
        {
            using BaseT = BundleSpaceGraph;

        public:
            BundleSpaceGraphSparse(const base::SpaceInformationPtr &si, BundleSpace *parent = nullptr);
            virtual ~BundleSpaceGraphSparse() override;

            virtual void grow() = 0;
            virtual bool getSolution(ompl::base::PathPtr &solution) override;

            virtual void getPlannerData(base::PlannerData &data) const override;
            void getPlannerDataRoadmap(base::PlannerData &data, std::vector<int> pathIdx) const;

            virtual Vertex addConfiguration(Configuration *q) override;
            virtual Vertex addConfigurationConditional(Configuration *q);

            virtual void deleteConfiguration(Configuration *q) override;

            virtual void addEdge(const Vertex a, const Vertex b) override;

            virtual unsigned int getNumberOfVertices() const;
            virtual unsigned int getNumberOfEdges() const;

            virtual void setGraphSampler(const std::string &sGraphSampler) override;

            virtual void setup() override;
            virtual void clear() override;

            virtual void init() override;

            virtual const Graph &getGraph() const override;

            // Using same conditions as SPARS algorithm to determine sparse graph
            // addition
            void findGraphNeighbors(Configuration *q, std::vector<Configuration *> &graphNeighborhood,
                                    std::vector<Configuration *> &visibleNeighborhood);
            bool checkAddCoverage(Configuration *q, std::vector<Configuration *> &visibleNeighborhoods);
            bool checkAddConnectivity(Configuration *q, std::vector<Configuration *> &visibleNeighborhood);
            bool checkAddInterface(Configuration *q, std::vector<Configuration *> &graphNeighborhood,
                                   std::vector<Configuration *> &visibleNeighborhood);
            bool checkAddPath(Configuration *q);

            void updateRepresentatives(Configuration *q);
            void getInterfaceNeighborRepresentatives(Configuration *q, std::set<Vertex> &interfaceRepresentatives);
            void addToRepresentatives(Vertex q, Vertex rep, const std::set<Vertex> &interfaceRepresentatives);
            void removeFromRepresentatives(Configuration *q);

            void getInterfaceNeighborhood(Configuration *q, std::vector<Vertex> &interfaceNeighborhood);
            void computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs);
            void computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs);
            Vertex getInterfaceNeighbor(Vertex q, Vertex rep);
            void computeDensePath(const Vertex &start, const Vertex &goal, std::deque<base::State *> &path);
            bool addPathToSpanner(const std::deque<base::State *> &dense_path, Vertex vp, Vertex vpp);

            void updatePairPoints(Configuration *q);

            virtual void print(std::ostream &out) const override;
            bool hasSparseGraphChanged();

            virtual const Configuration *nearest(const Configuration *s) const;

            double getSparseDelta() const
            {
                return sparseDelta_;
            }

        protected:
            double sparseDelta_{0.};
            double denseDelta_{0.};  // delta for dense graph -> move this BundleSpaceGraph.h
            double sparseDeltaFraction_{0.15};
            double denseDeltaFraction_{0.001};
            unsigned Nold_v{0};
            unsigned Nold_e{0};

            void setSparseDeltaFraction(double D)
            {
                sparseDeltaFraction_ = D;
            }
            double getSparseDeltaFraction() const
            {
                return sparseDeltaFraction_;
            }

            virtual void uniteComponentsSparse(Vertex m1, Vertex m2);
            bool sameComponentSparse(Vertex m1, Vertex m2);

            std::map<Vertex, VertexRank> vrankSparse;
            std::map<Vertex, Vertex> vparentSparse;
            boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank>>,
                                 boost::associative_property_map<std::map<Vertex, Vertex>>>
                disjointSetsSparse_{boost::make_assoc_property_map(vrankSparse),
                                    boost::make_assoc_property_map(vparentSparse)};

            ompl::base::PathPtr getPathSparse(const Vertex &start, const Vertex &goal);
            ompl::base::Cost costHeuristicSparse(Vertex u, Vertex v) const;

            Graph graphSparse_;
            RoadmapNeighborsPtr nearestSparse_;
            std::vector<Configuration *> graphNeighborhood;
            std::vector<Configuration *> visibleNeighborhood;

            Vertex v_start_sparse;
            Vertex v_goal_sparse;

            // From SPARS
            /** \brief A counter for the number of consecutive failed iterations of the algorithm */
            unsigned int consecutiveFailures_{0u};

            /** \brief The stretch factor in terms of graph spanners for SPARS to check against */
            double stretchFactor_{3.};

            /** \brief Geometric Path variable used for smoothing out paths. */
            geometric::PathGeometric geomPath_;

            /** \brief A path simplifier used to simplify dense paths added to S */
            geometric::PathSimplifierPtr psimp_;

            std::vector<Vertex> startGoalVertexPath_;
            std::vector<double> lengthsStartGoalVertexPath_;
        };
    };
};

#endif