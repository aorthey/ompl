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

/* Author: Andreas Orthey */

#include <ompl/multilevelqrrt/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevelqrrt/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevelqrrt/datastructures/src/BundleSpaceGraphGoalVisitor.hpp>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/helpers/SamplingHelper.h>
#include <ompl/multilevel/planners/FactoredPlanner.h>

#include <ompl/multilevelqrrt/datastructures/graphsampler/RandomVertex.h>
#include <ompl/multilevelqrrt/datastructures/graphsampler/RandomDegreeVertex.h>
#include <ompl/multilevelqrrt/datastructures/graphsampler/RandomEdge.h>
#include <ompl/multilevelqrrt/datastructures/importance/Greedy.h>
#include <ompl/multilevelqrrt/datastructures/importance/Exponential.h>
#include <ompl/multilevelqrrt/datastructures/importance/Uniform.h>
#include <ompl/multilevelqrrt/datastructures/pathrestriction/PathRestriction.h>

#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/util/Exception.h>
#include <ompl/control/SpaceInformation.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using Configuration = ompl::multilevelqrrt::BundleSpaceGraph::Configuration;

using namespace ompl::multilevelqrrt;

BundleSpaceGraph::BundleSpaceGraph(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("BundleSpaceGraph");

    // Functional primitives
    setGraphSampler("randomvertex");
    setImportance("uniform");

    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = false;

    Planner::declareParam<double>("range", this, &BundleSpaceGraph::setRange, &BundleSpaceGraph::getRange, "0.:1.:"
                                                                                                           "10000.");

    Planner::declareParam<double>("goal_bias", this, &BundleSpaceGraph::setGoalBias, &BundleSpaceGraph::getGoalBias,
                                  "0.:.05:1.");

    xRandom_ = new Configuration(getBundle());

    internal_space_sampler_ = getBundle()->allocStateSampler();
}

BundleSpaceGraph::~BundleSpaceGraph()
{
    deleteConfiguration(xRandom_);
}

void BundleSpaceGraph::setup()
{
    BaseT::setup();

    ompl::tools::SelfConfig sc(getBundle(), getName());
    sc.configurePlannerRange(maxDistance_);

    OMPL_DEBUG("Range distance graph sampling: %f (max extent %f)", maxDistance_, getBundle()->getMaximumExtent());

    if (!nearestDatastructure_)
    {
        if (!isDynamic())
        {
            //nearestDatastructure_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
            if(getBundle()->getStateSpace()->isMetricSpace()) {
                nearestDatastructure_ = std::make_shared<NearestNeighborsGNATNoThreadSafety<Configuration*>>();
            } else { 
                nearestDatastructure_ = std::make_shared<NearestNeighborsSqrtApprox<Configuration*>>();
            }
        }
        else
        {
            // for dynamical systems, we use a quasimetric
            //(non-symmetric metric), so we cannot use NN structures like GNAT
            nearestDatastructure_.reset(new NearestNeighborsSqrtApprox<Configuration *>());
        }
        nearestDatastructure_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) { return distance(a, b); });
    }

    if (hasBaseSpace() && getProjection()->isFibered())
    {
        pathRestriction_ = std::make_shared<PathRestriction>(this);
    }

    if (pdef_)
    {
        setup_ = true;

        optimizer_ = std::make_shared<ompl::geometric::PathSimplifier>(getBundle(), pdef_->getGoal(),
                                                                       getOptimizationObjectivePtr());
        optimizer_->freeStates(false);
    }
    else
    {
        setup_ = false;
    }
}

bool BundleSpaceGraph::findSection()
{
    if (!hasBaseSpace())
    {
      return false;
    }
     
    if (!getProjection()->isFibered())
    {
      return false;
    }

    base::PathPtr basePath = static_cast<BundleSpaceGraph *>(getChild())->getSolutionPathByReference();
    pathRestriction_->setBasePath(basePath);

    if (pathRestriction_->hasFeasibleSection(qStart_, qGoal_))
    {
        if (sameComponent(vStart_, qGoal_->index))
        {
            hasSolution_ = true;
            return true;
        }
    }
    return false;

    // const base::PathPtr basePath = static_cast<BundleSpaceGraph *>(getChild())->getSolutionPathByReference();

    // auto path_restriction = std::make_shared<PathRestriction>(getBundle(), getProjection());
    // path_restriction->setBasePath(base_path);

    // auto find_section = std::make_shared<FindSectionSideStep>(path_restriction);
    // auto maybe_section = find_section->solve(tree_, qGoal);

    // if(!maybe_section.has_value()) {
    //   OMPL_ERROR("Could not compute section.");
    //   return false;
    // }


    // return true;
    //return success(maybe_section.value());
}

void BundleSpaceGraph::clear()
{
    BaseT::clear();

    clearVertices();
    pis_.restart();

    graphLength_ = 0;
    bestCost_ = base::Cost(base::dInf);
    setup_ = false;
    vStart_ = 0;
    shortestVertexPath_.clear();

    startConfigurations_.clear();
    goalConfigurations_.clear();

    if (!isDynamic())
    {
        if (solutionPath_ != nullptr)
        {
            std::static_pointer_cast<geometric::PathGeometric>(solutionPath_)->clear();
        }
    }

    numVerticesWhenComputingSolutionPath_ = 0;

    importanceCalculator_->clear();
    //graphSampler_->clear();
    if (pathRestriction_ != nullptr)
    {
        pathRestriction_->clear();
    }
}

void BundleSpaceGraph::clearVertices()
{
    if (nearestDatastructure_)
    {
        std::vector<Configuration *> configs;
        nearestDatastructure_->list(configs);
        for (auto &config : configs)
        {
            deleteConfiguration(config);
        }
        nearestDatastructure_->clear();
    }
    graph_.clear();
}

void BundleSpaceGraph::setGoalBias(double goalBias)
{
    goalBias_ = goalBias;
}

double BundleSpaceGraph::getGoalBias() const
{
    return goalBias_;
}

void BundleSpaceGraph::setRange(double maxDistance)
{
    maxDistance_ = maxDistance;
}

double BundleSpaceGraph::getRange() const
{
    return maxDistance_;
}

BundleSpaceGraph::Configuration::Configuration(const ompl::base::SpaceInformationPtr &si) : state(si->allocState())
{
}
BundleSpaceGraph::Configuration::Configuration(const ompl::base::SpaceInformationPtr &si,
                                               const ompl::base::State *state_)
  : state(si->cloneState(state_))
{
}

void BundleSpaceGraph::deleteConfiguration(Configuration *q)
{
    if (q != nullptr)
    {
        if (q->state != nullptr)
        {
            getBundle()->freeState(q->state);
        }
        for (unsigned int k = 0; k < q->reachableSet.size(); k++)
        {
            Configuration *qk = q->reachableSet.at(k);
            if (qk->state != nullptr)
            {
                getBundle()->freeState(qk->state);
            }
        }
        if (isDynamic())
        {
            const ompl::control::SpaceInformationPtr siC =
                std::static_pointer_cast<ompl::control::SpaceInformation>(getBundle());
            siC->freeControl(q->control);
        }
        q->reachableSet.clear();

        delete q;
        q = nullptr;
    }
}

double BundleSpaceGraph::getImportance() const
{
    return importanceCalculator_->eval();
}

void BundleSpaceGraph::init()
{
    if (const base::State *state = pis_.nextStart())
    {
        qStart_ = new Configuration(getBundle(), state);
        vStart_ = addConfiguration(qStart_);
        qStart_->isStart = true;
    }

    if (qStart_ == nullptr)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        throw ompl::Exception("Invalid initial states.");
    }

    if (const base::State *state = pis_.nextGoal())
    {
        qGoal_ = new Configuration(getBundle(), state);
        qGoal_->isGoal = true;
    }

    if (qGoal_ == nullptr && getGoalPtr()->canSample())
    {
        OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
        throw ompl::Exception("Invalid goal states.");
    }
}

void BundleSpaceGraph::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool BundleSpaceGraph::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

const BundleSpaceGraph::Configuration *BundleSpaceGraph::nearest(const Configuration *q) const
{
    return nearestDatastructure_->nearest(const_cast<Configuration *>(q));
}

BundleSpaceGraph::Configuration *BundleSpaceGraph::addBundleConfiguration(ompl::base::State *state)
{
    Configuration *x = new Configuration(getBundle(), state);
    addConfiguration(x);
    return x;
}

void BundleSpaceGraph::addBundleEdge(const Configuration *a, const Configuration *b)
{
    addEdge(a->index, b->index);
}

BundleSpaceGraph::Vertex BundleSpaceGraph::addConfiguration(Configuration *q)
{
    Vertex m = boost::add_vertex(q, graph_);
    graph_[m]->total_connection_attempts = 1;
    graph_[m]->successful_connection_attempts = 0;
    disjointSets_.make_set(m);

    nearestDatastructure_->add(q);
    q->index = m;

    return m;
}

unsigned int BundleSpaceGraph::getNumberOfVertices() const
{
    return num_vertices(graph_);
}

unsigned int BundleSpaceGraph::getNumberOfEdges() const
{
    return num_edges(graph_);
}

const BundleSpaceGraph::Graph &BundleSpaceGraph::getGraph() const
{
    return graph_;
}

BundleSpaceGraph::Graph &BundleSpaceGraph::getGraphNonConst()
{
    return graph_;
}

const BundleSpaceGraph::RoadmapNeighborsPtr &BundleSpaceGraph::getRoadmapNeighborsPtr() const
{
    return nearestDatastructure_;
}

ompl::base::Cost BundleSpaceGraph::costHeuristic(Vertex u, Vertex v) const
{
    return getOptimizationObjectivePtr()->motionCostHeuristic(graph_[u]->state, graph_[v]->state);
}

template <template <typename T> class NN>
void BundleSpaceGraph::setNearestNeighbors()
{
    if (nearestDatastructure_ && nearestDatastructure_->size() == 0)
        OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
    nearestDatastructure_ = std::make_shared<NN<base::State *>>();
    if (!isSetup())
    {
        setup();
    }
}

double BundleSpaceGraph::distance(const Configuration *a, const Configuration *b) const
{
    return getBundle()->distance(a->state, b->state);
}

bool BundleSpaceGraph::checkMotion(const Configuration *a, const Configuration *b) const
{
    return getBundle()->checkMotion(a->state, b->state);
}

void BundleSpaceGraph::interpolate(const Configuration *a, const Configuration *b, Configuration *dest) const
{
    getBundle()->getStateSpace()->interpolate(a->state, b->state, 1.0, dest->state);
}

bool BundleSpaceGraph::steer(const Configuration *from, const Configuration *to,
                                                             Configuration *result)
{
    interpolate(from, to, result);
    bool val = getBundle()->checkMotion(from->state, result->state);
    return val;
}

Configuration *BundleSpaceGraph::steerTowards(const Configuration *from, const Configuration *to)
{
    Configuration *next = new Configuration(getBundle(), to->state);

    if (!steer(from, to, next))
    {
        deleteConfiguration(next);
        return nullptr;
    }
    return next;
}

Configuration *BundleSpaceGraph::steerTowards_Range(const Configuration *from, Configuration *to)
{
    double d = distance(from, to);
    if (d >= std::numeric_limits<double>::infinity()) {
        return nullptr;
    }
    if (d > maxDistance_)
    {
        getBundle()->getStateSpace()->interpolate(from->state, to->state, maxDistance_ / d, to->state);
    }

    if (!steer(from, to, to))
    {
        return nullptr;
    }
    Configuration *next = new Configuration(getBundle(), to->state);
    return next;
}

Configuration *BundleSpaceGraph::extendGraphTowards_Range(const Configuration *from, Configuration *to)
{
    if (!isDynamic())
    {
        double d = distance(from, to);
        if (d > maxDistance_)
        {
            getBundle()->getStateSpace()->interpolate(from->state, to->state, maxDistance_ / d, to->state);
        }
    }

    if (!steer(from, to, to))
    {
        return nullptr;
    }

    Configuration *next = new Configuration(getBundle(), to->state);
    addConfiguration(next);
    addBundleEdge(from, next);
    return next;
}

bool BundleSpaceGraph::connect(const Configuration *from, const Configuration *to)
{
    if (!steer(from, to, xRandom_))
    {
        return false;
    }

    addBundleEdge(from, to);
    return true;
}

void BundleSpaceGraph::setImportance(const std::string &sImportance)
{
    if (sImportance == "uniform")
    {
        OMPL_DEVMSG1("Uniform Importance Selected");
        importanceCalculator_ = std::make_shared<BundleSpaceImportanceUniform>(this);
    }
    else if (sImportance == "greedy")
    {
        OMPL_DEVMSG1("Greedy Importance Selected");
        importanceCalculator_ = std::make_shared<BundleSpaceImportanceGreedy>(this);
    }
    else if (sImportance == "exponential")
    {
        OMPL_DEBUG("Exponential Importance Selected");
        importanceCalculator_ = std::make_shared<BundleSpaceImportanceExponential>(this);
    }
    else
    {
        OMPL_ERROR("Importance calculator unknown: %s", sImportance.c_str());
        throw ompl::Exception("Unknown Importance");
    }
}

void BundleSpaceGraph::setGraphSampler(const std::string &sGraphSampler)
{
    if (sGraphSampler == "randomvertex")
    {
        OMPL_DEVMSG1("Random Vertex Sampler Selected");
        graphSampler_ = std::make_shared<BundleSpaceGraphSamplerRandomVertex>(this);
    }
    else if (sGraphSampler == "randomedge")
    {
        OMPL_DEVMSG1("Random Edge Sampler Selected");
        graphSampler_ = std::make_shared<BundleSpaceGraphSamplerRandomEdge>(this);
    }
    else if (sGraphSampler == "randomdegreevertex")
    {
        OMPL_DEVMSG1("Random Degree Vertex Sampler Selected");
        graphSampler_ = std::make_shared<BundleSpaceGraphSamplerRandomDegreeVertex>(this);
    }
    else
    {
        OMPL_ERROR("Sampler unknown: %s", sGraphSampler.c_str());
        throw ompl::Exception("Unknown Graph Sampler");
    }
}

BundleSpaceGraphSamplerPtr BundleSpaceGraph::getGraphSampler()
{
    return graphSampler_;
}

const std::pair<BundleSpaceGraph::Edge, bool> BundleSpaceGraph::addEdge(const Vertex a, const Vertex b)
{
    base::Cost weight = getOptimizationObjectivePtr()->motionCost(graph_[a]->state, graph_[b]->state);
    EdgeInternalState properties(weight);
    const std::pair<Edge, bool> e = boost::add_edge(a, b, properties, graph_);
    uniteComponents(a, b);
    return e;
}

BundleSpaceGraph::Vertex BundleSpaceGraph::getStartIndex() const
{
    return vStart_;
}

void BundleSpaceGraph::addGoalConfiguration(Configuration *x)
{
    goalConfigurations_.push_back(x);
    if (getOptimizationObjectivePtr()->isCostBetterThan(x->cost, bestCost_))
    {
        bestCost_ = x->cost;
    }
}

BundleSpaceGraph::Vertex BundleSpaceGraph::getGoalIndex() const
{
    if (goalConfigurations_.size() > 0)
    {
        return goalConfigurations_.front()->index;
    }
    else
    {
        OMPL_DEVMSG1("Returned NullVertex");
        return nullVertex();
    }
}

BundleSpaceGraph::Vertex BundleSpaceGraph::nullVertex() const
{
    return BGT::null_vertex();
}

void BundleSpaceGraph::setStartIndex(Vertex idx)
{
    vStart_ = idx;
}

ompl::base::PathPtr &BundleSpaceGraph::getSolutionPathByReference()
{
    return solutionPath_;
}

bool BundleSpaceGraph::getSolution(ompl::base::PathPtr &solution)
{
    if (hasSolution_)
    {
        if ((solutionPath_ != nullptr) && (getNumberOfVertices() == numVerticesWhenComputingSolutionPath_))
        {
        }
        else
        {
            Vertex goalVertex;
            for (unsigned int k = 0; k < goalConfigurations_.size(); k++)
            {
                Configuration *qk = goalConfigurations_.at(k);
                if (sameComponent(vStart_, qk->index))
                {
                    solutionPath_ = getPath(vStart_, qk->index);
                    goalVertex = qk->index;
                    break;
                }
            }
            if (solutionPath_ == nullptr)
            {
                throw "hasSolution_ is set, but no solution exists.";
            }
            numVerticesWhenComputingSolutionPath_ = getNumberOfVertices();

            if (!isDynamic() && solutionPath_ != solution && hasParent())
            {
                // @NOTE: optimization seems to improve feasibility of sections
                // in low-dim problems (up to 20 dof roughly), but will take too
                // much time for high-dim problems. Reducing vertices seems to
                // be the only optimization not significantly slowing everything
                // down.

                bool valid = false;
                for (unsigned int k = 0; k < 3; k++)
                {
                    geometric::PathGeometric &gpath = static_cast<geometric::PathGeometric &>(*solutionPath_);

                    valid = optimizer_->reduceVertices(gpath, 0, 0, 0.1);

                    if (!valid)
                    {
                        // reset solutionPath
                        solutionPath_ = getPath(vStart_, goalVertex);
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
        solution = solutionPath_;
        return true;
    }
    else
    {
        return false;
    }
}

ompl::base::PathPtr BundleSpaceGraph::getPath(const Vertex &start, const Vertex &goal)
{
    return getPath(start, goal, graph_);
}

ompl::base::PathPtr BundleSpaceGraph::getPath(const Vertex &start, const Vertex &goal, Graph &graph)
{
    std::vector<Vertex> prev(boost::num_vertices(graph));
    auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost),
                                                           get(boost::edge_bundle, graph));

    try
    {
        boost::astar_search(graph, start, [this, goal](const Vertex v) { return costHeuristic(v, goal); },
                            boost::predecessor_map(&prev[0])
                                .weight_map(weight)
                                .distance_compare([this](EdgeInternalState c1, EdgeInternalState c2) {
                                    return getOptimizationObjectivePtr()->isCostBetterThan(c1.getCost(), c2.getCost());
                                })
                                .distance_combine([this](EdgeInternalState c1, EdgeInternalState c2) {
                                    return getOptimizationObjectivePtr()->combineCosts(c1.getCost(), c2.getCost());
                                })
                                .distance_inf(getOptimizationObjectivePtr()->infiniteCost())
                                .distance_zero(getOptimizationObjectivePtr()->identityCost())
                                .visitor(BundleSpaceGraphGoalVisitor<Vertex>(goal)));
    }
    catch (BundleSpaceGraphFoundGoal &)
    {
    }

    auto p(std::make_shared<geometric::PathGeometric>(getBundle()));
    if (prev[goal] == goal)
    {
        return nullptr;
    }

    std::vector<Vertex> vpath;
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
    {
        graph[pos]->on_shortest_path = true;
        vpath.push_back(pos);
        p->append(graph[pos]->state);
    }
    graph[start]->on_shortest_path = true;
    vpath.push_back(start);
    p->append(graph[start]->state);

    shortestVertexPath_.clear();
    shortestVertexPath_.insert(shortestVertexPath_.begin(), vpath.rbegin(), vpath.rend());
    p->reverse();

    return p;
}

void BundleSpaceGraph::sampleBundleGoalBias(ompl::base::State *xRandom)
{
    if (hasSolution_)
    {
        // No Goal Biasing if we already found a solution on this bundle space
        sampleBundle(xRandom);
    }
    else
    {
        double s = rng_.uniform01();
        if (s < goalBias_ && getGoalPtr()->canSample())
        {
            getGoalPtr()->sampleGoal(xRandom);
        }
        else
        {
            sampleBundle(xRandom);
        }
    }
}

void BundleSpaceGraph::sampleFromDatastructure(ompl::base::State *xRandom)
{
    //graphSampler_->sample(xRandom);
  ompl::multilevel::DatastructureInformation info;
        // const double kDefaultPathRestrictionSamplingBias = 0.5;
        // const double kDefaultPathRestrictionSurroundingBias = 0.1;
        // const double kDefaultSamplingPerturbationValue  = 0.05;
  info.path_restriction_surrounding_sampling_bias = ompl::multilevel::kDefaultPathRestrictionSurroundingBias;
  info.path_restriction_sampling_bias = ompl::multilevel::kDefaultPathRestrictionSamplingBias;
  info.sampling_perturbation_bias = ompl::multilevel::kDefaultSamplingPerturbationValue;
  info.sampler = internal_space_sampler_;
  info.pdef = getProblemDefinition();
  info.si = getBundle();

  const auto& graph = getGraph();
  info.number_of_nodes = boost::num_vertices(graph);

  ::sampleFromDatastructure(info, graph, rng_, xRandom);
}

void BundleSpaceGraph::writeToGraphviz(std::string filename) const
{
    std::ofstream f(filename.c_str());
    std::vector<std::string> annotationVec;
    foreach (const Vertex v, boost::vertices(graph_))
    {
        Configuration *qv = graph_[v];
        const base::State *s = qv->state;
        std::ostringstream out;
        getBundle()->printState(s, out);
        annotationVec.push_back(out.str());
    }
    write_graphviz(f, graph_, boost::make_label_writer(&annotationVec[0]));
}

void BundleSpaceGraph::print(std::ostream &out) const
{
    BaseT::print(out);
    out << std::endl
        << " --[BundleSpaceGraph has " << getNumberOfVertices() << " vertices and " << getNumberOfEdges() << " edges.]"
        << std::endl;
}

void BundleSpaceGraph::printConfiguration(const Configuration *q) const
{
    getBundle()->printState(q->state);
}

void BundleSpaceGraph::getPlannerDataGraph(ompl::base::PlannerData &data, const Graph &graph, const Vertex vStart) const
{
    if (boost::num_vertices(graph) <= 0)
        return;

    multilevelqrrt::PlannerDataVertexAnnotated pstart(graph[vStart]->state);
    pstart.setLevel(getLevel());
    data.addStartVertex(pstart);

    for (unsigned int k = 0; k < goalConfigurations_.size(); k++)
    {
        Configuration *qgoal = goalConfigurations_.at(k);
        multilevelqrrt::PlannerDataVertexAnnotated pgoal(qgoal->state);
        pgoal.setLevel(getLevel());
        data.addGoalVertex(pgoal);
    }
    if (hasSolution_)
    {
        if (solutionPath_ != nullptr)
        {
            geometric::PathGeometric &gpath = static_cast<geometric::PathGeometric &>(*solutionPath_);

            std::vector<base::State *> gstates = gpath.getStates();

            multilevelqrrt::PlannerDataVertexAnnotated *pLast = &pstart;

            for (unsigned int k = 1; k < gstates.size(); k++)
            {
                multilevelqrrt::PlannerDataVertexAnnotated p(gstates.at(k));
                p.setLevel(getLevel());
                data.addVertex(p);
                data.addEdge(*pLast, p);
                pLast = &p;
            }
        }
    }

    foreach (const Edge e, boost::edges(graph))
    {
        const Vertex v1 = boost::source(e, graph);
        const Vertex v2 = boost::target(e, graph);

        multilevelqrrt::PlannerDataVertexAnnotated p1(graph[v1]->state);
        multilevelqrrt::PlannerDataVertexAnnotated p2(graph[v2]->state);
        p1.setLevel(getLevel());
        p2.setLevel(getLevel());
        data.addEdge(p1, p2);
    }
    foreach (const Vertex v, boost::vertices(graph))
    {
        multilevelqrrt::PlannerDataVertexAnnotated p(graph[v]->state);
        p.setLevel(getLevel());
        data.addVertex(p);
    }
}

void BundleSpaceGraph::getPlannerData(ompl::base::PlannerData &data) const
{
    OMPL_DEBUG("Graph (level %d) has %d/%d vertices/edges", getLevel(), boost::num_vertices(graph_),
               boost::num_edges(graph_));

    if (bestCost_.value() < ompl::base::dInf)
    {
        OMPL_DEBUG("Best Cost: %.2f", bestCost_.value());
    }
    getPlannerDataGraph(data, graph_, vStart_);
}
