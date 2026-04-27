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

#include <ompl/multilevelqrrt/planners/qrrt/QRRTImpl.h>
#include <ompl/multilevelqrrt/datastructures/graphsampler/GraphSampler.h>

#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionSideStep.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

ompl::multilevelqrrt::QRRTImpl::QRRTImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("QRRTImpl" + std::to_string(id_));
    setImportance("exponential");
    setGraphSampler("randomvertex");

    getGraphSampler()->disableSegmentBias();
    specs_.approximateSolutions = true;
    specs_.directed = true;
}

ompl::multilevelqrrt::QRRTImpl::~QRRTImpl()
{
}


#include <queue>
#include <unordered_map>

void ompl::multilevelqrrt::QRRTImpl::addTreeToGraph(const std::shared_ptr<ompl::multilevel::Tree>& tree) {
    if (!tree || !tree->getRoot()) return;

    TreeNode* root = tree->getRoot();

    std::unordered_map<TreeNode*, Configuration*> nodeToConfig;

    nodeToConfig[root] = qStart_;

    std::queue<TreeNode*> q;
    q.push(root);

    while (!q.empty()) {
        TreeNode* node = q.front();
        q.pop();

        // Add configuration only once per node
        auto it = nodeToConfig.find(node);
        if (it == nodeToConfig.end()) {
            auto config = addBundleConfiguration(node->getState());
            nodeToConfig[node] = config;
        }
        auto config = nodeToConfig[node];

        // Add edge if it has a parent (parent is guaranteed to exist in the map)
        if (node->getParent() != nullptr) {
            auto parentConfig = nodeToConfig[node->getParent()];  // must exist
            addBundleEdge(parentConfig, config);
        }

        // Enqueue children
        for (TreeNode* child : node->getChildren()) {   // adapt to your actual API
            q.push(child);
        }
    }
}

bool ompl::multilevelqrrt::QRRTImpl::findSectionSameAsFibrationRRT()
{
    if (!hasBaseSpace())
    {
      return false;
    }
     
    if (!getProjection()->isFibered())
    {
      return false;
    }
    using ompl::multilevel::TreeNode;
    using ompl::multilevel::Tree;
    auto tree = std::make_shared<Tree>(getBundle());

    tree->addNodeAndParent(qStart_->state, nullptr);
    base::PathPtr basePath = static_cast<BundleSpaceGraph *>(getChild())->getSolutionPathByReference();

    ompl::multilevel::PathRestrictionPtr pathRestriction = std::make_shared<ompl::multilevel::PathRestriction>(getBundle(), getProjection());
    pathRestriction->setBasePath(basePath);

    auto find_section = std::make_shared<ompl::multilevel::FindSectionSideStep>(pathRestriction);
    auto maybe_section = find_section->solve(tree, qGoal_->state);
    if(maybe_section.has_value()) {
      addTreeToGraph(tree);
    }
    return true;
}

void ompl::multilevelqrrt::QRRTImpl::grow()
{
    //(0) If first run, add start configuration
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        //Old method
        //findSection();

        //New method
        findSectionSameAsFibrationRRT();

    }
    //(1) Get Random Sample
    sampleBundleGoalBias(xRandom_->state);

    //(2) Get Nearest in Tree
    const Configuration *xNearest = nearest(xRandom_);

    //(3) Connect Nearest to Random (within range)
    Configuration *xNext = extendGraphTowards_Range(xNearest, xRandom_);

    //(4) If extension was successful, check if we reached goal
    if (xNext && !hasSolution_)
    {
        if (isDynamic())
        {
            double dist;
            bool satisfied = getGoalPtr()->isSatisfied(xNext->state, &dist);
            if (dist < bestCost_.value())
            {
                bestCost_ = base::Cost(dist);
            }
            if (satisfied)
            {
                goalConfigurations_.push_back(xNext);
                hasSolution_ = true;
            }
        }
        else
        {
            bool satisfied = getGoalPtr()->isSatisfied(xNext->state);
            if (satisfied)
            {
                goalConfigurations_.push_back(xNext);
                hasSolution_ = true;
            }
        }
    }
}
