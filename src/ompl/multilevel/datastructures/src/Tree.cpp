#include "ompl/multilevel/datastructures/Tree.h"

#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"

using namespace ompl::multilevel;

Tree::Tree(const base::SpaceInformationPtr &si)
  : si_(si) {
    if (!nn_) {
        if(si->getStateSpace()->isMetricSpace()) {
          nn_ = std::make_shared<NearestNeighborsGNATNoThreadSafety<TreeNode*>>();
        } else { 
          nn_ = std::make_shared<NearestNeighborsSqrtApprox<TreeNode*>>();
        }
    }
    nn_->setDistanceFunction([this](const TreeNode *a, const TreeNode *b) { return distance(si_, a, b); });
}

Tree::~Tree() {
    clear();
}

void Tree::clear() {
    if (nn_)
    {
        std::vector<TreeNode *> nodes;
        nn_->list(nodes);
        for (auto &node : nodes)
        {
            if (node->getState() != nullptr)
                si_->freeState(node->getState());
            delete node;
        }
        nn_->clear();
    }
}

size_t Tree::size() const {
    if(nn_) {
        return nn_->size();
    }
    return 0;
}

std::vector<TreeNode*> Tree::getNodes() const {
    //return nodes_;
    std::vector<TreeNode*> nodes;
    if (nn_)
        nn_->list(nodes);
    return nodes;
}

TreeNode* Tree::addNodeAndParent(const ompl::base::State* state, TreeNode* parent) {
  auto *node = new TreeNode(si_, state);
  node->setParent(parent);
  if(parent != nullptr) {
    parent->addChild(node);
  }

  nn_->add(node);
  nodes_.push_back(node);
  if(nodes_.size() == 1) {
    root_ = node;
  }
  return node;
}

ompl::base::SpaceInformationPtr Tree::getSpaceInformation() const {
  return si_;
}

TreeNode* Tree::nearest(TreeNode* parent) const {
  return nn_->nearest(parent);
}

TreeNode* Tree::getRoot() const {
  return root_;
}
