#include "ompl/multilevel/datastructures/TreeNode.h"

namespace ompl {
namespace multilevel {

TreeNode::TreeNode(const base::SpaceInformationPtr &si) : state_(si->allocState()) {};

TreeNode::TreeNode(const base::SpaceInformationPtr &si, const ompl::base::State* state) {
  state_ = si->allocState();
  si->copyState(state_, state);
  static size_t id_counter = 0;
  id_ = id_counter;
  id_counter++;
}

TreeNode::~TreeNode() {
  children_.clear();
}

ompl::base::State* TreeNode::getState() const {
  return state_;
}

void TreeNode::setState(ompl::base::State* state) {
  state_ = state;
}

TreeNode* TreeNode::getParent() const {
  return parent_;
}

void TreeNode::setParent(TreeNode* parent) {
  parent_ = parent;
}

void TreeNode::addChild(TreeNode* node) {
  children_.push_back(node);
}

std::vector<TreeNode*> TreeNode::getChildren() const {
  return children_;
}

double distance(const base::SpaceInformationPtr& si, const TreeNode *a, const TreeNode *b) 
{
    return si->distance(a->getState(), b->getState());
}


}
}
