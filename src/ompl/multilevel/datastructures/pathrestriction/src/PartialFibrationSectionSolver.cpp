#include <ompl/multilevel/datastructures/pathrestriction/PartialFibrationSectionSolver.h>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/Tree.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>

#include <queue>

namespace ompl {
namespace multilevel {

const float kRestrictionAccuracy = 1e-1;
const float kRestrictionAccuracyGoal = 1e-2;
const size_t kPropagationAttempts = 5;

struct TreeSearchNode {
  TreeNode* node{nullptr};
  size_t next_base_path_index{0};
  double distance_to_waypoint{std::numeric_limits<double>::infinity()};
};

struct TreeSearchNodeCmp
{
    bool operator()(const TreeSearchNode& lhs, const TreeSearchNode& rhs) const
    {
        if(lhs.next_base_path_index == rhs.next_base_path_index) {
          return lhs.distance_to_waypoint > rhs.distance_to_waypoint;
        }

        return lhs.next_base_path_index < rhs.next_base_path_index;
    }
};

std::vector<ompl::base::State*> makeStatesFromTreeNode(const TreeNode* node) {
  std::vector<ompl::base::State*> states;
  auto lastNode(node);
  while(lastNode != nullptr) {
    states.push_back(lastNode->getState());
    lastNode = lastNode->getParent();
  }
  std::reverse(states.begin(), states.end());
  return states;
}

typedef std::priority_queue<TreeSearchNode, std::vector<TreeSearchNode>, TreeSearchNodeCmp> OpenNodes;

bool stateExistsInOpenNodes(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const ompl::base::State* state, const OpenNodes& open_nodes) {

  OpenNodes tmp_open_nodes = open_nodes;

  while(!tmp_open_nodes.empty()) {
    auto node = tmp_open_nodes.top();
    if(factor->distance(state, node.node->getState()) < 1e-6) {
      return true;
    }
    tmp_open_nodes.pop();
  }
  return false;
}

std::optional<PathSectionPtr> partialFibrationSectionSolver(const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const TreePtr& tree, const PathRestrictionPtr& restriction, const ompl::base::State* targetState) {

  auto motion_validator = std::static_pointer_cast<ompl::multilevel::TaskSpaceMotionValidator>(factor->getMotionValidator());

  auto projection = restriction->getProjection();
  const auto base = projection->getBase();
  const auto bundle = projection->getBundle();

  auto bundleStateTmp = factor->allocState();
  auto baseStateTmp = base->allocState();

  OpenNodes open_nodes;

  TreeSearchNode start;
  start.node = tree->getRoot();
  start.next_base_path_index = 1;
  start.distance_to_waypoint = 0.0;
  open_nodes.push(start);

  while(!open_nodes.empty()) {

    auto node = open_nodes.top();
    open_nodes.pop();

    projection->project(node.node->getState(), baseStateTmp);
    auto baseWaypointState = restriction->getBaseStateAt(node.next_base_path_index);

    size_t counter = 0;

    while(counter++ < kPropagationAttempts) {
      projection->lift(baseWaypointState, bundleStateTmp);
      auto states = motion_validator->propagateMotion(node.node->getState(), bundleStateTmp);

      //Check if we made sufficient progress
      if(states.empty()) {
        continue;
      }

      if(stateExistsInOpenNodes(factor, states.back(), open_nodes)) {
        continue;
      }

      projection->project(states.back(), baseStateTmp);
      auto distance_to_target = base->distance(baseStateTmp, baseWaypointState);

      if(distance_to_target > kRestrictionAccuracy) {
        continue;
      }

      auto lastNode(node.node);
      for(const auto& state : states) {
        auto xNext = tree->addNodeAndParent(state, lastNode);
        lastNode = xNext;
      }

      auto distance_to_goal = bundle->distance(lastNode->getState(), targetState);
      if(distance_to_goal < kRestrictionAccuracyGoal) {
        auto total_states = makeStatesFromTreeNode(lastNode);
        factor->freeState(bundleStateTmp);
        base->freeState(baseStateTmp);
        return std::make_shared<PathSection>(total_states);
      }

      if(restriction->size() <= node.next_base_path_index + 1) {
        OMPL_WARN("Reached index %d/%d", node.next_base_path_index, restriction->size());
        continue;
      }

      TreeSearchNode new_node;
      new_node.node = lastNode;
      new_node.next_base_path_index = node.next_base_path_index + 1;
      new_node.distance_to_waypoint = distance_to_target;
      OMPL_WARN("create new node at waypoint %d (distance %f)", new_node.next_base_path_index, new_node.distance_to_waypoint);

      open_nodes.push(new_node);
    }

  }
  factor->freeState(bundleStateTmp);
  base->freeState(baseStateTmp);

  return std::nullopt;
}

}
}
