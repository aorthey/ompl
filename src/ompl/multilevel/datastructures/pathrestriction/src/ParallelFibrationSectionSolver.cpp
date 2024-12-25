#include <ompl/multilevel/datastructures/pathrestriction/ParallelFibrationSectionSolver.h>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>
#include <ompl/multilevel/datastructures/Tree.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>

namespace ompl {
namespace multilevel {

bool checkMotion(const FactoredSpaceInformationPtr& factor, const TreePtr& tree, TreeNode* node, const std::vector<ompl::base::State*>& states) {
  std::pair<base::State *, double> lastValid;
  lastValid.first = factor->allocState();
  lastValid.second = 0.0;

  // factor->printState(states.front());
  // factor->printState(states.back());
  TreeNode* lastNode(node);
  for (unsigned int k = 1; k < states.size(); k++)
  {
      if (!factor->checkMotion(lastNode->getState(), states.at(k), lastValid))
      {
        //std::cout << "Failed check motion" << std::endl;
        //factor->printState(lastValid.first);
        return false;
      }
      auto xNext = tree->addNodeAndParent(states.at(k), lastNode);
      //std::cout << "New state added during check motion" << std::endl;
      //factor->printState(states.at(k));
      lastNode = xNext;
  }
  return true;
}

std::vector<ompl::base::State*> makeSectionPathL2(const FactoredSpaceInformationPtr& factor, const std::unordered_map<std::string, PathRestrictionPtr>& path_restrictions) {
  auto children = factor->getChildren();

  if(children.size() != path_restrictions.size()) {
    throw ompl::Exception("Path restriction must match child spaces");
  }

  //////////////////////////////////////////////////////////////////////////////////
  //Get all base states and their respective positions along the path
  //////////////////////////////////////////////////////////////////////////////////
  std::vector<BasePathStateInformation> joint_base_path_states;

  for(const auto& restriction : path_restrictions) {
      const auto& basePath = restriction.second->getBasePath();
      const auto L = restriction.second->getLengthBasePath();
      const auto N = basePath.size();
      for(size_t k = 0; k < N; k++) {
        auto d = restriction.second->getLengthBasePathUntil(k) / L;
        BasePathStateInformation state_information;
        state_information.restriction_name = restriction.first;
        state_information.base_path_index = k;
        state_information.position_on_base_path = d;
        std::cout << "Restriction " << restriction.first << " state at index " << k << " position " << d << std::endl;
        joint_base_path_states.push_back(state_information);
      }
  }

  //////////////////////////////////////////////////////////////////////////////////
  // Sort all states and insert support states
  //////////////////////////////////////////////////////////////////////////////////

  std::sort(joint_base_path_states.begin(), joint_base_path_states.end(), 
      [](auto lhs, auto rhs) {
          return lhs.position_on_base_path < rhs.position_on_base_path;
      }
  );
  joint_base_path_states.erase( 
      std::unique( joint_base_path_states.begin(), joint_base_path_states.end(), 
        [](const auto& lhs, const auto& rhs) -> bool {
          return std::abs(lhs.position_on_base_path - rhs.position_on_base_path) < std::numeric_limits<double>::epsilon();
        }
      ), 
      joint_base_path_states.end() );


  auto statesChild = factor->allocChildStates();
  std::vector<ompl::base::State*> states;

  for(const auto& state_information : joint_base_path_states) {
    OMPL_ERROR("State on restriction %s at index %d (position %f).", state_information.restriction_name.c_str(), 
        state_information.base_path_index, state_information.position_on_base_path);

    const auto restriction_name = state_information.restriction_name;
    auto base_state = path_restrictions.at(restriction_name)->getBaseStateAt(state_information.base_path_index);
    auto s = state_information.position_on_base_path;

    for(const auto& path_restriction : path_restrictions) {
      auto name = path_restriction.first;
      if(name == restriction_name) {
        factor->getChild(name)->copyState(statesChild.at(name), base_state);
        continue;
      }
      path_restriction.second->interpolateBasePath(s * path_restriction.second->getLengthBasePath(), statesChild.at(name));
    }

    ////////////////////////////////////////////////////////////////////////////////
    //Lift samples to factor space
    ////////////////////////////////////////////////////////////////////////////////
    auto state = factor->allocState();
    factor->lift(statesChild, state);
    states.push_back(state);
  }

  //////////////////////////////////////////////////////////////////////////////////
  // Print states
  //////////////////////////////////////////////////////////////////////////////////
  // for(const auto& state : states) {
  //   factor->printState(state);
  // }

  factor->freeChildStates(statesChild);
  return states;
}

std::vector<ompl::base::State*> makeSectionPathL1(const FactoredSpaceInformationPtr& factor, const std::unordered_map<std::string, PathRestrictionPtr>& path_restrictions) {
  auto children = factor->getChildren();

  if(children.size() != path_restrictions.size()) {
    throw ompl::Exception("Path restriction must match child spaces");
  }

  //////////////////////////////////////////////////////////////////////////////////
  //Get all base states and their respective positions along the path
  //////////////////////////////////////////////////////////////////////////////////
  auto statesChild = factor->allocChildStates();

  std::vector<ompl::base::State*> states;

  std::unordered_map<std::string, bool> finished_interpolating;

  for(const auto& path_restriction : path_restrictions) {
      const auto name = path_restriction.first;
      finished_interpolating[name] = false;
  }

  size_t counter = 0;
  for(const auto& path_restriction : path_restrictions) {
      const auto& basePath = path_restriction.second->getBasePath();
      const auto L = path_restriction.second->getLengthBasePath();
      const auto N = basePath.size();
      const auto name = path_restriction.first;

      size_t startK = (counter == 0 ? 0 : 1); //skip first state because it is already the end state of the last segment
      counter++;
      for(size_t k = startK; k < N; k++) {
        auto d = path_restriction.second->getLengthBasePathUntil(k) / L;
        BasePathStateInformation state_information;
        state_information.restriction_name = name;
        state_information.base_path_index = k;
        state_information.position_on_base_path = d;
        //std::cout << "Restriction " << path_restriction.first << " state at index " << k << " position " << d << std::endl;

        auto base_state = path_restriction.second->getBaseStateAt(state_information.base_path_index);

        for(const auto& other_path_restriction : path_restrictions) {
          const auto other_name = other_path_restriction.first;
          if(other_name == name) {
            factor->getChild(other_name)->copyState(statesChild.at(other_name), base_state);
            //factor->getChild(other_name)->printState(statesChild.at(other_name));
            continue;
          }
          auto position = (finished_interpolating.at(other_name) ? other_path_restriction.second->getLengthBasePath() : 0.0);
          //std::cout << other_name << ":" << position << std::endl;
          other_path_restriction.second->interpolateBasePath(position, statesChild.at(other_name));
          //factor->getChild(other_name)->printState(statesChild.at(other_name));
        }

        auto state = factor->allocState();
        factor->lift(statesChild, state);
        states.push_back(state);
      }
      finished_interpolating.at(name) = true;
  }

  //////////////////////////////////////////////////////////////////////////////////
  // Print states
  //////////////////////////////////////////////////////////////////////////////////
  // for(const auto& state : states) {
  //   factor->printState(state);
  // }

  factor->freeChildStates(statesChild);
  return states;
}

std::optional<PathSectionPtr> parallelFibrationSectionSolver(const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const TreePtr& tree, const std::unordered_map<std::string, PathRestrictionPtr>& path_restrictions) {

  if(path_restrictions.empty()) {
    return std::nullopt;
  }

  if(std::dynamic_pointer_cast<ompl::multilevel::TaskSpaceMotionValidator>(factor->getMotionValidator()) != nullptr) {
    OMPL_WARN("Cannot compute section using Task Space constraints");
    return std::nullopt;
  }

  auto states = makeSectionPathL1(factor, path_restrictions);
  if(!checkMotion(factor, tree, tree->getRoot(), states)) {
    return std::nullopt;
  }
  return std::make_shared<PathSection>(states);
}

}
}
