#include "ompl/base/goals/FactoredGoal.h"

ompl::base::FactoredGoal::FactoredGoal(const ompl::multilevel::FactoredSpaceInformationPtr& si, 
    const std::unordered_map<std::string, ompl::base::GoalSampleableRegionPtr>& goals)
  : ompl::base::GoalSampleableRegion(si), goals_(goals)
{
    type_ = FACTORED_GOAL;

    //Verify that goals are consistent with space information
    for(const auto& goal : goals) {
      const auto& name = goal.first;
      if(!si->hasChild(name)) {
        OMPL_ERROR("Could not find child with name %s", name.c_str());
        throw "UnknownChild";
      }
      tmp_goal_states_.insert({name, si->getChild(name)->allocState()});
    }
}

ompl::base::FactoredGoal::~FactoredGoal() {
    for(const auto& goal : goals_) {
      const auto& name = goal.first;
      auto it = tmp_goal_states_.find(name);
      if(it == tmp_goal_states_.end()) {
        continue;
      }
      goal.second->getSpaceInformation()->freeState(it->second);
    }
}

void ompl::base::FactoredGoal::sampleGoal(ompl::base::State *state) const {
  auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si_);
  for(const auto& goal : goals_) {
    const auto& name = goal.first;
    auto it = tmp_goal_states_.find(name);
    if(it == tmp_goal_states_.end()) {
      OMPL_ERROR("Could not find child with name %s for goal sampling.", name.c_str());
      throw "StateNotAllocated";
    }
    goal.second->sampleGoal(it->second);
  }
  factor->lift(tmp_goal_states_, state);
}

unsigned int ompl::base::FactoredGoal::maxSampleCount() const {
  unsigned int count = 0;
  for(const auto& goal : goals_) {
    count += goal.second->maxSampleCount();
  }
  return count;
}

double ompl::base::FactoredGoal::distanceGoal(const ompl::base::State *state) const {
  auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si_);
  double distance = 0.0f;
  factor->project(state, tmp_goal_states_);
  for(const auto& goal : goals_) {
    const auto& name = goal.first;
    auto it = tmp_goal_states_.find(name);
    if(it == tmp_goal_states_.end()) {
      OMPL_ERROR("Could not find child with name %s for goal distance.", name.c_str());
      throw "StateNotAllocated";
    }
    distance += goal.second->distanceGoal(it->second);
  }
  return distance;
}

std::optional<ompl::base::GoalSampleableRegionPtr> ompl::base::FactoredGoal::getFactorGoal(const std::string name) const {
  auto it = goals_.find(name);
  if(it == goals_.end()) {
    return std::nullopt;
  }
  return it->second;
}
