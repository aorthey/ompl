#include "ompl/multilevel/planners/RestrictionSampler.h"

#include "ompl/base/StateSpace.h"
#include "ompl/multilevel/datastructures/FactoredSpaceInformation.h"
#include "ompl/multilevel/planners/FactoredPlanner.h"

#include <algorithm>

using namespace ompl::multilevel;

RestrictionSampler::RestrictionSampler(const FactoredSpaceInformationPtr& factor, const std::vector<FactoredPlannerPtr>& children_planner) 
  : ompl::base::StateSampler(factor->getStateSpace().get()), factor_(factor)
{
    baseSampler_ = factor->allocStateSampler();

    if (!factor_->hasChildren())
    {
      OMPL_ERROR("Cannot create RestrictionSampler with a childless factor.");
      throw "NoChildrenError";
    }

    const auto& children = factor_->getChildren();

    if (children.size() != children_planner.size())
    {
      OMPL_ERROR("Number of children %d does not match children planner (%d)", children.size(), children_planner.size());
      throw "MismatchChildrenPlannerError";
    }

    size_t cumulative_dim = 0;
    for(const auto& child : children) 
    {
      cumulative_dim += child->getStateDimension();
    }

    if (children.size() > 1 && factor->getStateDimension() != cumulative_dim)
    {
      OMPL_ERROR("Children dimensions sum to %d which does not match dimension %d.", cumulative_dim, factor->getStateDimension());
      throw "MismatchChildrenPlannerDimensionsError";
    }

    for(const auto& child : children) 
    {
      const auto& name = child->getName();

      auto iterator = std::find_if(children_planner.begin(), children_planner.end(),
         [name](const auto& planner) {
          auto child = std::static_pointer_cast<FactoredSpaceInformation>(planner->getSpaceInformation());
          return child->getName() == name;
         }
       );

      if(iterator == children_planner.end()) 
      {
        OMPL_ERROR("Could not find %s in children planner.", name.c_str());
        throw "NonExistingChildrenPlannerError";
      }

      childrenPlanner_[name] = *iterator;
      statesChild_[name] = child->allocState();
      childNames_.push_back(name);
    }
}

RestrictionSampler::~RestrictionSampler() 
{
    for(const auto& child : statesChild_) 
    {
        const auto& name = child.first;
        auto state = child.second;
        if(!state) {
            continue;
        }
        if(childrenPlanner_.count(name) > 0) 
        {
            auto child_factor = std::static_pointer_cast<FactoredSpaceInformation>(childrenPlanner_.at(name)->getSpaceInformation());
            child_factor->freeState(state);
        }
    }
}

void RestrictionSampler::sampleUniform(base::State *state) {
    ////////////////////////////////////////////////////////////////////////////////
    //Obtain samples from all children
    ////////////////////////////////////////////////////////////////////////////////
    for(const auto& name : childNames_) {
        childrenPlanner_[name]->sampleFromDatastructure(statesChild_[name]);
    }

    ////////////////////////////////////////////////////////////////////////////////
    //Lift samples to factor space
    ////////////////////////////////////////////////////////////////////////////////
    factor_->lift(statesChild_, state);
    return;
}

void RestrictionSampler::sampleUniformNear(base::State *state, const base::State *near, double distance) {
    OMPL_WARN("sampleUniformNear is not using restriction sampling.");
    return baseSampler_->sampleUniformNear(state, near, distance);
}
void RestrictionSampler::sampleGaussian(base::State *state, const base::State *mean, double stdDev) {
    OMPL_WARN("sampleGaussian is not using restriction sampling.");
    return baseSampler_->sampleGaussian(state, mean, stdDev);
}
