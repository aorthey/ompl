#ifndef TESTS_MULTILEVEL_FACTORIZATION_COMMON_
#define TESTS_MULTILEVEL_FACTORIZATION_COMMON_
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/RN_RM.h>
#include <ompl/util/Console.h>

using namespace ompl::base;
using namespace ompl::multilevel;

const unsigned int kDefaultNumberIterations = 500;

ompl::base::StateSpacePtr CreateCubeStateSpace(size_t dim) {
  ompl::base::StateSpacePtr space(new RealVectorStateSpace(dim));
  ompl::base::RealVectorBounds bounds_space(dim);
  bounds_space.setLow(0);
  bounds_space.setHigh(+1);
  space->as<RealVectorStateSpace>()->setBounds(bounds_space);
  return space;
}

ompl::multilevel::FactoredSpaceInformationPtr CreateCubeSpaceInformation(size_t dim, std::string name) {
  auto space = CreateCubeStateSpace(dim);
  space->setName(name);
  return std::make_shared<FactoredSpaceInformation>(space);
}

ompl::base::State* AllocState(const ompl::base::SpaceInformationPtr& si, const std::vector<float>& vector) {
  auto state = si->allocState();
  const auto *state_RN = state->as<ompl::base::RealVectorStateSpace::StateType>();
  for(size_t k = 0; k < si->getStateDimension(); k++) {
    state_RN->values[k] = vector.at(k);
  }
  return state;
}

ScopedState<> CreateState(const ompl::base::StateSpacePtr& space, const float value, const float step_size = 0.0f) {
  ScopedState<> state(space);
  for(size_t k = 0; k < space->getDimension(); k++) {
    state[k] = value + k * step_size;
  }
  return state;
}

#endif // TESTS_MULTILEVEL_FACTORIZATION_COMMON_
