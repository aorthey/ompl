/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/SubspaceFiberedProjection.h>
#include <ompl/base/SpaceInformation.h>

ompl::multilevel::Projection_FiberedSubspace::Projection_FiberedSubspace(
    const base::SpaceInformationPtr& siBundle, const base::SpaceInformationPtr& siBase, unsigned int subspace_index)
  : Projection_FiberedSubspace(siBundle->getStateSpace(), siBase->getStateSpace(), subspace_index) {} 

ompl::multilevel::Projection_FiberedSubspace::Projection_FiberedSubspace(ompl::base::StateSpacePtr bundleSpace, 
    ompl::base::StateSpacePtr baseSpace, unsigned int subspace_index)
  : FiberedProjection(bundleSpace, baseSpace)
{
  if(!bundleSpace->isCompound()) {
    throw std::domain_error("Not a compound space:" + bundleSpace->getName());
  }
  auto compound_space = bundleSpace->as<ompl::base::CompoundStateSpace>();
  if(subspace_index >= compound_space->getSubspaceCount()) {
    throw std::domain_error("Subspace index has to be valid, but " + std::to_string(subspace_index) 
       + " is larger than " + std::to_string(compound_space->getSubspaceCount()));
  }

  subspace_index_ = subspace_index;
  setType(PROJECTION_SUBSPACE);
}

void ompl::multilevel::Projection_FiberedSubspace::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const 
{
  auto cstate = xBundle->as<base::CompoundState>();
  getBase()->copyState(xBase, cstate->operator[](subspace_index_));
}

void ompl::multilevel::Projection_FiberedSubspace::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                              ompl::base::State *xBundle) const 
{
  inclusionMap(xBase, xBundle);

  auto bundle_state = xBundle->as<base::CompoundState>();
  auto fiber_state = xFiber->as<base::CompoundState>();

  for(const auto& subspace_indices : subspace_bundle_to_subspace_fiber_index_) {
    auto bundle_index = subspace_indices.first;
    auto fiber_index = subspace_indices.second;
    getBundle()->copyState(bundle_state->operator[](bundle_index), fiber_state->operator[](fiber_index));
  }
}

std::vector<size_t> ompl::multilevel::Projection_FiberedSubspace::getInclusionIndices() const 
{
  std::vector<size_t> indices;

  auto compound_space = getBundle()->as<ompl::base::CompoundStateSpace>();

  const std::vector<ompl::base::StateSpacePtr> subspaces = compound_space->getSubspaces();

  size_t current_dimension = 0;
  for(size_t k = 0; k < subspaces.size(); k++) {
    auto subspace = subspaces.at(k);
    if(k != subspace_index_) {
      for(size_t i = 0; i < subspace->getDimension(); i++) {
        indices.push_back(current_dimension + i);
      }
    }
    current_dimension = current_dimension + subspace->getDimension();
  }
  return indices;
}

void ompl::multilevel::Projection_FiberedSubspace::inclusionMap(const ompl::base::State *xBase, ompl::base::State *xBundle) const 
{
  auto cstate = xBundle->as<base::CompoundState>();
  getBase()->copyState(cstate->operator[](subspace_index_), xBase);
}

void ompl::multilevel::Projection_FiberedSubspace::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const 
{
  auto bundle_state = xBundle->as<base::CompoundState>();
  auto fiber_state = xFiber->as<base::CompoundState>();

  for(const auto& subspace_indices : subspace_bundle_to_subspace_fiber_index_) {
    auto bundle_index = subspace_indices.first;
    auto fiber_index = subspace_indices.second;
    getBundle()->copyState(fiber_state->operator[](fiber_index), bundle_state->operator[](bundle_index));
  }
}

ompl::base::StateSpacePtr ompl::multilevel::Projection_FiberedSubspace::computeFiberSpace()
{
  auto compound_space = getBundle()->as<ompl::base::CompoundStateSpace>();

  const std::vector<ompl::base::StateSpacePtr> subspaces = compound_space->getSubspaces();
  const std::vector<double> subspace_weights = compound_space->getSubspaceWeights();

  std::vector<ompl::base::StateSpacePtr> fiber_subspaces;
  std::vector<double> fiber_subspace_weights;

  size_t fiber_index = 0;

  for(size_t k = 0; k < subspaces.size(); k++) {
    if(k == subspace_index_) {
      continue;
    }
    subspace_bundle_to_subspace_fiber_index_.insert({k, fiber_index});
    fiber_subspaces.push_back(subspaces.at(k));
    fiber_subspace_weights.push_back(subspace_weights.at(k));
    fiber_index++;
  }

  auto fiber_space = std::make_shared<ompl::base::CompoundStateSpace>(fiber_subspaces, fiber_subspace_weights);
  return fiber_space;
}
