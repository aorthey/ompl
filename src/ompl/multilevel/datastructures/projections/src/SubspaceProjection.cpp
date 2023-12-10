/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>

ompl::multilevel::Projection_Subspace::Projection_Subspace(ompl::base::StateSpacePtr bundleSpace, 
    ompl::base::StateSpacePtr baseSpace, unsigned int subspace_index)
  : InclusionProjection(bundleSpace, baseSpace)
{
  if(!bundleSpace->isCompound()) {
    OMPL_ERROR("Need a compound space for a subspace projection.");
    throw "NotACompoundSpace";
  }
  auto compound_space = bundleSpace->as<base::CompoundStateSpace>();
  if(subspace_index >= compound_space->getSubspaceCount() || subspace_index < 0) {
    OMPL_ERROR("Subspace index has to be valid (%d is not in [%d, %d]).", subspace_index, 0, compound_space->getSubspaceCount());
    throw "InvalidSubspaceIndex";
  }
  immersion_space_ = compound_space->getSubspace(subspace_index);
  if(immersion_space_->getType() != baseSpace->getType()) {
    OMPL_ERROR("Subspace type (%d) has to be equal to base space type (%d).", immersion_space_->getType(), baseSpace->getType());
    throw "InvalidBaseSpaceType";
  }
  if(!(immersion_space_->covers(baseSpace) && baseSpace->covers(immersion_space_))) {
    OMPL_ERROR("Subspace has to be equal to base space.");
    throw "InvalidBaseSpace";
  }

  subspace_index_ = subspace_index;
}

void ompl::multilevel::Projection_Subspace::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const 
{
  auto cstate = xBundle->as<base::CompoundState>();
  getBase()->copyState(xBase, cstate->operator[](subspace_index_));
}

void ompl::multilevel::Projection_Subspace::lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const
{
  inclusionMap(xBase, xBundle);
}

void ompl::multilevel::Projection_Subspace::inclusionMap(const ompl::base::State *xBase, ompl::base::State *xBundle) const 
{
  auto cstate = xBundle->as<base::CompoundState>();
  immersion_space_->copyState(cstate->operator[](subspace_index_), xBase);

//   std::vector<size_t> indices = getInclusionIndices();
//   for(const auto& index : indices) 
//   {
//     double* value = getBundle()->getValueAddressAtIndex(xBundle, index);
//     const double* baseValue = getBase()->getValueAddressAtIndex(xBase, index);
//     *value = *baseValue;
//   }
}
