/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
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

#include <ompl/multilevel/datastructures/projections/TimeBasedProjection.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl::multilevel;

////////////////////////////////////////////////////////////////////////////////
// Internal projections
////////////////////////////////////////////////////////////////////////////////
unsigned int InternalTimeBasedCompoundProjection::GetBaseIndexFromBundleIndex(unsigned int bundle_index) const {
  return (bundle_index < time_component_index_ ? bundle_index : bundle_index - 1);
}
void InternalTimeBasedCompoundProjection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const {
  const auto cstateBundle = xBundle->as<base::CompoundState>();
  auto cstateBase = xBase->as<base::CompoundState>();

  auto compoundSpaceBundle = bundleSpace->as<base::CompoundStateSpace>();

  for(unsigned int bundle_index = 0; bundle_index < compoundSpaceBundle->getSubspaceCount(); bundle_index++) {
    if(bundle_index == time_component_index_) {
      continue;
    }
    auto subspace = compoundSpaceBundle->getSubspace(bundle_index);
    auto base_index = GetBaseIndexFromBundleIndex(bundle_index);
    subspace->copyState(cstateBase->operator[](base_index), cstateBundle->operator[](bundle_index));
  }
}
void InternalTimeBasedCompoundProjection::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                  ompl::base::State *xBundle) const {
  const auto cstateBase = xBase->as<base::CompoundState>();
  const auto *xFiber_Time = xFiber->as<base::TimeStateSpace::StateType>();
  auto cstateBundle = xBundle->as<base::CompoundState>();

  auto compoundSpaceBundle = bundleSpace->as<base::CompoundStateSpace>();

  for(unsigned int bundle_index = 0; bundle_index < compoundSpaceBundle->getSubspaceCount(); bundle_index++) {
    if(bundle_index == time_component_index_) {
      cstateBundle->components[time_component_index_]->as<base::TimeStateSpace::StateType>()->position = xFiber_Time->position;
      continue;
    }
    auto subspace = compoundSpaceBundle->getSubspace(bundle_index);
    auto base_index = GetBaseIndexFromBundleIndex(bundle_index);
    subspace->copyState(cstateBundle->operator[](bundle_index), cstateBase->operator[](base_index));
  }
}
void InternalTimeBasedCompoundProjection::verify() {
 auto compoundSpaceBundle = bundleSpace->as<base::CompoundStateSpace>();
 auto compoundSpaceBase = baseSpace->as<base::CompoundStateSpace>();

 if(compoundSpaceBundle->getSubspaceCount() - 1 != compoundSpaceBase->getSubspaceCount()) {
   OMPL_ERROR("Base space should have %d components, but has %d.", compoundSpaceBundle->getSubspaceCount() - 1, compoundSpaceBase->getSubspaceCount());
   throw "InvalidComponents";
 }

 //Verify that spaces match up in type and dimension
 for(unsigned int bundle_index = 0; bundle_index < compoundSpaceBundle->getSubspaceCount(); bundle_index++) {
   if(bundle_index == time_component_index_) {
     continue;
   }
   auto bundleSubspace = compoundSpaceBundle->getSubspace(bundle_index);
   auto base_index = GetBaseIndexFromBundleIndex(bundle_index);
   auto baseSubspace = compoundSpaceBase->getSubspace(base_index);
   if(baseSubspace->getType() != bundleSubspace->getType()) {
     OMPL_ERROR("Subspaces type %d does not match %d.", baseSubspace->getType(), bundleSubspace->getType());
     throw "SubspacesAreNotEquivalent";
   } 
   if(baseSubspace->getDimension() != bundleSubspace->getDimension()) {
     OMPL_ERROR("Subspaces dimension %d does not match %d.", baseSubspace->getDimension(), bundleSubspace->getDimension());
     throw "SubspacesAreNotEquivalent";
   } 
 }
}
////////////////////////////////////////////////////////////////////////////////
// Internal projection, non compound
////////////////////////////////////////////////////////////////////////////////
void InternalTimeBasedNonCompoundProjection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const {
  const auto cstateBundle = xBundle->as<base::CompoundState>();
  auto compoundSpaceBundle = bundleSpace->as<base::CompoundStateSpace>();

  for(unsigned int bundle_index = 0; bundle_index < compoundSpaceBundle->getSubspaceCount(); bundle_index++) {
    if(bundle_index == time_component_index_) {
      continue;
    }
    auto subspace = compoundSpaceBundle->getSubspace(bundle_index);
    subspace->copyState(xBase, cstateBundle->operator[](bundle_index));
  }
}
void InternalTimeBasedNonCompoundProjection::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                  ompl::base::State *xBundle) const {
  const auto *xFiber_Time = xFiber->as<base::TimeStateSpace::StateType>();
  auto cstateBundle = xBundle->as<base::CompoundState>();

  auto compoundSpaceBundle = bundleSpace->as<base::CompoundStateSpace>();

  for(unsigned int bundle_index = 0; bundle_index < compoundSpaceBundle->getSubspaceCount(); bundle_index++) {
    if(bundle_index == time_component_index_) {
      cstateBundle->components[time_component_index_]->as<base::TimeStateSpace::StateType>()->position = xFiber_Time->position;
      continue;
    }
    auto subspace = compoundSpaceBundle->getSubspace(bundle_index);
    subspace->copyState(cstateBundle->operator[](bundle_index), xBase);
  }
}
void InternalTimeBasedNonCompoundProjection::verify() {
  auto compoundSpaceBundle = bundleSpace->as<base::CompoundStateSpace>();
  if(compoundSpaceBundle->getSubspaceCount() - 1 != 1) {
    OMPL_ERROR("Bundle space should have 2 components, but has %d.", compoundSpaceBundle->getSubspaceCount());
    throw "InvalidNumberOfComponents";
  }
  for(unsigned int bundle_index = 0; bundle_index < compoundSpaceBundle->getSubspaceCount(); bundle_index++) {
    if(bundle_index == time_component_index_) {
      continue;
    }
    auto bundleSubspace = compoundSpaceBundle->getSubspace(bundle_index);
    if(baseSpace->getType() != bundleSubspace->getType()) {
      OMPL_ERROR("Base space type %d does not match %d.", baseSpace->getType(), bundleSubspace->getType());
      throw "SubspacesAreNotEquivalent";
    } 
    if(baseSpace->getDimension() != bundleSubspace->getDimension()) {
      OMPL_ERROR("Base space dimension %d does not match %d.", baseSpace->getDimension(), bundleSubspace->getDimension());
      throw "SubspacesAreNotEquivalent";
    } 
  }
}

////////////////////////////////////////////////////////////////////////////////
// Global projection
////////////////////////////////////////////////////////////////////////////////
Projection_TimeBased::Projection_TimeBased(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : BaseT(bundleSpace, baseSpace)
{
    setType(PROJECTION_TIME_BASED);

    if(!bundleSpace->isCompound()) {
      OMPL_ERROR("Need a compound space for a time-based projection.");
      throw "NotACompoundSpace";
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Find subspace index of time component in bundle
    ////////////////////////////////////////////////////////////////////////////////
    auto compoundSpaceBundle = bundleSpace->as<base::CompoundStateSpace>();
    bool found_time_space = false;
    int time_component_index = 0;
    for(unsigned int bundle_index = 0; bundle_index < compoundSpaceBundle->getSubspaceCount(); bundle_index++) {
      auto subspace = compoundSpaceBundle->getSubspace(bundle_index);
      if(subspace->getType() != base::StateSpaceType::STATE_SPACE_TIME) {
        continue;
      }
      found_time_space = true;
      time_component_index = bundle_index;
      break;
    }
    if(!found_time_space) {
      OMPL_ERROR("Need a time subspace for a time-based projection.");
      throw "DidNotFindTimeSpace";
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Verify that base space has correct subspace structure
    ////////////////////////////////////////////////////////////////////////////////
    if(baseSpace->isCompound()) {
      internal_projection_ = std::make_shared<InternalTimeBasedCompoundProjection>(bundleSpace, baseSpace);
    } else {
      internal_projection_ = std::make_shared<InternalTimeBasedNonCompoundProjection>(bundleSpace, baseSpace);
    }
    internal_projection_->time_component_index_ = time_component_index;
    internal_projection_->verify();
}

void Projection_TimeBased::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
  const auto *cstate = static_cast<const base::CompoundState *>(xBundle);
  auto *xFiber_Time = xFiber->as<base::TimeStateSpace::StateType>();
  xFiber_Time->position = cstate->components[internal_projection_->time_component_index_]->as<base::TimeStateSpace::StateType>()->position;
}

void Projection_TimeBased::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
  internal_projection_->project(xBundle, xBase);
}

void Projection_TimeBased::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                               ompl::base::State *xBundle) const
{
  internal_projection_->lift(xBase, xFiber, xBundle);
}

ompl::base::StateSpacePtr Projection_TimeBased::computeFiberSpace()
{
  return getBundle()->as<base::CompoundStateSpace>()->getSubspace(internal_projection_->time_component_index_);
}
