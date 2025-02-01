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

#include <ompl/multilevel/datastructures/projections/XR3R2SO2ToXR3Projection.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Exception.h>

using namespace ompl::multilevel;
using namespace ompl::base;

XR3R2SO2ToXR3Projection::XR3R2SO2ToXR3Projection(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_R3R2SO2_R3);
    if(!BundleSpace->isCompound())
    {
        throw ompl::Exception("BundleSpace is not compound, but should be XR3R2SO2");
    }
    auto cspace = BundleSpace->as<CompoundStateSpace>();

    subspace_index_ = cspace->getSubspaceCount() - 1;

    auto ispace = cspace->getSubspace(subspace_index_);
    if(!ispace->isCompound())
    {
        throw ompl::Exception("Last subspace is not compound, but should be R3R2SO2");
    }
    auto cispace = ispace->as<CompoundStateSpace>();
    if(cispace->getSubspaceCount() != 3) {
        throw ompl::Exception("Last subspace is compound with "+std::to_string(cispace->getSubspaceCount())+" subspaces, but should be R3R2SO2");
    }
    if(!(cispace->getSubspace(0)->getType() == STATE_SPACE_REAL_VECTOR))
    {
        throw ompl::Exception("First subspace of last compound space is not R3");
    }
    if(!(cispace->getSubspace(1)->getType() == STATE_SPACE_REAL_VECTOR))
    {
        throw ompl::Exception("Second subspace of last compound space is not R2");
    }
    if(!(cispace->getSubspace(2)->getType() == STATE_SPACE_SO2))
    {
        throw ompl::Exception("Third subspace of last compound space is not SO2");
    }
}

void XR3R2SO2ToXR3Projection::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    //auto cspace = xBundle->as<base::CompoundState>();
    //auto cspace->getSubspace(subspace_index_);
    const auto *xBundle_R2 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::RealVectorStateSpace::StateType>(1);
    const auto *xBundle_SO2 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::SO2StateSpace::StateType>(2);
    //const auto *xBundle_SO2 = xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(2);

    auto *xFiber_R2 = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    auto *xFiber_SO2 = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(1);

    xFiber_R2->values[0] = xBundle_R2->values[0];
    xFiber_R2->values[1] = xBundle_R2->values[1];
    xFiber_SO2->value = xBundle_SO2->value;
}

void XR3R2SO2ToXR3Projection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    //copy the first subspaces
    for(size_t k = 0; k < subspace_index_; k++) {
      getBase()->as<base::CompoundStateSpace>()->getSubspace(k)->copyState(
          xBase->as<base::CompoundState>()->as<base::CompoundState>(k), xBundle->as<base::CompoundState>()->as<base::CompoundState>(k));
    }

    //copy the last subspace of R3
    const auto& xBundle_R3 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::RealVectorStateSpace::StateType>(0)->values;
    auto& xBase_R3 = xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(subspace_index_)->values;
    xBase_R3[0] = xBundle_R3[0];
    xBase_R3[1] = xBundle_R3[1];
    xBase_R3[2] = xBundle_R3[2];
}

void XR3R2SO2ToXR3Projection::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                               ompl::base::State *xBundle) const
{
    for(size_t k = 0; k < subspace_index_; k++) {
      getBase()->as<base::CompoundStateSpace>()->getSubspace(k)->copyState(
          xBundle->as<base::CompoundState>()->as<base::CompoundState>(k), xBase->as<base::CompoundState>()->as<base::CompoundState>(k));
    }

    const auto& xBase_R3 = xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(subspace_index_)->values;
    const auto& xFiber_R2 = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0)->values;
    const auto& xFiber_SO2 = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(1)->value;

    auto& xBundle_R3 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::RealVectorStateSpace::StateType>(0)->values;
    auto& xBundle_R2 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::RealVectorStateSpace::StateType>(1)->values;
    auto& xBundle_SO2 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::SO2StateSpace::StateType>(2)->value;

    xBundle_R3[0] = xBase_R3[0];
    xBundle_R3[1] = xBase_R3[1];
    xBundle_R3[2] = xBase_R3[2];

    xBundle_R2[0] = xFiber_R2[0];
    xBundle_R2[1] = xFiber_R2[1];

    xBundle_SO2 = xFiber_SO2;
}

ompl::base::StateSpacePtr XR3R2SO2ToXR3Projection::computeFiberSpace()
{
    ompl::base::StateSpacePtr R2(std::make_shared<RealVectorStateSpace>(2));
    ompl::base::StateSpacePtr SO2(std::make_shared<SO2StateSpace>());
    auto bundleR2 = getBundle()->as<base::CompoundStateSpace>()->as<base::CompoundStateSpace>(subspace_index_)->as<RealVectorStateSpace>(1);
    R2->as<RealVectorStateSpace>()->setBounds(bundleR2->getBounds());
    return R2 + SO2;
}
