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

#include <ompl/multilevel/datastructures/projections/XSE2RNToXR2Projection.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

XSE2RNToXR2Projection::XSE2RNToXR2Projection(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_SE2RN_R2);
    if(!BundleSpace->isCompound())
    {
        throw ompl::Exception("BundleSpace is not compound, but should be XSE2RN");
    }
    auto cspace = BundleSpace->as<ompl::base::CompoundStateSpace>();

    subspace_index_ = cspace->getSubspaceCount() - 1;

    auto ispace = cspace->getSubspace(subspace_index_);
    if(!ispace->isCompound())
    {
        throw ompl::Exception("Last subspace is not compound, but should be SE2RN");
    }
    auto cispace = ispace->as<ompl::base::CompoundStateSpace>();
    if(cispace->getSubspaceCount() != 2) {
        throw ompl::Exception("Last subspace is compound with "+std::to_string(cispace->getSubspaceCount())+" subspaces, but should be SE2RN");
    }
    if(!(cispace->getSubspace(0)->getType() == ompl::base::STATE_SPACE_SE2))
    {
        throw ompl::Exception("Third subspace of last compound space is not SE2");
    }
    if(!(cispace->getSubspace(1)->getType() == ompl::base::STATE_SPACE_REAL_VECTOR))
    {
        throw ompl::Exception("First subspace of last compound space is not RN");
    }
}

void XSE2RNToXR2Projection::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_SE2 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::SE2StateSpace::StateType>(0);
    const auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::RealVectorStateSpace::StateType>(1);

    auto *xFiber_SO2 = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    auto *xFiber_RN = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xFiber_SO2->value = xBundle_SE2->getYaw();
    for (unsigned int k = 0; k < getFiberDimension() - 1; k++)
    {
        xFiber_RN->values[k] = xBundle_RN->values[k];
    }
}

void XSE2RNToXR2Projection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    //copy the first subspaces
    for(size_t k = 0; k < subspace_index_; k++) {
      getBase()->as<base::CompoundStateSpace>()->getSubspace(k)->copyState(
          xBase->as<base::CompoundState>()->as<base::CompoundState>(k), xBundle->as<base::CompoundState>()->as<base::CompoundState>(k));
    }

    //copy the R2 from last subspace
    const auto *xBundle_SE2 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::SE2StateSpace::StateType>(0);

    auto *xBase_R2 = xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(subspace_index_);

    xBase_R2->values[0] = xBundle_SE2->getX();
    xBase_R2->values[1] = xBundle_SE2->getY();
}

void XSE2RNToXR2Projection::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                               ompl::base::State *xBundle) const
{
    for(size_t k = 0; k < subspace_index_; k++) {
      getBase()->as<base::CompoundStateSpace>()->getSubspace(k)->copyState(
          xBundle->as<base::CompoundState>()->as<base::CompoundState>(k), xBase->as<base::CompoundState>()->as<base::CompoundState>(k));
    }

    auto *xBundle_SE2 = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::SE2StateSpace::StateType>(0);
    auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::CompoundState>(subspace_index_)->as<base::RealVectorStateSpace::StateType>(1);

    const auto *xBase_R2 = xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(subspace_index_);

    const auto *xFiber_SO2 = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    const auto *xFiber_RN = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBundle_SE2->setX(xBase_R2->values[0]);
    xBundle_SE2->setY(xBase_R2->values[1]);
    xBundle_SE2->setYaw(xFiber_SO2->value);

    for (unsigned int k = 0; k < getFiberDimension() - 1; k++)
    {
        xBundle_RN->values[k] = xFiber_RN->values[k];
    }
}

ompl::base::StateSpacePtr XSE2RNToXR2Projection::computeFiberSpace()
{

    base::CompoundStateSpace *compound = getBundle()->as<base::CompoundStateSpace>()->getSubspace(subspace_index_)->as<ompl::base::CompoundStateSpace>();
    auto space_RN = compound->getSubspace(1)->as<base::RealVectorStateSpace>();
    auto N = space_RN->getDimension();

    base::StateSpacePtr SO2(new base::SO2StateSpace());
    base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
    RN->as<base::RealVectorStateSpace>()->setBounds(space_RN->getBounds());
    return SO2 + RN;
}
