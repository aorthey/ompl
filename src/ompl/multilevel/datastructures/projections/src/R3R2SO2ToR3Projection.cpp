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

#include <ompl/multilevel/datastructures/projections/R3R2SO2ToR3Projection.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl::multilevel;
using namespace ompl::base;

R3R2SO2ToR3Projection::R3R2SO2ToR3Projection(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_R3R2SO2_R3);
}

void R3R2SO2ToR3Projection::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_R2 = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
    const auto *xBundle_SO2 = xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(2);

    auto *xFiber_R2 = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    auto *xFiber_SO2 = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(1);

    xFiber_R2->values[0] = xBundle_R2->values[0];
    xFiber_R2->values[1] = xBundle_R2->values[1];
    xFiber_SO2->value = xBundle_SO2->value;
}

void R3R2SO2ToR3Projection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_R3 = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    getBase()->copyState(xBase, xBundle_R3);
}

void R3R2SO2ToR3Projection::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                               ompl::base::State *xBundle) const
{
    auto *xBundle_R3 = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    auto *xBundle_R2 = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
    auto *xBundle_SO2 = xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(2);

    const auto *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();
    const auto *xFiber_R2 = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    const auto *xFiber_SO2 = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(1);

    xBundle_R3->values[0] = xBase_R3->values[0];
    xBundle_R3->values[1] = xBase_R3->values[1];
    xBundle_R3->values[2] = xBase_R3->values[2];

    xBundle_R2->values[0] = xFiber_R2->values[0];
    xBundle_R2->values[1] = xFiber_R2->values[1];

    xBundle_SO2->value = xFiber_SO2->value;
}

ompl::base::StateSpacePtr R3R2SO2ToR3Projection::computeFiberSpace()
{
    ompl::base::StateSpacePtr R2(std::make_shared<RealVectorStateSpace>(2));
    ompl::base::StateSpacePtr SO2(std::make_shared<SO2StateSpace>());
    auto bundleR2 = getBundle()->as<CompoundStateSpace>()->as<RealVectorStateSpace>(1);
    R2->as<RealVectorStateSpace>()->setBounds(bundleR2->getBounds());
    return R2 + SO2;
}
