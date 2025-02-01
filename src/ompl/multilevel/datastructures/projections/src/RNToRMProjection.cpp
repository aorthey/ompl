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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ranges>
#include <numeric>
#include <boost/iterator/counting_iterator.hpp>

#include <ompl/multilevel/datastructures/projections/RNToRMProjection.h>

using namespace ompl::multilevel;

RNToRMProjection::RNToRMProjection(const base::SpaceInformationPtr& bundleSi, const base::SpaceInformationPtr& baseSi)
  : RNToRMProjection(bundleSi->getStateSpace(), baseSi->getStateSpace()) 
{
}

RNToRMProjection::RNToRMProjection(const base::SpaceInformationPtr& bundleSi, const base::SpaceInformationPtr& baseSi, std::vector<size_t> projected_dimensions)
  : RNToRMProjection(bundleSi->getStateSpace(), baseSi->getStateSpace(), projected_dimensions) 
{
}
RNToRMProjection::RNToRMProjection(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : RNToRMProjection(BundleSpace, BaseSpace, 
    std::vector<size_t>(boost::counting_iterator<size_t>(0), boost::counting_iterator<size_t>(BaseSpace->getDimension())))
{
}
RNToRMProjection::RNToRMProjection(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace, std::vector<size_t> projected_dimensions)
  : BaseT(BundleSpace, BaseSpace), projected_dimensions_(projected_dimensions)
{
    if(projected_dimensions.size() != BaseSpace->getDimension()) {
      OMPL_ERROR("For projection from %s to %s, you need to specify how each dimension \n \
          is mapped to the bundle space. However, you specified %d dimensions instead of %d.", 
       BundleSpace->getName().c_str(), BaseSpace->getName().c_str(),
       projected_dimensions.size(), BaseSpace->getDimension());
      throw std::out_of_range("InvalidDimensionError");
    }
    for(const auto dimension : projected_dimensions) {
      if(dimension > BundleSpace->getDimension() - 1) {
        OMPL_ERROR("For projection from %s to %s, you specified a dimension mapping to value %d, \n \
            which is outside the dimensions of the bundle space (%d).",
         BundleSpace->getName().c_str(), BaseSpace->getName().c_str(),
         dimension, BundleSpace->getDimension());
        throw std::out_of_range("InvalidDimensionError");
      }
    }

    size_t ctr_base = 0;
    size_t ctr_fiber = 0;
    for (unsigned int k = 0; k < getDimension(); k++) 
    {
      if(isProjectedDimension(k)) {
        map_projected_dimension_to_base_.insert({k, ctr_base});
        ctr_base++;
      } else {
        non_projected_dimensions_.push_back(k);
        map_non_projected_dimension_to_fiber_.insert({k, ctr_fiber});
        ctr_fiber++;
      }
    }
    auto total_size = map_projected_dimension_to_base_.size() + map_non_projected_dimension_to_fiber_.size();
    if(total_size != getDimension()) {
      OMPL_ERROR("Dimension error: Base has %d, fiber has %d, but dimension is %d.", map_projected_dimension_to_base_.size(),
          map_non_projected_dimension_to_fiber_.size(), getDimension());
      throw std::out_of_range("InvalidDimensionError");
    }
    setType(PROJECTION_RN_RM);
}

std::vector<size_t> RNToRMProjection::getInclusionIndices() const 
{
  auto index_selector = [](const auto& pair){return pair.first;};
  std::vector<size_t> indices(map_projected_dimension_to_base_.size());
  transform(map_projected_dimension_to_base_.begin(), map_projected_dimension_to_base_.end(), indices.begin(), index_selector);
  return indices;
}

void RNToRMProjection::inclusionMap(const ompl::base::State *xBase, ompl::base::State *xBundle) const 
{
  std::vector<size_t> indices = getInclusionIndices();

  for(const auto& bundle_base_indices : map_projected_dimension_to_base_) 
  {
    const auto& indexBundle = bundle_base_indices.first;
    const auto& indexBase = bundle_base_indices.second;

    double* value = getBundle()->getValueAddressAtIndex(xBundle, indexBundle);
    const double* baseValue = getBase()->getValueAddressAtIndex(xBase, indexBase);
    *value = *baseValue;
  }
}


bool RNToRMProjection::isProjectedDimension(size_t input) const 
{
    for(const auto& dim : projected_dimensions_) {
      if(dim == input) {
        return true;
      }
    }
    return false;
}

void RNToRMProjection::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_RN = xBundle->as<base::RealVectorStateSpace::StateType>();

    auto *xFiber_RM = xFiber->as<base::RealVectorStateSpace::StateType>();

    for(const auto& dim : non_projected_dimensions_) 
    {
      const auto& fdim = map_non_projected_dimension_to_fiber_.at(dim);
      xFiber_RM->values[fdim] = xBundle_RN->values[dim];
    }
}

void RNToRMProjection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_RN = xBundle->as<base::RealVectorStateSpace::StateType>();
    auto *xBase_RM = xBase->as<base::RealVectorStateSpace::StateType>();

    for(const auto& dim : projected_dimensions_) 
    {
      const auto& bdim = map_projected_dimension_to_base_.at(dim);
      xBase_RM->values[bdim] = xBundle_RN->values[dim];
    }
}

void RNToRMProjection::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                            ompl::base::State *xBundle) const
{
    auto *xBundle_RN = xBundle->as<base::RealVectorStateSpace::StateType>();
    const auto *xBase_RM = xBase->as<base::RealVectorStateSpace::StateType>();
    const auto *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    for(const auto& dim : projected_dimensions_)
    {
        const auto& bdim = map_projected_dimension_to_base_.at(dim);
        xBundle_RN->values[dim] = xBase_RM->values[bdim];
    }
    for(const auto& dim : non_projected_dimensions_)
    {
        const auto& fdim = map_non_projected_dimension_to_fiber_.at(dim);
        xBundle_RN->values[dim] = xFiber_RJ->values[fdim];
    }
}

ompl::base::StateSpacePtr RNToRMProjection::computeFiberSpace()
{
    unsigned int N = non_projected_dimensions_.size();
    base::StateSpacePtr FiberSpace = std::make_shared<base::RealVectorStateSpace>(N);
    base::RealVectorBounds Bundle_bounds =
        std::static_pointer_cast<base::RealVectorStateSpace>(getBundle())->getBounds();

    std::vector<double> low;
    low.resize(N);
    std::vector<double> high;
    high.resize(N);
    base::RealVectorBounds Fiber_bounds(N);
    for(const auto& dim : non_projected_dimensions_)
    {
        const auto& fdim = map_non_projected_dimension_to_fiber_.at(dim);
        Fiber_bounds.setLow(fdim, Bundle_bounds.low.at(dim));
        Fiber_bounds.setHigh(fdim, Bundle_bounds.high.at(dim));
    }
    std::static_pointer_cast<base::RealVectorStateSpace>(FiberSpace)->setBounds(Fiber_bounds);
    return FiberSpace;
}
