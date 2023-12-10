/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/InclusionProjection.h>

#include <boost/iterator/counting_iterator.hpp>

ompl::multilevel::InclusionProjection::InclusionProjection(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : ompl::multilevel::Projection(bundleSpace, baseSpace)
{
}

std::vector<size_t> ompl::multilevel::InclusionProjection::getInclusionIndices() const 
{
    return std::vector<size_t>(boost::counting_iterator<size_t>(0), boost::counting_iterator<size_t>(getBaseDimension()));
}

void ompl::multilevel::InclusionProjection::inclusionMap(const ompl::base::State *xBase, ompl::base::State *xBundle) const 
{
  std::vector<size_t> indices = getInclusionIndices();
  for(const auto& index : indices) 
  {
    double* value = getBundle()->getValueAddressAtIndex(xBundle, index);
    const double* baseValue = getBase()->getValueAddressAtIndex(xBase, index);
    *value = *baseValue;
  }
}
