#include <ompl/multilevel/datastructures/pathrestriction/PathRestrictionHelpers.h>

#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

namespace ompl {
namespace multilevel {

const unsigned int kPathSectionMaxFiberSampling = 10;
bool findFeasibleStateOnFiber(const PathRestrictionPtr& restriction, const HeadPtr& head, ompl::base::State *xBundle) {

    auto projection = std::static_pointer_cast<FiberedProjection>(restriction->getProjection());
    auto base = projection->getBase();

    const double location_on_base_path = head->getLocationOnBasePath();

    if (projection->getCoDimension() == 0)
    {
        restriction->interpolateBasePath(location_on_base_path, xBundle);
        return true;
    }

    auto fiber = projection->getFiber();
    ompl::base::State *xBase = base->allocState();
    ompl::base::State *xBaseForward = base->allocState();
    ompl::base::State *xFiberTmp = fiber->allocState();

    restriction->interpolateBasePath(location_on_base_path, xBase);

    auto dStep = base->getLongestValidSegmentLength();
    restriction->interpolateBasePath(location_on_base_path + dStep, xBaseForward);

    const ompl::base::StateSamplerPtr samplerFiber = projection->getFiberSamplerPtr();

    unsigned int ctr = 0;

    while (ctr++ < kPathSectionMaxFiberSampling)
    {
        samplerFiber->sampleUniform(xFiberTmp);

        projection->lift(xBaseForward, xFiberTmp, xBundle);

        // New sample must be valid AND not reachable from last valid
        if (restriction->getSpaceInformation()->isValid(xBundle))
        {
            projection->lift(xBase, xFiberTmp, xBundle);
            fiber->freeState(xFiberTmp);
            base->freeState(xBaseForward);
            base->freeState(xBase);
            return true;
        }
    }
    fiber->freeState(xFiberTmp);
    base->freeState(xBaseForward);
    base->freeState(xBase);
    return false;
}

}
}
