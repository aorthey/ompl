#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PATHRESTRICTIONHELPERS_
#define OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PATHRESTRICTIONHELPERS_

#include <ompl/base/State.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>

namespace ompl
{
    namespace multilevel
    {
        bool findFeasibleStateOnFiber(const PathRestrictionPtr& restriction, const HeadPtr& head, ompl::base::State *xBundle);
    }
}
#endif
