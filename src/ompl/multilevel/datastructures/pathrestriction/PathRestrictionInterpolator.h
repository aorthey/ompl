#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PATH_RESTRICTION_INTERPOLATOR_
#define OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PATH_RESTRICTION_INTERPOLATOR_

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include <optional>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Path */
        OMPL_CLASS_FORWARD(Path);
        /// @endcond
    }
    namespace geometric
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::PathGeometric */
        OMPL_CLASS_FORWARD(PathGeometric);
        /// @endcond
    }
    namespace multilevel
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Head);
        OMPL_CLASS_FORWARD(PathRestriction);
        OMPL_CLASS_FORWARD(PathSection);
        /// @endcond

        /** \brief Interpolate along restriction using L2 metric
          *  ---------------
          *            ____x
          *       ____/
          *   ___/
          *  x
          *  --------------- */
        PathSectionPtr interpolateL2(const PathRestrictionPtr&, const HeadPtr&);

        /** \brief Interpolate along restriction using L1 metric
          * (Fiber first)
          *   ---------------
          *    _____________x
          *   |
          *   |
          *   x
          *   --------------- */
        PathSectionPtr interpolateL1FiberFirst(const PathRestrictionPtr&, const HeadPtr&);

        /** \brief Interpolate along restriction using L1 metric (Fiber Last)
          *   ---------------
          *                 x
          *                 |
          *                 |
          *   x_____________|
          *   --------------- */
        PathSectionPtr interpolateL1FiberLast(const PathRestrictionPtr&, const HeadPtr&);
    }
}

#endif
