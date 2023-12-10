/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PROJECTIONS_INCLUSIONPROJECTION__
#define OMPL_MULTILEVEL_DATASTRUCTURES_PROJECTIONS_INCLUSIONPROJECTION__
#include <ompl/base/State.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/multilevel/datastructures/ProjectionTypes.h>
#include <ompl/multilevel/datastructures/Projection.h>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateSpace */
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond
    }
    namespace multilevel
    {
        /* \brief A bundle projection with an explicit fiber space representation
         * which can be explicitly sampled to lift states */
        class InclusionProjection : public Projection
        {
        public:
            InclusionProjection(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace);

            virtual ~InclusionProjection() = default;

            /* \brief Compute all indices which contain values which are
             * kept during projection */
            virtual std::vector<size_t> getInclusionIndices() const;

            /* \brief Map base state into bundle space, but keep non-inclusion
             * indices constant (xBundle is only changed partially) */
            virtual void inclusionMap(const ompl::base::State *xBase, ompl::base::State *xBundle) const;
        };
    }
}

#endif

