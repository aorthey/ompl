/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PROJECTIONS_SUBSPACEPROJECTION__
#define OMPL_MULTILEVEL_DATASTRUCTURES_PROJECTIONS_SUBSPACEPROJECTION__
#include <ompl/base/State.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/multilevel/datastructures/ProjectionTypes.h>
#include <ompl/multilevel/datastructures/projections/InclusionProjection.h>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::SpaceInformation */
        OMPL_CLASS_FORWARD(SpaceInformation);
        /** \brief Forward declaration of ompl::base::StateSpace */
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond
    }
    namespace multilevel
    {
        /* \brief A bundle projection without an explicit fiber space representation */
        class SubspaceProjection : public InclusionProjection
        {
        public:
            SubspaceProjection(const base::SpaceInformationPtr& siBundle, const base::SpaceInformationPtr& siBase, unsigned int subspace_index);
            SubspaceProjection(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace, unsigned int subspace_index);

            virtual ~SubspaceProjection() = default;

            void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;
            void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const override;
            void inclusionMap(const ompl::base::State *xBase, ompl::base::State *xBundle) const override;
        private:
            base::StateSpacePtr immersion_space_;
            unsigned int subspace_index_;
        };
    }
}

#endif

