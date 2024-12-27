/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PROJECTIONS_FIBEREDSUBSPACEPROJECTION__
#define OMPL_MULTILEVEL_DATASTRUCTURES_PROJECTIONS_FIBEREDSUBSPACEPROJECTION__
#include <ompl/base/State.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/multilevel/datastructures/ProjectionTypes.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

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
        /* \brief A bundle projection with an explicit fiber space representation */
        class Projection_FiberedSubspace : public FiberedProjection
        {
        public:
            Projection_FiberedSubspace(const base::SpaceInformationPtr& siBundle, const base::SpaceInformationPtr& siBase, unsigned int subspace_index);
            Projection_FiberedSubspace(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace, unsigned int subspace_index);

            virtual ~Projection_FiberedSubspace() = default;

            void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

            void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                              ompl::base::State *xBundle) const override;

            std::vector<size_t> getInclusionIndices() const override;

            void inclusionMap(const ompl::base::State *xBase, ompl::base::State *xBundle) const override;

            void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const override;

        protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;

        private:
            unsigned int subspace_index_;
            std::unordered_map<size_t, size_t> subspace_bundle_to_subspace_fiber_index_;
        };
    }
}

#endif

