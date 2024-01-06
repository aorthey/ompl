#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_TASKSPACEMOTIONVALIDATOR_
#define OMPL_MULTILEVEL_DATASTRUCTURES_TASKSPACEMOTIONVALIDATOR_

#include <ompl/base/State.h>
#include <ompl/base/DiscreteMotionValidator.h>

namespace ompl
{
    namespace multilevel
    {
        class TaskSpaceMotionValidator : public ompl::base::DiscreteMotionValidator
        {
        public:
            TaskSpaceMotionValidator(ompl::base::SpaceInformation *si) : ompl::base::DiscreteMotionValidator(si) {};
            TaskSpaceMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::DiscreteMotionValidator(si) {};

            /* \brief Propagate states forward until invalid or reached s2 */
            virtual std::vector<ompl::base::State*> propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const = 0;
        };

    }
}
#endif
