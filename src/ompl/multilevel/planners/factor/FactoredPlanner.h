#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
#define OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_

#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTtask.h"
#include "ompl/geometric/planners/rlrt/RLRT.h"

#include <optional>

namespace ompl {
    namespace multilevel {

        OMPL_CLASS_FORWARD(FactoredPlanner);
        OMPL_CLASS_FORWARD(FactoredSpaceInformation);

        const double kPathRestrictionSamplingBias = 0.2;
        const double kPathRestrictionSurroundingBias = 0.1;
        const double kSamplingPerturbationValue  = 0.05;
        typedef ompl::geometric::RRTtask BaseTypePlanner;

        class FactoredPlanner : public BaseTypePlanner {
          public:
            /** \brief Constructor */
            FactoredPlanner(const FactoredSpaceInformationPtr& si, const std::vector<FactoredPlannerPtr>& children_planner = {});

            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            void sampleFromDatastructure(ompl::base::State* state);
            void sampleFromPath(const std::vector<base::State *>& path_states, ompl::base::State* state);

            size_t getNumberOfSamples() const;

            void setSeed(size_t seed);
        };
    }
}

#endif // OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
