#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
#define OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_

#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rlrt/RLRT.h"

#include <optional>

namespace ompl {
    namespace multilevel {

        OMPL_CLASS_FORWARD(FactoredPlanner);
        OMPL_CLASS_FORWARD(FactoredSpaceInformation);

        typedef ompl::geometric::RRT BaseTypePlanner;

        class FactoredPlanner : public BaseTypePlanner {
          public:
            /** \brief Constructor */
            FactoredPlanner(const FactoredSpaceInformationPtr& si, const std::vector<FactoredPlannerPtr>& children_planner = {});

            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            void sampleFromDatastructure(ompl::base::State* state);
            void sampleFromPath(const std::vector<base::State *>& path_states, ompl::base::State* state);

            void setSeed(size_t seed);
        };
    }
}

#endif // OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
