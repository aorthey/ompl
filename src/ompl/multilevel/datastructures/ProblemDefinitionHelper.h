#ifndef OMPL_MULTILEVEL_PLANNERS_DATASTRUCTURES_PROBLEMDEFINITIONHELPER_
#define OMPL_MULTILEVEL_PLANNERS_DATASTRUCTURES_PROBLEMDEFINITIONHELPER_

#include <unordered_map>
#include <string>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>

namespace ompl {
    namespace multilevel {
        std::unordered_map<std::string, ompl::base::ProblemDefinitionPtr>
        computeProblemDefinitions(
            const FactoredSpaceInformationPtr& root_factor,
            const ompl::base::ProblemDefinitionPtr& root_pdef,
            double goal_threshold = 0.0);
    }
}

#endif
