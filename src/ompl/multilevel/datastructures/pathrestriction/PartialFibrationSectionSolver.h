/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PARTIALFIBRATIONSECTIONSOLVER_
#define OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PARTIALFIBRATIONSECTIONSOLVER_

#include <ompl/base/State.h>
#include <ompl/util/ClassForward.h>

#include <optional>

namespace ompl {
namespace multilevel {

OMPL_CLASS_FORWARD(PathRestriction);
OMPL_CLASS_FORWARD(PathSection);
OMPL_CLASS_FORWARD(Tree);
OMPL_CLASS_FORWARD(FactoredSpaceInformation);

std::optional<PathSectionPtr> partialFibrationSectionSolver(const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const TreePtr& tree, const PathRestrictionPtr& path_restrictions, const base::State* targetState);

}
}

#endif
