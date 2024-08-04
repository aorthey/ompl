/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PARALLELFIBRATIONSECTIONSOLVER_
#define OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PARALLELFIBRATIONSECTIONSOLVER_

#include <ompl/base/State.h>
#include <ompl/util/ClassForward.h>

#include <unordered_map>
#include <optional>

namespace ompl {
namespace multilevel {

OMPL_CLASS_FORWARD(PathRestriction);
OMPL_CLASS_FORWARD(PathSection);
OMPL_CLASS_FORWARD(Tree);
OMPL_CLASS_FORWARD(FactoredSpaceInformation);

struct BasePathStateInformation {
  std::string restriction_name;
  size_t base_path_index;
  double position_on_base_path;
};

std::optional<PathSectionPtr> parallelFibrationSectionSolver(const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const TreePtr& tree, const std::unordered_map<std::string, PathRestrictionPtr>& path_restrictions);

}
}

#endif
