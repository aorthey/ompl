/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_SECTIONNODE_
#define OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_SECTIONNODE_

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

namespace ompl
{
    namespace multilevel {

        struct SectionNode {
          SectionNode() = default;
          SectionNode(const base::SpaceInformationPtr &si) : state(si->allocState()) {};
          SectionNode(const base::SpaceInformationPtr &si, const base::State* input_state) : state(si->allocState()) {
            si->copyState(state, input_state);
          }
          ~SectionNode() = default;

          base::State* state{nullptr};
          SectionNode* parent{nullptr};
        };
    }
}
#endif
