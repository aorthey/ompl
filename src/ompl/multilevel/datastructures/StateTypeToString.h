/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_STATETYPETOSTRING_
#define OMPL_MULTILEVEL_DATASTRUCTURES_STATETYPETOSTRING_

#include <string>

#include <ompl/util/ClassForward.h>

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
        std::string stateTypeToString(ompl::base::StateSpacePtr space);
    }
}
#endif
