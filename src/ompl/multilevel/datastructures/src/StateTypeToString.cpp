#include <ompl/multilevel/datastructures/StateTypeToString.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/util/Exception.h>

std::string ompl::multilevel::stateTypeToString(ompl::base::StateSpacePtr space) 
{
    std::string state_type_string;
    int type = space->getType();
    if (type == ompl::base::STATE_SPACE_REAL_VECTOR)
    {
        int N = space->getDimension();
        state_type_string = "R";
        state_type_string += std::to_string(N);
    }
    else if (type == ompl::base::STATE_SPACE_SE2)
    {
        state_type_string = "SE2";
    }
    else if (type == ompl::base::STATE_SPACE_SE3)
    {
        state_type_string = "SE3";
    }
    else if (type == ompl::base::STATE_SPACE_SO2)
    {
        state_type_string = "SO2";
    }
    else if (type == ompl::base::STATE_SPACE_SO3)
    {
        state_type_string = "SO3";
    }
    else if (type == ompl::base::STATE_SPACE_TIME)
    {
        state_type_string = "T";
    }
    else if (space->isCompound())
    {
        ompl::base::CompoundStateSpace *space_compound = space->as<ompl::base::CompoundStateSpace>();
        const std::vector<ompl::base::StateSpacePtr> space_decomposed = space_compound->getSubspaces();

        for (unsigned int k = 0; k < space_decomposed.size(); k++)
        {
            ompl::base::StateSpacePtr s0 = space_decomposed.at(k);
            state_type_string = state_type_string + ompl::multilevel::stateTypeToString(s0);
            if (k < space_decomposed.size() - 1)
                state_type_string += "x";
        }
    }
    else
    {
        throw ompl::Exception("Unknown State Space");
    }
    return state_type_string;
}
