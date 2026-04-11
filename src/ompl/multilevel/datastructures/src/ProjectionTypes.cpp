#include <ompl/multilevel/datastructures/ProjectionTypes.h>

using namespace ompl::multilevel;

std::string ompl::multilevel::AsString(const ProjectionType& type)
{
    switch (type)
    {
        case PROJECTION_NONE:
            return "PROJECTION_NONE";
        case PROJECTION_EMPTY_SET:
            return "PROJECTION_EMPTY_SET";
        case PROJECTION_IDENTITY:
            return "PROJECTION_IDENTITY";
        case PROJECTION_CONSTRAINED_RELAXATION:
            return "PROJECTION_CONSTRAINED_RELAXATION";
        case PROJECTION_RN_RM:
            return "PROJECTION_RN_RM";
        case PROJECTION_SE2_R2:
            return "PROJECTION_SE2_R2";
        case PROJECTION_SE2RN_R2:
            return "PROJECTION_SE2RN_R2";
        case PROJECTION_SE2RN_SE2:
            return "PROJECTION_SE2RN_SE2";
        case PROJECTION_SE2RN_SE2RM:
            return "PROJECTION_SE2RN_SE2RM";
        case PROJECTION_SO2RN_SO2:
            return "PROJECTION_SO2RN_SO2";
        case PROJECTION_SO2RN_SO2RM:
            return "PROJECTION_SO2RN_SO2RM";
        case PROJECTION_SE3_R3:
            return "PROJECTION_SE3_R3";
        case PROJECTION_SE3RN_R3:
            return "PROJECTION_SE3RN_R3";
        case PROJECTION_SE3RN_SE3:
            return "PROJECTION_SE3RN_SE3";
        case PROJECTION_SE3RN_SE3RM:
            return "PROJECTION_SE3RN_SE3RM";
        case PROJECTION_SO3RN_SO3:
            return "PROJECTION_SO3RN_SO3";
        case PROJECTION_SO3RN_SO3RM:
            return "PROJECTION_SO3RN_SO3RM";
        case PROJECTION_RNSO2_RN:
            return "PROJECTION_RNSO2_RN";
        case PROJECTION_SO2N_SO2M:
            return "PROJECTION_SO2N_SO2M";
        case PROJECTION_TASK_SPACE:
            return "PROJECTION_TASK_SPACE";
        case PROJECTION_TIME_BASED:
            return "PROJECTION_TIME_BASED";
        case PROJECTION_SUBSPACE:
            return "PROJECTION_SUBSPACE";
        case PROJECTION_R3R2SO2_R3:
            return "PROJECTION_R3R2SO2_R3";
        case PROJECTION_R3SO2_R3:
            return "PROJECTION_R3SO2_R3";
        case PROJECTION_XR3SO2_XR3:
            return "PROJECTION_XR3SO2_XR3";
        case PROJECTION_XSE2RN_XR2:
            return "PROJECTION_XSE2RN_XR2";
        case PROJECTION_XR3R2SO2_XR3:
            return "PROJECTION_XR3R2SO2_XR3";
        case PROJECTION_COMPOUND:
            return "PROJECTION_COMPOUND";
        case PROJECTION_UNKNOWN:
            return "PROJECTION_UNKNOWN";
        default:
            return "PROJECTION_UNKNOWN";
    }
}
