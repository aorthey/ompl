#include <fstream>

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/SE2_R2.h>
#include "MultiRobotManipulatorsCommon.h"

using namespace ompl::base;
using namespace ompl::geometric;

class ProjectionJointSpaceToSE2 : public ompl::multilevel::Projection
{
public:
    ProjectionJointSpaceToSE2(StateSpacePtr bundle, StateSpacePtr base, PlanarManipulator *manip)
      : Projection(bundle, base), manip_(manip)
    {
        type_ = ompl::multilevel::PROJECTION_TASK_SPACE;
    }

    void project(const State *xBundle, State *xBase) const
    {
        double *xBundleValues = xBundle->as<PlanarManipulatorStateSpace::StateType>()->values;
        std::vector<double> reals;
        for(size_t dim = 0; dim < getBundle()->getDimension(); dim++)
        {
          reals.push_back(xBundleValues[dim]);
        }

        Eigen::Affine2d eeFrame;
        manip_->FK(reals, eeFrame);

        double x = eeFrame.translation()(0);
        double y = eeFrame.translation()(1);
        double yaw = acos(eeFrame.matrix()(0, 0));

        xBase->as<SE2StateSpace::StateType>()->setXY(x, y);
        xBase->as<SE2StateSpace::StateType>()->setYaw(yaw);

        getBundle()->printState(xBundle);
        getBase()->printState(xBase);
    }

    void lift(const State *xBase, State *xBundle) const
    {
        auto *xBase_SE2 = xBase->as<ompl::base::SE2StateSpace::StateType>();

        // to Eigen
        Eigen::Affine2d eeFrame = Eigen::Affine2d::Identity();
        eeFrame.translation()(0) = xBase_SE2->getX();
        eeFrame.translation()(1) = xBase_SE2->getY();
        eeFrame.rotate(xBase_SE2->getYaw());

        std::vector<double> solution;
        manip_->FABRIK(solution, eeFrame);

        double *angles = xBundle->as<PlanarManipulatorStateSpace::StateType>()->values;
        for (uint k = 0; k < solution.size(); k++)
        {
            angles[k] = solution.at(k);
        }
    }

private:
    PlanarManipulator *manip_;
};

int main()
{
    Eigen::Affine2d baseFrame;
    Eigen::Affine2d goalFrame;

    PlanarManipulator manipulator = PlanarManipulator(numLinks, 1.0 / numLinks);
    PolyWorld world = createCorridorProblem(numLinks, baseFrame, goalFrame);

    // #########################################################################
    // ## Create robot joint configuration space [TOTAL SPACE]
    // #########################################################################
    ompl::base::StateSpacePtr space(new PlanarManipulatorStateSpace(numLinks));
    ompl::base::RealVectorBounds bounds(numLinks);
    bounds.setLow(-M_PI);
    bounds.setHigh(M_PI);
    space->as<PlanarManipulatorStateSpace>()->setBounds(bounds);
    manipulator.setBounds(bounds.low, bounds.high);

    auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));
    factor->setStateValidityChecker(std::make_shared<PlanarManipulatorCollisionChecker>(factor, manipulator, &world));
    factor->setStateValidityCheckingResolution(0.001);

    // #########################################################################
    // ## Create task space [SE2 BASE SPACE]
    // #########################################################################
    ompl::base::StateSpacePtr spaceSE2(new SE2StateSpace());
    ompl::base::RealVectorBounds boundsWorkspace(2);
    boundsWorkspace.setLow(-2);
    boundsWorkspace.setHigh(+2);
    spaceSE2->as<SE2StateSpace>()->setBounds(boundsWorkspace);

    auto child(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(spaceSE2));
    child->setStateValidityChecker(std::make_shared<SE2CollisionChecker>(child, &world));
    child->setStateValidityCheckingResolution(0.001);

    // #########################################################################
    // ## Create task space [R2 BASE SPACE]
    // #########################################################################
    ompl::multilevel::ProjectionPtr projAB = std::make_shared<ProjectionJointSpaceToSE2>(space, spaceSE2, &manipulator);

    factor->addChild(child, projAB);

    // #########################################################################
    // ## Set start state
    // #########################################################################
    ompl::base::State *start = factor->allocState();
    double *start_angles = start->as<PlanarManipulatorStateSpace::StateType>()->values;

    for (int i = 0; i < numLinks; ++i)
    {
        start_angles[i] = 1e-1 * (pow(-1, i)) + i * 1e-3;
    }

    // #########################################################################
    // ## Set goal state
    // #########################################################################
    ompl::base::State *goal = factor->allocState();

    std::vector<double> goalJoints;
    manipulator.IK(goalJoints, goalFrame);

    double *goal_angles = goal->as<PlanarManipulatorStateSpace::StateType>()->values;
    goal_angles[0] = 0.346324;
    goal_angles[1] = 0.0828153;
    goal_angles[2] = 2.96842;
    goal_angles[3] = -2.17559;
    goal_angles[4] = -0.718962;
    goal_angles[5] = 0.16532;
    goal_angles[6] = -0.228314;
    goal_angles[7] = 0.172762;

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->addStartState(start);
    pdef->setGoalState(goal, 1e-3);

    factor->freeState(start);
    factor->freeState(goal);

    // #########################################################################
    // ## Invoke planner
    // #########################################################################
    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setup();

    PlannerStatus status = planner->Planner::solve(timeout);

    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
        status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        PathPtr path = pdef->getSolutionPath();
        PathGeometric &pgeo = *static_cast<PathGeometric *>(path.get());
        OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());

        pgeo.interpolate(250);
        WriteVisualization(manipulator, &world, pgeo);
    }
}
