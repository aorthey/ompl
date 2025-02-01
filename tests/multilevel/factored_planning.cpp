#define BOOST_TEST_MODULE "FactoredMotionPlanning"
#include <boost/test/unit_test.hpp>

#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/SE2ToR2Projection.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;
using namespace ompl::multilevel;

const size_t kMaximumIterations = 500;

bool boxConstraint(const double values[])
{
    const double x = values[0] - 0.5;
    const double y = values[1] - 0.5;
    double pos_cnstr = sqrt(x * x + y * y);
    return pos_cnstr > 0.2;
}
bool isStateValid_SE2(const State *state)
{
    const auto *SE2state = state->as<SE2StateSpace::StateType>();
    const auto *R2 = SE2state->as<RealVectorStateSpace::StateType>(0);
    const auto *SO2 = SE2state->as<SO2StateSpace::StateType>(1);
    return boxConstraint(R2->values) && (SO2->value < boost::math::constants::pi<double>() / 2.0);
}
bool isStateValid_R2(const State *state)
{
    const auto *R2 = state->as<RealVectorStateSpace::StateType>();
    return boxConstraint(R2->values);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_OneLevelPlanning)
{
    auto R2(std::make_shared<RealVectorStateSpace>(2));
    R2->setBounds(0, 1);
    R2->setName("SpaceR2");
    auto factor(std::make_shared<FactoredSpaceInformation>(R2));
    factor->setStateValidityChecker(isStateValid_R2);

    // Define Planning Problem
    using R2State = ScopedState<RealVectorStateSpace>;
    R2State start(R2);
    R2State goal(R2);
    start->values[0] = 0;
    start->values[1] = 0;
    goal->values[0] = 1;
    goal->values[1] = 1;

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

    PlannerStatus solved = planner->solve(ptc);

    BOOST_CHECK_EQUAL(solved, ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION);

    pdef->getSolutionPath()->print(std::cout);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_RigidBodyPlanning)
{
    auto SE2(std::make_shared<SE2StateSpace>());
    RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    SE2->setBounds(bounds);
    SE2->setName("SpaceSE2");
    auto factor(std::make_shared<FactoredSpaceInformation>(SE2));
    factor->setStateValidityChecker(isStateValid_SE2);

    auto R2(std::make_shared<RealVectorStateSpace>(2));
    R2->setBounds(0, 1);
    R2->setName("SpaceR2");
    auto factor_R2(std::make_shared<FactoredSpaceInformation>(R2));
    factor_R2->setStateValidityChecker(isStateValid_R2);

    auto projection = std::make_shared<SE2ToR2Projection>(SE2, R2);

    BOOST_CHECK(factor->addChild(factor_R2, projection));

    // Define Planning Problem
    using SE2State = ScopedState<SE2StateSpace>;
    SE2State start(SE2);
    SE2State goal(SE2);
    start->setXY(0, 0);
    start->setYaw(0);
    goal->setXY(1, 1);
    goal->setYaw(0);

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setSeed(0);
    planner->setup();

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

    PlannerStatus solved = planner->solve(ptc);

    BOOST_CHECK(solved);

    pdef->getSolutionPath()->print(std::cout);
}

