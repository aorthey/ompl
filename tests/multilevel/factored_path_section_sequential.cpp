#define BOOST_TEST_MODULE "FactoredPathSectionPlanning"
#include <boost/test/unit_test.hpp>

#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/SE2ToR2Projection.h>
#include <ompl/multilevel/datastructures/projections/RNToRMProjection.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;
using namespace ompl::multilevel;

const size_t kMaximumIterations = 5;

bool isStateValid_Midsection(const State *state)
{
    const auto *SE2state = state->as<SE2StateSpace::StateType>();
    const auto *R2 = SE2state->as<RealVectorStateSpace::StateType>(0);
    const auto *SO2 = SE2state->as<SO2StateSpace::StateType>(1);
    const auto x = R2->values[0];
    const auto y = R2->values[1];

    auto d = std::sqrt(x*x + y*y);
    if( d < 0.25 || d > 0.75) {
      return true;
    }
    return (std::abs(SO2->value) > boost::math::constants::pi<double>() / 2.0);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingPathSectionTest)
{
    const std::string kNameTotalSpace = "SpaceSE2";
    const std::string kNameBaseSpace = "SpaceR2";

    auto SE2(std::make_shared<SE2StateSpace>());
    RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    SE2->setBounds(bounds);
    SE2->setName(kNameTotalSpace);
    auto factor(std::make_shared<FactoredSpaceInformation>(SE2));
    factor->setStateValidityChecker(isStateValid_Midsection);

    auto R2(std::make_shared<RealVectorStateSpace>(2));
    R2->setBounds(0, 1);
    R2->setName(kNameBaseSpace);
    auto factor_R2(std::make_shared<FactoredSpaceInformation>(R2));

    auto projection = std::make_shared<SE2ToR2Projection>(SE2, R2);

    BOOST_CHECK(factor->addChild(factor_R2, projection));

    // Define Planning Problem
    using SE2State = ScopedState<SE2StateSpace>;
    SE2State start(SE2);
    SE2State goal(SE2);
    start->setXY(0, 0);
    start->setYaw(-0.25);
    goal->setXY(1, 1);
    goal->setYaw(+0.25);

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start, goal);

    ompl::RNG::setSeed(1);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->setRange(1000);
    planner->setSmoothIntermediateSolutions(false);
    planner->setSelectorFunctionType(SelectorFunctionType::kLastLevel);

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));
    PlannerStatus solved = planner->solve(ptc);

    BOOST_CHECK(solved);
    BOOST_CHECK_GT(planner->getNumberOfIterations(), 0u);
    BOOST_CHECK_LE(planner->getNumberOfIterations(), 2u);

    pdef->getSolutionPath()->print(std::cout);
}

bool isStateValid_ZigZag(const State *state)
{
    const auto *SE2state = state->as<SE2StateSpace::StateType>();
    const auto *R2 = SE2state->as<RealVectorStateSpace::StateType>(0);
    const auto *SO2 = SE2state->as<SO2StateSpace::StateType>(1);
    const auto x = R2->values[0];
    const auto y = R2->values[1];
    const auto yaw = SO2->value;
    const auto pi = boost::math::constants::pi<double>();

    if(y > 0.0) {
      return false;
    }
    if(x > 0.0 && x <= 0.2) {
      return true;
    }
    if(x > 0.2 && x <= 0.4) {
      if(yaw < 0.5*pi) {
        return false;
      }
      return true;
    }
    if(x > 0.4 && x <= 0.6) {
      return true;
    }
    if(x > 0.6 && x <= 0.8) {
      if(yaw < -0.5*pi) {
        return true;
      }
      return false;
    }
    return true;
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingPathSectionZigZagTest)
{
    const auto kNameTotalSpace = "SpaceSE2";
    const auto kNameBaseSpace = "SpaceR2";

    auto SE2(std::make_shared<SE2StateSpace>());
    RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    SE2->setBounds(bounds);
    SE2->setName(kNameTotalSpace);
    auto factor(std::make_shared<FactoredSpaceInformation>(SE2));
    factor->setStateValidityChecker(isStateValid_ZigZag);

    auto R2(std::make_shared<RealVectorStateSpace>(2));
    R2->setBounds(0, 1);
    R2->setName(kNameBaseSpace);
    auto factor_R2(std::make_shared<FactoredSpaceInformation>(R2));

    auto projection = std::make_shared<SE2ToR2Projection>(SE2, R2);

    BOOST_CHECK(factor->addChild(factor_R2, projection));

    // Define Planning Problem
    using SE2State = ScopedState<SE2StateSpace>;
    SE2State start(SE2);
    SE2State goal(SE2);
    start->setXY(0, 0);
    start->setYaw(-1.0);
    goal->setXY(1, 0);
    goal->setYaw(+1.0);

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setSeed(0);
    planner->setup();
    planner->setRange(1000);
    planner->setSmoothIntermediateSolutions(false);
    planner->setSelectorFunctionType(SelectorFunctionType::kLastLevel);

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));
    PlannerStatus solved = planner->solve(ptc);

    BOOST_CHECK(solved);
    BOOST_CHECK_GT(planner->getNumberOfIterations(), 0u);
    BOOST_CHECK_LE(planner->getNumberOfIterations(), 2u);

    auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
    path->print(std::cout);

    BOOST_CHECK_GE(path->getStateCount(), 6u);
    BOOST_CHECK_LE(path->getStateCount(), 7u);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingPathSectionMultiEdgePathTest)
{
    const auto kNameTotalSpace = "SpaceSE2";
    const auto kNameBaseSpace = "SpaceR2";

    auto SE2(std::make_shared<SE2StateSpace>());
    RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    SE2->setBounds(bounds);
    SE2->setName(kNameTotalSpace);
    auto factor(std::make_shared<FactoredSpaceInformation>(SE2));

    auto R2(std::make_shared<RealVectorStateSpace>(2));
    R2->setBounds(0, 1);
    R2->setName(kNameBaseSpace);
    auto child(std::make_shared<FactoredSpaceInformation>(R2));

    auto projection = std::make_shared<SE2ToR2Projection>(SE2, R2);

    BOOST_CHECK(factor->addChild(child, projection));

    // Define Planning Problem
    using SE2State = ScopedState<SE2StateSpace>;
    SE2State start(SE2);
    SE2State goal(SE2);
    start->setXY(0, 0);
    start->setYaw(-1.0);
    goal->setXY(0, 1);
    goal->setYaw(+1.0);

    using R2State = ScopedState<RealVectorStateSpace>;
    R2State x1(R2);
    x1->values[0] = 0;
    x1->values[1] = 0;
    R2State x2(R2);
    x2->values[0] = 1;
    x2->values[1] = 0;
    R2State x3(R2);
    x3->values[0] = 1;
    x3->values[1] = 1;
    R2State x4(R2);
    x4->values[0] = 0;
    x4->values[1] = 1;

    std::vector<const ompl::base::State*> path_states;
    path_states.push_back(x1.get());
    path_states.push_back(x2.get());
    path_states.push_back(x3.get());
    path_states.push_back(x4.get());
    auto base_path = std::make_shared<ompl::geometric::PathGeometric>(child, path_states);


    auto child_planner = std::make_shared<ompl::multilevel::FactoredPlanner>(child);
    auto child_pdef = std::make_shared<ProblemDefinition>(child);
    child_pdef->addSolutionPath(base_path);
    child_planner->setProblemDefinition(child_pdef);

    std::vector<ompl::multilevel::FactoredPlannerPtr> planners = {child_planner};
    auto factor_planner = std::make_shared<ompl::multilevel::FactoredPlanner>(factor, planners);

    auto factor_pdef = std::make_shared<ProblemDefinition>(factor);
    factor_pdef->setStartAndGoalStates(start, goal);
    factor_planner->setProblemDefinition(factor_pdef);

    ompl::base::IterationTerminationCondition itc(1);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(factor_pdef));
    PlannerStatus solved = factor_planner->solve(ptc);

    BOOST_CHECK(solved);
    auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(factor_pdef->getSolutionPath());
    path->print(std::cout);
    BOOST_CHECK_EQUAL(path->getStateCount(), 5u);
}

bool isStateValid_FencePath(const State *state)
{
    const auto *SE2state = state->as<SE2StateSpace::StateType>();
    const auto *R2 = SE2state->as<RealVectorStateSpace::StateType>(0);
    const auto *SO2 = SE2state->as<SO2StateSpace::StateType>(1);
    const auto x = R2->values[0];
    const auto y = R2->values[1];
    const auto yaw = SO2->value;
    const auto pi = boost::math::constants::pi<double>();
    const auto Epsilon = std::numeric_limits<double>::epsilon();

    if(y < Epsilon) {
      if(x > 0.5 && std::abs(yaw) < 0.5*pi) {
        return false;
      }
      return true;
    }
    if(x > 1.0 - Epsilon) {
      if(y <= 0.2) {
        return true;
      }
      if(y >= 0.8) {
        return true;
      }
      if(std::abs(yaw) > 0.25*pi) {
        return false;
      }
      return true;
    }

    if(y > 1.0 - Epsilon) {
      if(x > 0.5 && std::abs(yaw) < 0.5*pi) {
        return false;
      }
      return true;
    }

    return true;
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingPathSectionFencePathTest)
{
    const auto kNameTotalSpace = "SpaceSE2";
    const auto kNameBaseSpace = "SpaceR2";

    auto SE2(std::make_shared<SE2StateSpace>());
    RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    SE2->setBounds(bounds);
    SE2->setName(kNameTotalSpace);
    auto factor(std::make_shared<FactoredSpaceInformation>(SE2));
    factor->setStateValidityChecker(isStateValid_FencePath);

    auto R2(std::make_shared<RealVectorStateSpace>(2));
    R2->setBounds(0, 1);
    R2->setName(kNameBaseSpace);
    auto child(std::make_shared<FactoredSpaceInformation>(R2));

    auto projection = std::make_shared<SE2ToR2Projection>(SE2, R2);

    BOOST_CHECK(factor->addChild(child, projection));

    // Define Planning Problem
    using SE2State = ScopedState<SE2StateSpace>;
    SE2State start(SE2);
    SE2State goal(SE2);
    start->setXY(0, 0);
    start->setYaw(-1.0);
    goal->setXY(0, 1);
    goal->setYaw(+1.0);

    using R2State = ScopedState<RealVectorStateSpace>;
    R2State x1(R2);
    x1->values[0] = 0;
    x1->values[1] = 0;
    R2State x2(R2);
    x2->values[0] = 1;
    x2->values[1] = 0;
    R2State x3(R2);
    x3->values[0] = 1;
    x3->values[1] = 1;
    R2State x4(R2);
    x4->values[0] = 0;
    x4->values[1] = 1;

    std::vector<const ompl::base::State*> path_states;
    path_states.push_back(x1.get());
    path_states.push_back(x2.get());
    path_states.push_back(x3.get());
    path_states.push_back(x4.get());
    auto base_path = std::make_shared<ompl::geometric::PathGeometric>(child, path_states);


    auto child_planner = std::make_shared<ompl::multilevel::FactoredPlanner>(child);
    auto child_pdef = std::make_shared<ProblemDefinition>(child);
    child_pdef->addSolutionPath(base_path);
    child_planner->setProblemDefinition(child_pdef);

    std::vector<ompl::multilevel::FactoredPlannerPtr> planners = {child_planner};
    auto factor_planner = std::make_shared<ompl::multilevel::FactoredPlanner>(factor, planners);

    auto factor_pdef = std::make_shared<ProblemDefinition>(factor);
    factor_pdef->setStartAndGoalStates(start, goal);
    factor_planner->setProblemDefinition(factor_pdef);

    ompl::base::IterationTerminationCondition itc(1);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(factor_pdef));
    PlannerStatus solved = factor_planner->solve(ptc);

    BOOST_CHECK(solved);
    auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(factor_pdef->getSolutionPath());
    path->print(std::cout);
    BOOST_CHECK_GE(path->getStateCount(), 10u);
    BOOST_CHECK_GE(path->length(), 3.0);
}
