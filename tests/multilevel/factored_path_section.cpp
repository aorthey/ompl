#define BOOST_TEST_MODULE "FactoredPathSectionPlanning"
#include <boost/test/unit_test.hpp>

#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/SE2_R2.h>
#include <ompl/multilevel/datastructures/projections/RN_RM.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;
using namespace ompl::multilevel;

const size_t kMaximumIterations = 5;

bool isStateValid_InvalidMidsection(const State *state)
{
    const auto *SE2state = state->as<SE2StateSpace::StateType>();
    const auto *SO2 = SE2state->as<SO2StateSpace::StateType>(1);
    return (std::abs(SO2->value) > boost::math::constants::pi<double>() / 4.0);
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
    factor->setStateValidityChecker(isStateValid_InvalidMidsection);

    auto R2(std::make_shared<RealVectorStateSpace>(2));
    R2->setBounds(0, 1);
    R2->setName(kNameBaseSpace);
    auto factor_R2(std::make_shared<FactoredSpaceInformation>(R2));

    auto projection = std::make_shared<Projection_SE2_R2>(SE2, R2);

    BOOST_CHECK(factor->addChild(factor_R2, projection));

    // Define Planning Problem
    using SE2State = ScopedState<SE2StateSpace>;
    SE2State start(SE2);
    SE2State goal(SE2);
    start->setXY(0, 0);
    start->setYaw(-1.0);
    goal->setXY(1, 1);
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
    const std::string kNameTotalSpace = "SpaceSE2";
    const std::string kNameBaseSpace = "SpaceR2";

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

    auto projection = std::make_shared<Projection_SE2_R2>(SE2, R2);

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
    const std::string kNameTotalSpace = "SpaceSE2";
    const std::string kNameBaseSpace = "SpaceR2";

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

    auto projection = std::make_shared<Projection_SE2_R2>(SE2, R2);

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
    const std::string kNameTotalSpace = "SpaceSE2";
    const std::string kNameBaseSpace = "SpaceR2";

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

    auto projection = std::make_shared<Projection_SE2_R2>(SE2, R2);

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

const float kRadiusDiskRobots = 0.1;

bool robotsCollide(const size_t nRobots, const double values[])
{
  for(size_t robot1 = 0; robot1 < nRobots; robot1++) 
  {
    for(size_t robot2 = 0; robot2 < nRobots; robot2++) 
    {
      if(robot1 == robot2) {
        continue;
      }
      const double& r1x = values[robot1*2];
      const double& r1y = values[robot1*2+1];
      const double& r2x = values[robot2*2];
      const double& r2y = values[robot2*2+1];

      float distance = std::sqrt(std::pow(r1x-r2x, 2) + std::pow(r1y-r2y, 2));
      if(distance < 2*kRadiusDiskRobots) {
        return true;
      }
    }
  }
  return false;
}

bool isStateValid_ComponentSpace(const State *state)
{
    const auto *RN = state->as<RealVectorStateSpace::StateType>();
    return !robotsCollide(2, RN->values);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingPathSectionMultiRobotTest)
{
    auto space = std::make_shared<RealVectorStateSpace>(4);
    space->setBounds(0, 1);
    space->setName("ComponentSpace");
    auto factor(std::make_shared<FactoredSpaceInformation>(space));
    factor->setStateValidityChecker(isStateValid_ComponentSpace);

    auto space1(std::make_shared<RealVectorStateSpace>(2));
    space1->setBounds(0, 1);
    space1->setName("Space1");
    auto space2(std::make_shared<RealVectorStateSpace>(2));
    space2->setBounds(0, 1);
    space2->setName("Space2");

    auto factor1(std::make_shared<FactoredSpaceInformation>(space1));
    auto projection1 = std::make_shared<Projection_RN_RM>(space, space1, std::vector<size_t>({0, 1}));
    BOOST_CHECK(factor->addChild(factor1, projection1));

    auto factor2(std::make_shared<FactoredSpaceInformation>(space2));
    auto projection2 = std::make_shared<Projection_RN_RM>(space, space2, std::vector<size_t>({2, 3}));
    BOOST_CHECK(factor->addChild(factor2, projection2));

    ScopedState<> start(space);
    ScopedState<> goal(space);
    const auto *startRN = start->as<RealVectorStateSpace::StateType>();
    startRN->values[0] = 0.0;
    startRN->values[1] = 0.0;
    startRN->values[2] = 0.0;
    startRN->values[3] = 1.0;
    const auto *goalRN = goal->as<RealVectorStateSpace::StateType>();
    goalRN->values[0] = 1.0;
    goalRN->values[1] = 1.0;
    goalRN->values[2] = 1.0;
    goalRN->values[3] = 0.0;

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

    auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
    path->print(std::cout);
    BOOST_CHECK_EQUAL(path->getStateCount(), 3u);
    BOOST_CHECK_LT(path->length(), 3.0);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingPathSectionMultiRobotMultiPathTest)
{
    auto space = std::make_shared<RealVectorStateSpace>(4);
    space->setBounds(0, 1);
    space->setName("ComponentSpace");
    auto factor(std::make_shared<FactoredSpaceInformation>(space));
    factor->setStateValidityChecker(isStateValid_ComponentSpace);

    auto space1(std::make_shared<RealVectorStateSpace>(2));
    space1->setBounds(0, 1);
    space1->setName("Space1");
    auto space2(std::make_shared<RealVectorStateSpace>(2));
    space2->setBounds(0, 1);
    space2->setName("Space2");

    auto child1(std::make_shared<FactoredSpaceInformation>(space1));
    auto projection1 = std::make_shared<Projection_RN_RM>(space, space1, std::vector<size_t>({0, 1}));
    BOOST_CHECK(factor->addChild(child1, projection1));

    auto child2(std::make_shared<FactoredSpaceInformation>(space2));
    auto projection2 = std::make_shared<Projection_RN_RM>(space, space2, std::vector<size_t>({2, 3}));
    BOOST_CHECK(factor->addChild(child2, projection2));

    ////////////////////////////////////////////////////////////////////////////////
    // Define Planning Problem for child 1
    ////////////////////////////////////////////////////////////////////////////////
    using R2State = ScopedState<RealVectorStateSpace>;
    R2State x1(space1);
    x1->values[0] = 1;
    x1->values[1] = 0;
    R2State x2(space1);
    x2->values[0] = 0.4;
    x2->values[1] = 0.6;
    R2State x3(space1);
    x3->values[0] = 0.6;
    x3->values[1] = 0.2;
    R2State x4(space1);
    x4->values[0] = 0;
    x4->values[1] = 1;

    std::vector<const ompl::base::State*> path_states1;
    path_states1.push_back(x1.get());
    path_states1.push_back(x2.get());
    path_states1.push_back(x3.get());
    path_states1.push_back(x4.get());
    auto base_path1 = std::make_shared<ompl::geometric::PathGeometric>(child1, path_states1);

    auto child_planner1 = std::make_shared<ompl::multilevel::FactoredPlanner>(child1);
    auto child_pdef1 = std::make_shared<ProblemDefinition>(child1);
    child_pdef1->addSolutionPath(base_path1);
    child_planner1->setProblemDefinition(child_pdef1);

    ////////////////////////////////////////////////////////////////////////////////
    // Define Planning Problem for child 2
    ////////////////////////////////////////////////////////////////////////////////
    R2State y1(space2);
    y1->values[0] = 0;
    y1->values[1] = 0;
    R2State y2(space2);
    y2->values[0] = 0.4;
    y2->values[1] = 0.6;
    R2State y3(space2);
    y3->values[0] = 0.6;
    y3->values[1] = 0.4;
    R2State y4(space2);
    y4->values[0] = 1;
    y4->values[1] = 1;

    std::vector<const ompl::base::State*> path_states2;
    path_states2.push_back(y1.get());
    path_states2.push_back(y2.get());
    path_states2.push_back(y3.get());
    path_states2.push_back(y4.get());
    auto base_path2 = std::make_shared<ompl::geometric::PathGeometric>(child2, path_states2);

    auto child_planner2 = std::make_shared<ompl::multilevel::FactoredPlanner>(child2);
    auto child_pdef2 = std::make_shared<ProblemDefinition>(child2);
    child_pdef2->addSolutionPath(base_path2);
    child_planner2->setProblemDefinition(child_pdef2);

    ////////////////////////////////////////////////////////////////////////////////
    // Define Planning Problem for total space
    ////////////////////////////////////////////////////////////////////////////////

    std::vector<ompl::multilevel::FactoredPlannerPtr> planners = {child_planner1, child_planner2};
    auto factor_planner = std::make_shared<ompl::multilevel::FactoredPlanner>(factor, planners);

    auto factor_pdef = std::make_shared<ProblemDefinition>(factor);
    ScopedState<> start(space);
    ScopedState<> goal(space);
    const auto *startRN = start->as<RealVectorStateSpace::StateType>();
    startRN->values[0] = 1.0;
    startRN->values[1] = 0.0;
    startRN->values[2] = 0.0;
    startRN->values[3] = 0.0;
    const auto *goalRN = goal->as<RealVectorStateSpace::StateType>();
    goalRN->values[0] = 0.0;
    goalRN->values[1] = 1.0;
    goalRN->values[2] = 1.0;
    goalRN->values[3] = 1.0;
    factor_pdef->setStartAndGoalStates(start, goal);

    factor_planner->setProblemDefinition(factor_pdef);

    ompl::base::IterationTerminationCondition itc(1);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(factor_pdef));
    PlannerStatus solved = factor_planner->solve(ptc);

    BOOST_CHECK(solved);
    auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(factor_pdef->getSolutionPath());
    path->print(std::cout);
    BOOST_CHECK_GE(path->getStateCount(), 7u);
}
