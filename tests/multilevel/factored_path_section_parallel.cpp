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
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;
using namespace ompl::multilevel;

const double kRadiusDiskRobots = 0.1;
const size_t kMaximumIterations = 5;

bool isStateValid_ComponentSpaceNRobots(const State *state, size_t nRobots)
{
  for(size_t robot1 = 0; robot1 < nRobots; robot1++) 
  {
    for(size_t robot2 = 0; robot2 < nRobots; robot2++) 
    {
      if(robot1 == robot2) {
        continue;
      }

      const auto *R1 = state->as<CompoundState>()->operator[](robot1)->as<RealVectorStateSpace::StateType>();
      const auto *R2 = state->as<CompoundState>()->operator[](robot2)->as<RealVectorStateSpace::StateType>();

      const double& r1x = R1->values[0];
      const double& r1y = R1->values[1];
      const double& r2x = R2->values[0];
      const double& r2y = R2->values[1];

      double distance = std::sqrt(std::pow(r1x-r2x, 2) + std::pow(r1y-r2y, 2));
      if(distance < 2*kRadiusDiskRobots) {
        OMPL_ERROR("Collision at [%f, %f] to [%f, %f]. Distance %f.", r1x, r1y, r2x, r2y, distance);
        return false;
      }
    }
  }
  return true;
}

FactoredSpaceInformationPtr MakeNRobotSpace(size_t N) {
  auto space = std::make_shared<CompoundStateSpace>();
  for(size_t k = 0; k < N; k++) {
    auto spacek(std::make_shared<RealVectorStateSpace>(2));
    spacek->setBounds(-1, 1);
    spacek->setName("Space"+std::to_string(k));
    space->addSubspace(spacek, 1.0);
  }
  space->setName("CompoundSpace");
  space->printSettings(std::cout);

  auto factor(std::make_shared<FactoredSpaceInformation>(space));

  auto validityCheckerFunction = std::bind(isStateValid_ComponentSpaceNRobots, std::placeholders::_1, N);
  factor->setStateValidityChecker(validityCheckerFunction);

  auto subspaces = space->getSubspaces();
  size_t index = 0;
  for(const auto& subspace : subspaces) {
    auto factor_subspace(std::make_shared<FactoredSpaceInformation>(subspace));
    auto projection_subspace = std::make_shared<SubspaceProjection>(factor, factor_subspace, index);
    BOOST_CHECK(factor->addChild(factor_subspace, projection_subspace));
    index++;
  }
  return factor;
}

void SetCompoundState(ompl::base::State* state, size_t index, double x, double y) {
  auto *RN = state->as<CompoundState>()->operator[](index)->as<RealVectorStateSpace::StateType>();
  RN->values[0] = x;
  RN->values[1] = y;
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingPathSectionMultiRobotTest)
{
    auto factor = MakeNRobotSpace(2);
    auto space = factor->getStateSpace();

    ScopedState<> start(space);
    ScopedState<> goal(space);

    SetCompoundState(start.get(), 0, 0.0, 0.0);
    SetCompoundState(start.get(), 1, 0.0, 1.0);
    SetCompoundState(goal.get(), 0, 1.0, 1.0);
    SetCompoundState(goal.get(), 1, 1.0, 0.0);

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setSeed(0);
    planner->setup();
    planner->setRange(1000);
    planner->setSmoothIntermediateSolutions(false);
    planner->setSelectorFunctionType(SelectorFunctionType::kLastLevel);

    auto nd = factor->getStateSpace()->validSegmentCount(start.get(), goal.get());
    BOOST_CHECK_GT(nd, 10u);
    BOOST_CHECK_LE(nd, 50u);
    BOOST_CHECK(!factor->checkMotion(start.get(), goal.get()));

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));
    PlannerStatus solved = planner->solve(ptc);
    BOOST_CHECK(solved);

    auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
    path->print(std::cout);
    BOOST_CHECK_EQUAL(path->getStateCount(), 3u);
    BOOST_CHECK_GE(path->length(), 2.0);
    BOOST_CHECK_LE(path->length(), 2.0 * sqrt(2));
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingPathSectionMultiRobotMultiPathTest)
{
    auto factor = MakeNRobotSpace(2);
    auto space = factor->getStateSpace();
    auto subspaces = space->as<CompoundStateSpace>()->getSubspaces();
    auto space1 = subspaces.at(0);
    auto space2 = subspaces.at(1);
    auto child1 = factor->getChildren().at(0);
    auto child2 = factor->getChildren().at(1);

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
    SetCompoundState(start.get(), 0, 1.0, 0.0);
    SetCompoundState(start.get(), 1, 0.0, 0.0);
    SetCompoundState(goal.get(), 0, 0.0, 1.0);
    SetCompoundState(goal.get(), 1, 1.0, 1.0);
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

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingParallelPathSectionThreeRobotTest)
{
    auto factor = MakeNRobotSpace(3);
    auto space = factor->getStateSpace();

    ScopedState<> start(space);
    ScopedState<> goal(space);

    const size_t numPoints = 3;
    const double pi = M_PI;
    size_t robot_index = 0;
    for (size_t i = 0; i < numPoints; ++i) {
        double angle = 2.0 * pi * i / (2*numPoints);
        SetCompoundState(start.get(), robot_index, cos(angle), sin(angle));
        SetCompoundState(goal.get(), robot_index, cos(angle+pi), sin(angle+pi));
        robot_index++;
    }

    factor->printState(start.get(), std::cout);
    factor->printState(goal.get(), std::cout);

    //Straight line should be invalid
    factor->setup();
    BOOST_CHECK(factor->isSetup());
    BOOST_CHECK(!factor->checkMotion(start.get(), goal.get()));
    BOOST_CHECK(factor->isValid(start.get()));
    BOOST_CHECK(factor->isValid(goal.get()));

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
    BOOST_CHECK_EQUAL(path->getStateCount(), 4u);
    BOOST_CHECK_LT(path->length(), 10.0);
    BOOST_CHECK_GE(path->length(), 5.0);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComputingParallelPathSectionThreeRobotInSequenceTest)
{
    auto factor = MakeNRobotSpace(3);
    auto space = factor->getStateSpace();

    ScopedState<> start(space);
    ScopedState<> goal(space);

    SetCompoundState(start.get(), 0, -1, +0);
    SetCompoundState(start.get(), 1, +0, +1);
    SetCompoundState(start.get(), 2, +1, +0);

    SetCompoundState(goal.get(), 0, +1, +0);
    SetCompoundState(goal.get(), 1, +0, -1);
    SetCompoundState(goal.get(), 2, +0, +1);

    factor->printState(start.get(), std::cout);
    factor->printState(goal.get(), std::cout);

    //Straight line should be invalid
    factor->setup();
    BOOST_CHECK(factor->isSetup());
    BOOST_CHECK(!factor->checkMotion(start.get(), goal.get()));
    BOOST_CHECK(factor->isValid(start.get()));
    BOOST_CHECK(factor->isValid(goal.get()));

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
    BOOST_CHECK_EQUAL(path->getStateCount(), 4u);
    BOOST_CHECK_GE(path->length(), 2.0);
    BOOST_CHECK_LE(path->length(), 2.0 + 2.0 + 1.0*sqrt(2));
}
