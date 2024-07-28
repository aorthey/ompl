#define BOOST_TEST_MODULE "FactoredMotionPlanningMultiRobot"
#include <boost/test/unit_test.hpp>

#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/RN_RM.h>

#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <fstream>

using namespace ompl::base;
using namespace ompl::multilevel;

//Problem: All robots start as disks with certain radius. Start position is
//equally distributed around the unit circle in R2 workspace.
//Goal position is to reach antipodal points for each robot.

const size_t kNumberOfDiskRobots = 3;
const float kRadiusDiskRobots = 0.1;
const size_t kDefaultSeed = 23;

const float kLowerBound = -1;
const float kUpperBound = +1;
const size_t kTotalDimension = kNumberOfDiskRobots * 2;

const size_t kMaximumIterations = 500;

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
    return !robotsCollide(kNumberOfDiskRobots, RN->values);
}

ScopedState<> CreateStartStates(const ompl::base::StateSpacePtr& space) 
{
  ScopedState<> state(space);

  float step_size = 2*M_PI / (kNumberOfDiskRobots);

  for(size_t k = 0; k < kNumberOfDiskRobots; k++) 
  {
    float position = k * step_size;
    state[2*k] = std::cos(position);
    state[2*k+1] = std::sin(position);
    OMPL_INFORM("Position: %f is (%f,%f)", position, state[2*k], state[2*k+1]);
  }
  return state;
}

ScopedState<> CreateGoalStates(const ompl::base::StateSpacePtr& space) 
{
  ScopedState<> state(space);

  float step_size = 2*M_PI / (kNumberOfDiskRobots);

  for(size_t k = 0; k < kNumberOfDiskRobots; k++) 
  {
    float position = k * step_size;
    if(position > M_PI) position -= M_PI;
    else position += M_PI;
    state[2*k] = std::cos(position);
    state[2*k+1] = std::sin(position);
    OMPL_INFORM("Position: %f is (%f,%f)", position, state[2*k], state[2*k+1]);
  }
  return state;
}

FactoredSpaceInformationPtr constructComponentSpace() 
{
    auto component_space = std::make_shared<RealVectorStateSpace>(kTotalDimension);
    component_space->setBounds(kLowerBound, kUpperBound);
    component_space->setName("SpaceComponentSpace");
    auto factor(std::make_shared<FactoredSpaceInformation>(component_space));
    factor->setStateValidityChecker(isStateValid_ComponentSpace);
    return factor;
}

FactoredSpaceInformationPtr constructDecompositionFactorTree() 
{
    auto factor = constructComponentSpace();

    for(size_t k = 0; k < kNumberOfDiskRobots; k++) 
    {
      auto robot_space(std::make_shared<RealVectorStateSpace>(2));
      robot_space->setBounds(kLowerBound, kUpperBound);
      robot_space->setName("SpaceRobot"+std::to_string(k));
      auto robot_factor(std::make_shared<FactoredSpaceInformation>(robot_space));
      auto projection = std::make_shared<Projection_RN_RM>(factor->getStateSpace(), robot_space, std::vector<size_t>({2*k, 2*k+1}));

      BOOST_CHECK(factor->addChild(robot_factor, projection));
    }
    return factor;
}

FactoredSpaceInformationPtr constructPrioritizedFactorTree() 
{
    auto factor = constructComponentSpace();

    FactoredSpaceInformationPtr current = factor;

    for(size_t k = 0; k < kNumberOfDiskRobots - 1; k++) 
    {
      auto subdimension = kTotalDimension - 2*(k+1);
      auto robot_space(std::make_shared<RealVectorStateSpace>(subdimension));
      robot_space->setBounds(kLowerBound, kUpperBound);
      std::string name = "SpaceRobot";
      const auto Nrobots = kNumberOfDiskRobots - k - 1;
      for(size_t j = 0; j < Nrobots; j++) 
      {
        name+= std::to_string(j);
      }
      robot_space->setName(name);
      auto robot_factor(std::make_shared<FactoredSpaceInformation>(robot_space));

      robot_factor->setStateValidityChecker(
        [Nrobots](const State *state) -> bool
        {
            const auto *RN = state->as<RealVectorStateSpace::StateType>();
            return !robotsCollide(Nrobots, RN->values);
        }
      );

      auto projection = std::make_shared<Projection_RN_RM>(current->getStateSpace(), robot_space);
      BOOST_CHECK(current->addChild(robot_factor, projection));
      current = robot_factor;
    }
    return factor;
}

BOOST_AUTO_TEST_CASE(FactoredMultiRobot_ComponentSpacePlanning)
{
    auto factor = constructComponentSpace();

    auto start = CreateStartStates(factor->getStateSpace());
    auto goal = CreateGoalStates(factor->getStateSpace());

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->setSeed(kDefaultSeed);

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

    PlannerStatus solved = planner->solve(ptc);

    BOOST_CHECK_EQUAL(solved, ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION);

    pdef->getSolutionPath()->print(std::cout);
}

BOOST_AUTO_TEST_CASE(FactoredMultiRobot_DecompositionBasedPlanning)
{
    auto factor = constructDecompositionFactorTree();

    auto start = CreateStartStates(factor->getStateSpace());
    auto goal = CreateGoalStates(factor->getStateSpace());

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->setSeed(kDefaultSeed);

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

    PlannerStatus solved = planner->solve(ptc);

    BOOST_CHECK(solved);

    pdef->getSolutionPath()->print(std::cout);
}

BOOST_AUTO_TEST_CASE(FactoredMultiRobot_PrioritizedBasedPlanning)
{
    auto factor = constructPrioritizedFactorTree();

    auto start = CreateStartStates(factor->getStateSpace());
    auto goal = CreateGoalStates(factor->getStateSpace());

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setSeed(kDefaultSeed);
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

    PlannerStatus solved = planner->solve(ptc);

    BOOST_CHECK_EQUAL(solved, ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION);
    auto path = pdef->getSolutionPath();

    BOOST_CHECK_GT(path->length(), 0.0f);
    path->print(std::cout);
    BOOST_CHECK(path->check());

    std::ofstream pathfile(boost::str(boost::format("factored_multirobot_path_prioritized.dat")).c_str());
    path->as<ompl::geometric::PathGeometric>()->printAsMatrix(pathfile);
}
