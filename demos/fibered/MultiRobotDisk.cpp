#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>

#include <iostream>
#include <fstream>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>

#include "MultiRobotDiskEnvironment.h"

//Problem: All robots start as disks with certain radius. Start position is
//equally distributed around the unit circle in R2 workspace.
//Goal position is to reach antipodal points for each robot.

const size_t kNumberOfDiskRobots = 7;
const float kRadiusDiskRobots = 0.15;
const size_t kMaximumIterations = 10000;
const size_t kDefaultSeed = 23;

int main()
{
  MultiRobotDiskEnvironment env(kNumberOfDiskRobots, kRadiusDiskRobots);

  // auto factor = env.constructDecompositionFactorTree();
  auto factor = env.constructPrioritizedFactorTree();

  auto start = env.CreateStartStates(factor->getStateSpace());
  auto goal = env.CreateGoalStates(factor->getStateSpace());

  ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
  pdef->setStartAndGoalStates(start, goal);

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setSeed(kDefaultSeed);

  ompl::base::IterationTerminationCondition itc(kMaximumIterations);
  auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

  PlannerStatus solved = planner->solve(ptc);

  if(solved == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION) {
    auto path = pdef->getSolutionPath();
    path->print(std::cout);

    auto geom_path = path->as<ompl::geometric::PathGeometric>();
    auto path_simplifier = std::make_shared<ompl::geometric::PathSimplifier>(factor);
    path_simplifier->simplifyMax(*geom_path);
    path_simplifier->smoothBSpline(*geom_path);

    geom_path->print(std::cout);

    std::ofstream pathfile(boost::str(boost::format("multi_robot_disk_path_prioritization.dat")).c_str());
    geom_path->printAsMatrix(pathfile);
  } else {
    auto path = pdef->getSolutionPath();
    if(path) {
      OMPL_INFORM("Found approximate path");
      path->print(std::cout);
    }
  }
  return 0;
}
