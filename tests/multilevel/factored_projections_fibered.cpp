#define BOOST_TEST_MODULE "FactoredMotionPlanningLifts"

#include <boost/mpl/vector.hpp>
#include <boost/mpl/push_back.hpp>
#include <boost/test/unit_test.hpp>

#include <vector>

#include "factorization_common.h"
#include <ompl/multilevel/datastructures/projections/FiberedSubspaceProjection.h>
#include <ompl/multilevel/datastructures/projections/SE2RNToR2Projection.h>
#include <ompl/multilevel/datastructures/projections/SE3RNToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/SE3ToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/SE2ToR2Projection.h>

#include <ompl/multilevel/datastructures/projections/R3R2SO2ToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/R3SO2ToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/XR3R2SO2ToXR3Projection.h>
#include <ompl/multilevel/datastructures/projections/XR3SO2ToXR3Projection.h>
#include <ompl/multilevel/datastructures/projections/XSE2RNToXR2Projection.h>

#include <ompl/multilevel/datastructures/projections/SO2NToSO2MProjection.h>
#include <ompl/multilevel/datastructures/projections/RNSO2ToRNProjection.h>

#include <ompl/multilevel/datastructures/projections/SO3RNToSO3RMProjection.h>
#include <ompl/multilevel/datastructures/projections/SO2RNToSO2RMProjection.h>
#include <ompl/multilevel/datastructures/projections/SE3RNToSE3RMProjection.h>
#include <ompl/multilevel/datastructures/projections/SE2RNToSE2RMProjection.h>

#include <ompl/multilevel/datastructures/projections/SO3RNToSO3Projection.h>
#include <ompl/multilevel/datastructures/projections/SO2RNToSO2Projection.h>
#include <ompl/multilevel/datastructures/projections/SE3RNToSE3Projection.h>
#include <ompl/multilevel/datastructures/projections/SE2RNToSE2Projection.h>

#include <ompl/multilevel/datastructures/projections/XTimeToXProjection.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include "ompl/geometric/PathGeometric.h"

#include <ompl/util/Console.h>

const size_t kNumberOfSamplesToTest = 5;
const float kLiftingAccuracy = 1e-5;
const float kPathPlanningAccuracy = 1e-5;
const size_t kMaximumIterations = 10;

////////////////////////////////////////////////////////////////////////////////
/// Helper functions
////////////////////////////////////////////////////////////////////////////////

RealVectorBounds GetDefaultBounds(unsigned int dim) {
  auto bounds = RealVectorBounds(dim);
  bounds.setLow(0.0);
  bounds.setHigh(+1.0);
  return bounds;
}

std::shared_ptr<RealVectorStateSpace> MakeRN(unsigned int dim) {
  auto space = std::make_shared<RealVectorStateSpace>(dim);
  space->setBounds(GetDefaultBounds(dim));
  return space;
}

std::shared_ptr<SO2StateSpace> MakeSO2() {
  auto space = std::make_shared<SO2StateSpace>();
  return space;
}

std::shared_ptr<SE2StateSpace> MakeSE2() {
  auto space = std::make_shared<SE2StateSpace>();
  space->setBounds(GetDefaultBounds(2));
  return space;
}

std::shared_ptr<SO3StateSpace> MakeSO3() {
  auto space = std::make_shared<SO3StateSpace>();
  return space;
}

std::shared_ptr<SE3StateSpace> MakeSE3() {
  auto space = std::make_shared<SE3StateSpace>();
  space->setBounds(GetDefaultBounds(3));
  return space;
}

std::shared_ptr<TimeStateSpace> MakeTime() {
  auto space = std::make_shared<TimeStateSpace>();
  return space;
}

std::shared_ptr<CompoundStateSpace> MakeCompound(const std::initializer_list<StateSpacePtr>& spaces) {
  std::vector<StateSpacePtr> compounds(spaces);
  std::vector<double> weights(spaces.size(), 1.0);
  return std::make_shared<CompoundStateSpace>(compounds, weights);
}

////////////////////////////////////////////////////////////////////////////////
/// List of fibered projection types
////////////////////////////////////////////////////////////////////////////////

struct RN_RM {
  auto getProjection() {return std::make_shared<RNToRMProjection>(MakeRN(29), MakeRN(17));}
};
struct RNSO2_RN {
  auto getProjection() {return std::make_shared<RNSO2ToRNProjection>(MakeRN(29)+MakeSO2(), MakeRN(29));}
};
struct SE2_R2 {
  auto getProjection() {return std::make_shared<SE2ToR2Projection>(MakeSE2(), MakeRN(2));}
};
struct SE2RN_R2 {
  auto getProjection() {return std::make_shared<SE2RNToR2Projection>(MakeSE2()+MakeRN(4), MakeRN(2));}
};
struct SE3_R3 {
  auto getProjection() {return std::make_shared<SE3ToR3Projection>(MakeSE3(), MakeRN(3));}
};
struct SE3RN_R3 {
  auto getProjection() {return std::make_shared<SE3RNToR3Projection>(MakeSE3()+MakeRN(4), MakeRN(3));}
};
struct SO2N_SO2M {
  auto getProjection() {return std::make_shared<SO2NToSO2MProjection>(MakeSO2()+MakeSO2()+MakeSO2()+MakeSO2()+MakeSO2(), MakeSO2()+MakeSO2());}
};
struct R3R2SO2_R3 {
  auto getProjection() {return std::make_shared<R3R2SO2ToR3Projection>(MakeRN(3)+MakeRN(2)+MakeSO2(), MakeRN(3));}
};
struct R3SO2_R3 {
  auto getProjection() {return std::make_shared<R3SO2ToR3Projection>(MakeRN(3)+MakeSO2(), MakeRN(3));}
};
struct XR3R2SO2_XR3 {
  auto getProjection() {return std::make_shared<XR3R2SO2ToXR3Projection>(
      MakeCompound({MakeCompound({MakeRN(3), MakeRN(2), MakeSO2()}), MakeCompound({MakeRN(3), MakeRN(2), MakeSO2()})}),
      MakeCompound({MakeCompound({MakeRN(3), MakeRN(2), MakeSO2()}), MakeRN(3)}));}
};
struct XR3SO2_XR3 {
  auto getProjection() {return std::make_shared<XR3SO2ToXR3Projection>(
      MakeCompound({MakeCompound({MakeRN(3), MakeSO2()}), MakeCompound({MakeRN(3), MakeSO2()})}),
      MakeCompound({MakeCompound({MakeRN(3), MakeSO2()}), MakeRN(3)}));}
};
struct XSE2RN_XR2 {
  auto getProjection() {return std::make_shared<XSE2RNToXR2Projection>(
      MakeCompound({MakeCompound({MakeSE2(), MakeRN(7)}), MakeCompound({MakeSE2(), MakeRN(7)})}),
      MakeCompound({MakeCompound({MakeSE2(), MakeRN(7)}), MakeRN(2)}));}
};
struct SO2RN_SO2RM {
  auto getProjection() {return std::make_shared<SO2RNToSO2RMProjection>(MakeSO2()+MakeRN(17), MakeSO2()+MakeRN(9));}
};
struct SO3RN_SO3RM {
  auto getProjection() {return std::make_shared<SO3RNToSO3RMProjection>(MakeSO3()+MakeRN(4), MakeSO3()+MakeRN(1));}
};
struct SE2RN_SE2RM {
  auto getProjection() {return std::make_shared<SE2RNToSE2RMProjection>(MakeSE2()+MakeRN(17), MakeSE2()+MakeRN(9));}
};
struct SE3RN_SE3RM {
  auto getProjection() {return std::make_shared<SE3RNToSE3RMProjection>(MakeSE3()+MakeRN(4), MakeSE3()+MakeRN(1));}
};
struct SO2RN_SO2 {
  auto getProjection() {return std::make_shared<SO2RNToSO2Projection>(MakeSO2()+MakeRN(17), MakeSO2());}
};
struct SE2RN_SE2 {
  auto getProjection() {return std::make_shared<SE2RNToSE2Projection>(MakeSE2()+MakeRN(17), MakeSE2());}
};
struct SO3RN_SO3 {
  auto getProjection() {return std::make_shared<SO3RNToSO3Projection>(MakeSO3()+MakeRN(17), MakeSO3());}
};
struct SE3RN_SE3 {
  auto getProjection() {return std::make_shared<SE3RNToSE3Projection>(MakeSE3()+MakeRN(17), MakeSE3());}
};
struct SubspaceSE3 {
  static auto MakeSubspace() {
    static auto space = MakeSE3();
    return space;
  }
  auto getProjection() {return std::make_shared<FiberedSubspaceProjection>(
      MakeCompound({MakeSE3(), MakeSubspace(), MakeSE3()}), 
      MakeSubspace());}
};
struct SubspaceSE2RN {
  static auto MakeSubspace() {
    static auto space = MakeSE2();
    return space;
  }
  auto getProjection() {return std::make_shared<FiberedSubspaceProjection>(
      MakeCompound({MakeSE2(), MakeSubspace(), MakeSE2()}),
      MakeSubspace());}
};
struct TimeBasedSE3 {
  auto getProjection() {return std::make_shared<XTimeToXProjection>(MakeCompound({MakeSE3(), MakeTime()}), MakeCompound({MakeSE3()}));}
};
struct TimeBasedSE2RN {
  auto getProjection() {return std::make_shared<XTimeToXProjection>(MakeCompound({MakeSE2(), MakeRN(7), MakeTime()}), MakeCompound({MakeSE2(), MakeRN(7)}));}
};

typedef boost::mpl::vector<
    RN_RM,
    RNSO2_RN,
    R3R2SO2_R3,
    R3SO2_R3,
    XR3R2SO2_XR3,
    XR3SO2_XR3,
    XSE2RN_XR2,
    SO2N_SO2M,
    SO2RN_SO2,
    SO2RN_SO2RM,
    SO3RN_SO3RM,
    SO3RN_SO3,
    SE2_R2,
    SE2RN_R2, 
    SE2RN_SE2RM,
    SE2RN_SE2,
    SE3_R3,
    SE3RN_R3,
    SE3RN_SE3RM,
    SE3RN_SE3
> ProjectionTypes1;

//Workaround to get around size limitation of mpl::vector
typedef boost::mpl::push_back<ProjectionTypes1, SubspaceSE2RN>::type ProjectionTypes2;
typedef boost::mpl::push_back<ProjectionTypes2, SubspaceSE3>::type ProjectionTypes3;
typedef boost::mpl::push_back<ProjectionTypes3, TimeBasedSE3>::type ProjectionTypes4;
typedef boost::mpl::push_back<ProjectionTypes4, TimeBasedSE2RN>::type ProjectionTypes;

////////////////////////////////////////////////////////////////////////////////
/// Tests over all projection types
////////////////////////////////////////////////////////////////////////////////

BOOST_AUTO_TEST_CASE_TEMPLATE(FiberedProjections_ProjectStatesAndLiftAgainTest, T, ProjectionTypes)
{
  T templated_object;
  auto projAB = templated_object.getProjection();
  OMPL_INFORM("\n%s\nRunning test for projection %s\n%s\n", std::string(80, '*').c_str(), projAB->getTypeAsString().c_str(), std::string(80, '*').c_str());

  auto spaceA = projAB->getBundle();
  auto spaceB = projAB->getBase();

  auto A = std::make_shared<FactoredSpaceInformation>(spaceA);
  auto B = std::make_shared<FactoredSpaceInformation>(spaceB);
  BOOST_CHECK(A->addChild(B, projAB));
  A->setup();
  BOOST_CHECK_GT(A->getSpaceMeasure(), 0.0);
  BOOST_CHECK_GT(B->getSpaceMeasure(), 0.0);
  BOOST_CHECK(A->isSetup());
  BOOST_CHECK(B->isSetup());
  auto F = projAB->getFiber();

  //Allocate states
  for(size_t k = 0; k < kNumberOfSamplesToTest; k++) {
    auto stateA = A->allocState();
    auto stateAprime = A->allocState();
    auto stateB = B->allocState();
    auto stateF = F->allocState();

    //Generate sample on A
    auto samplerA = A->allocStateSampler();
    samplerA->sampleUniform(stateA);

    //Generate projection onto B
    projAB->project(stateA, stateB);

    //Generate projection onto F
    projAB->projectFiber(stateA, stateF);

    //Lift states back to A
    projAB->lift(stateB, stateF, stateAprime);

    auto distance = A->getStateSpace()->distance(stateAprime, stateAprime);
    if(distance > kLiftingAccuracy) {
      OMPL_INFORM("%s", std::string(80, '*').c_str());
      OMPL_INFORM("Distance after projection and lift: %f", distance);
      OMPL_INFORM("%s", std::string(80, '*').c_str());
      A->printState(stateA);
      OMPL_INFORM("%s", std::string(80, '*').c_str());
      B->printState(stateB);
      OMPL_INFORM("%s", std::string(80, '*').c_str());
      A->printState(stateAprime);
    }

    BOOST_CHECK_CLOSE(distance, 0.0, kLiftingAccuracy);
    BOOST_CHECK_CLOSE(A->getStateSpace()->distance(stateAprime, stateAprime), 0.0, 1e-5);
    BOOST_CHECK_CLOSE(A->getStateSpace()->distance(stateA, stateA), 0.0, 1e-5);

    //Clean up
    A->freeState(stateA);
    A->freeState(stateAprime);
    B->freeState(stateB);
    F->freeState(stateF);
  }
}

BOOST_AUTO_TEST_CASE_TEMPLATE(FiberedProjections_PlanOverProjection, T, ProjectionTypes)
{
  T templated_object;
  auto projAB = templated_object.getProjection();
  OMPL_INFORM("\n%s\nRunning planning test for projection %s\n%s\n", std::string(80, '*').c_str(), projAB->getTypeAsString().c_str(), std::string(80, '*').c_str());

  //Setup spaces
  auto spaceA = projAB->getBundle();
  auto spaceB = projAB->getBase();

  auto A = std::make_shared<FactoredSpaceInformation>(spaceA);
  auto B = std::make_shared<FactoredSpaceInformation>(spaceB);
  BOOST_CHECK(A->addChild(B, projAB));
  A->setup();

  auto start = A->allocState();
  auto goal = A->allocState();

  //Create random start/goal states
  auto samplerA = A->allocStateSampler();

  for(size_t k = 0; k < kNumberOfSamplesToTest; k++) {
    samplerA->sampleUniform(start);
    samplerA->sampleUniform(goal);

    auto pdef = std::make_shared<ProblemDefinition>(A);
    pdef->setStartAndGoalStates(start, goal);

    //Plan a connection
    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(A);
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->setRange(std::numeric_limits<double>::infinity());

    ompl::base::IterationTerminationCondition itc(kMaximumIterations);
    auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

    PlannerStatus solved = planner->solve(ptc);

    BOOST_CHECK(solved);
    BOOST_CHECK_EQUAL(solved, ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION);

    auto path = pdef->getSolutionPath();
    BOOST_CHECK(path->check());

    auto gpath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(path);
    BOOST_CHECK(gpath);
    BOOST_CHECK_EQUAL(gpath->getStateCount(), 2u);

    auto distance_expected = A->getStateSpace()->distance(start, goal);
    BOOST_CHECK_CLOSE(gpath->length(), distance_expected, kPathPlanningAccuracy);

    pdef->getSolutionPath()->print(std::cout);
  }

  A->freeState(start);
  A->freeState(goal);
}
