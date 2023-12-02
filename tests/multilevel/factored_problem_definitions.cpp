#define BOOST_TEST_MODULE "FactoredMotionPlanningProblemDefinition"
#include <boost/test/unit_test.hpp>

#include "factorization_common.h"

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_CorrectProjectionToProblemDefinition)
{
    //    A(6)
    //  /     \
    // B(4)   C(2)
    // |
    // D(2)
    ompl::base::StateSpacePtr space_A = CreateCubeStateSpace(6);
    space_A->setName("SpaceA");
    auto A = std::make_shared<FactoredSpaceInformation>(space_A);

    ompl::base::StateSpacePtr space_B = CreateCubeStateSpace(4);
    space_B->setName("SpaceB");
    auto B = std::make_shared<FactoredSpaceInformation>(space_B);

    ompl::base::StateSpacePtr space_C = CreateCubeStateSpace(2);
    space_C->setName("SpaceC");
    auto C = std::make_shared<FactoredSpaceInformation>(space_C);

    ompl::base::StateSpacePtr space_D = CreateCubeStateSpace(2);
    space_D->setName("SpaceD");
    auto D = std::make_shared<FactoredSpaceInformation>(space_D);

    auto projAB = std::make_shared<Projection_RN_RM>(space_A, space_B, std::vector<size_t>({0,1,4,5}));
    auto projBD = std::make_shared<Projection_RN_RM>(space_B, space_D, std::vector<size_t>({0,3}));
    auto projAC = std::make_shared<Projection_RN_RM>(space_A, space_C, std::vector<size_t>({2,3}));

    A->addChild(B, projAB);
    A->addChild(C, projAC);
    B->addChild(D, projBD);

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(A);
    ScopedState<> start = CreateState(space_A, 0.01, 0.01);
    ScopedState<> goal = CreateState(space_A, 0.91f, 0.01);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(A);
    planner->setProblemDefinition(pdef);
    planner->setup();

    auto pdefs = planner->getProblemDefinitions();

    BOOST_CHECK_GT(pdefs.count("SpaceA"), 0u);
    BOOST_CHECK_GT(pdefs.count("SpaceB"), 0u);
    BOOST_CHECK_GT(pdefs.count("SpaceC"), 0u);
    BOOST_CHECK_GT(pdefs.count("SpaceD"), 0u);
    BOOST_CHECK_EQUAL(pdefs.count("SpaceE"), 0u);

    auto pdefA = pdefs["SpaceA"];
    auto pdefB = pdefs["SpaceB"];
    auto pdefC = pdefs["SpaceC"];
    auto pdefD = pdefs["SpaceD"];

    auto startA = pdefA->getStartState(0);
    auto startB = pdefB->getStartState(0);
    auto startC = pdefC->getStartState(0);
    auto startD = pdefD->getStartState(0);

    const auto *startA_RN = startA->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *startB_RN = startB->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *startC_RN = startC->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *startD_RN = startD->as<ompl::base::RealVectorStateSpace::StateType>();

    BOOST_CHECK_EQUAL(startA_RN->values[0], startB_RN->values[0]);
    BOOST_CHECK_EQUAL(startA_RN->values[1], startB_RN->values[1]);
    BOOST_CHECK_EQUAL(startA_RN->values[4], startB_RN->values[2]);
    BOOST_CHECK_EQUAL(startA_RN->values[5], startB_RN->values[3]);

    BOOST_CHECK_EQUAL(startA_RN->values[2], startC_RN->values[0]);
    BOOST_CHECK_EQUAL(startA_RN->values[3], startC_RN->values[1]);
    BOOST_CHECK_NE(startA_RN->values[0], startC_RN->values[0]);
    BOOST_CHECK_NE(startA_RN->values[0], startC_RN->values[1]);
    BOOST_CHECK_NE(startA_RN->values[1], startC_RN->values[0]);
    BOOST_CHECK_NE(startA_RN->values[1], startC_RN->values[1]);

    BOOST_CHECK_EQUAL(startB_RN->values[0], startD_RN->values[0]);
    BOOST_CHECK_EQUAL(startB_RN->values[3], startD_RN->values[1]);
    BOOST_CHECK_NE(startB_RN->values[1], startD_RN->values[0]);
    BOOST_CHECK_NE(startB_RN->values[1], startD_RN->values[1]);
    BOOST_CHECK_NE(startB_RN->values[2], startD_RN->values[0]);
    BOOST_CHECK_NE(startB_RN->values[2], startD_RN->values[1]);
}
