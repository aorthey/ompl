#define BOOST_TEST_MODULE "FactoredMotionPlanningLifts"
#include <boost/test/unit_test.hpp>

#include "factorization_common.h"
#include <ompl/util/Console.h>

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_InclusionMaps)
{
    ompl::base::StateSpacePtr space_A = CreateCubeStateSpace(4);
    space_A->setName("SpaceA");
    auto A = std::make_shared<FactoredSpaceInformation>(space_A);

    ompl::base::StateSpacePtr space_B = CreateCubeStateSpace(2);
    space_B->setName("SpaceB");
    auto B = std::make_shared<FactoredSpaceInformation>(space_B);

    ompl::base::StateSpacePtr space_C = CreateCubeStateSpace(2);
    space_C->setName("SpaceC");
    auto C = std::make_shared<FactoredSpaceInformation>(space_C);

    const auto indicesB = std::vector<size_t>({0,1});
    const auto indicesC = std::vector<size_t>({2,3});

    auto projAB = std::make_shared<Projection_RN_RM>(space_A, space_B, indicesB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(space_A, space_C, indicesC);
    BOOST_CHECK(A->addChild(C, projAC));

    ////Create states to lift
    auto stateA = A->allocState();
    auto stateB = B->allocState();
    auto stateC = C->allocState();

    auto samplerB = B->allocStateSampler();
    auto samplerC = C->allocStateSampler();

    samplerB->sampleUniform(stateB);
    samplerC->sampleUniform(stateC);

    std::unordered_map<std::string, State*> baseStates;
    baseStates.insert({B->getName(), stateB});
    baseStates.insert({C->getName(), stateC});

    A->lift(baseStates, stateA);

    const auto *stateA_RN = stateA->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateB_RN = stateB->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateC_RN = stateC->as<ompl::base::RealVectorStateSpace::StateType>();

    BOOST_CHECK_EQUAL(stateA_RN->values[0], stateB_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[1], stateB_RN->values[1]);
    BOOST_CHECK_EQUAL(stateA_RN->values[2], stateC_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[3], stateC_RN->values[1]);

    A->freeState(stateA);
    B->freeState(stateB);
    C->freeState(stateC);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_InclusionMapsShifted)
{
    ompl::base::StateSpacePtr space_A = CreateCubeStateSpace(4);
    space_A->setName("SpaceA");
    auto A = std::make_shared<FactoredSpaceInformation>(space_A);

    ompl::base::StateSpacePtr space_B = CreateCubeStateSpace(2);
    space_B->setName("SpaceB");
    auto B = std::make_shared<FactoredSpaceInformation>(space_B);

    ompl::base::StateSpacePtr space_C = CreateCubeStateSpace(2);
    space_C->setName("SpaceC");
    auto C = std::make_shared<FactoredSpaceInformation>(space_C);

    const auto indicesB = std::vector<size_t>({0,3});
    const auto indicesC = std::vector<size_t>({1,2});

    auto projAB = std::make_shared<Projection_RN_RM>(space_A, space_B, indicesB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(space_A, space_C, indicesC);
    BOOST_CHECK(A->addChild(C, projAC));

    ////Create states to lift
    auto stateA = A->allocState();
    auto stateB = B->allocState();
    auto stateC = C->allocState();

    auto samplerB = B->allocStateSampler();
    auto samplerC = C->allocStateSampler();

    samplerB->sampleUniform(stateB);
    samplerC->sampleUniform(stateC);

    std::unordered_map<std::string, State*> baseStates;
    baseStates.insert({B->getName(), stateB});
    baseStates.insert({C->getName(), stateC});

    A->lift(baseStates, stateA);

    const auto *stateA_RN = stateA->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateB_RN = stateB->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateC_RN = stateC->as<ompl::base::RealVectorStateSpace::StateType>();

    BOOST_CHECK_EQUAL(stateA_RN->values[0], stateB_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[1], stateC_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[2], stateC_RN->values[1]);
    BOOST_CHECK_EQUAL(stateA_RN->values[3], stateB_RN->values[1]);

    A->freeState(stateA);
    B->freeState(stateB);
    C->freeState(stateC);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_InclusionMapsMultiConnected)
{
    ompl::base::StateSpacePtr space_A = CreateCubeStateSpace(9);
    space_A->setName("SpaceA");
    auto A = std::make_shared<FactoredSpaceInformation>(space_A);

    ompl::base::StateSpacePtr space_B = CreateCubeStateSpace(2);
    space_B->setName("SpaceB");
    auto B = std::make_shared<FactoredSpaceInformation>(space_B);

    ompl::base::StateSpacePtr space_C = CreateCubeStateSpace(2);
    space_C->setName("SpaceC");
    auto C = std::make_shared<FactoredSpaceInformation>(space_C);

    ompl::base::StateSpacePtr space_D = CreateCubeStateSpace(5);
    space_D->setName("SpaceD");
    auto D = std::make_shared<FactoredSpaceInformation>(space_D);

    const auto indicesB = std::vector<size_t>({0,3});
    const auto indicesC = std::vector<size_t>({1,8});
    const auto indicesD = std::vector<size_t>({2,4,5,6,7});

    auto projAB = std::make_shared<Projection_RN_RM>(space_A, space_B, indicesB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(space_A, space_C, indicesC);
    BOOST_CHECK(A->addChild(C, projAC));
    auto projAD = std::make_shared<Projection_RN_RM>(space_A, space_D, indicesD);
    BOOST_CHECK(A->addChild(D, projAD));

    ////Create states to lift
    auto stateA = A->allocState();
    auto stateB = B->allocState();
    auto stateC = C->allocState();
    auto stateD = D->allocState();

    auto samplerB = B->allocStateSampler();
    auto samplerC = C->allocStateSampler();
    auto samplerD = D->allocStateSampler();

    samplerB->sampleUniform(stateB);
    samplerC->sampleUniform(stateC);
    samplerD->sampleUniform(stateD);

    std::unordered_map<std::string, State*> baseStates;
    baseStates.insert({B->getName(), stateB});
    baseStates.insert({C->getName(), stateC});
    baseStates.insert({D->getName(), stateD});

    A->lift(baseStates, stateA);

    const auto *stateA_RN = stateA->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateB_RN = stateB->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateC_RN = stateC->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateD_RN = stateD->as<ompl::base::RealVectorStateSpace::StateType>();

    BOOST_CHECK_EQUAL(stateA_RN->values[0], stateB_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[3], stateB_RN->values[1]);

    BOOST_CHECK_EQUAL(stateA_RN->values[1], stateC_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[8], stateC_RN->values[1]);

    BOOST_CHECK_EQUAL(stateA_RN->values[2], stateD_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[4], stateD_RN->values[1]);
    BOOST_CHECK_EQUAL(stateA_RN->values[5], stateD_RN->values[2]);
    BOOST_CHECK_EQUAL(stateA_RN->values[6], stateD_RN->values[3]);
    BOOST_CHECK_EQUAL(stateA_RN->values[7], stateD_RN->values[4]);

    A->freeState(stateA);
    B->freeState(stateB);
    C->freeState(stateC);
    D->freeState(stateD);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_LeafNodeLift)
{
    ompl::base::StateSpacePtr space_A = CreateCubeStateSpace(9);
    space_A->setName("SpaceA");
    auto A = std::make_shared<FactoredSpaceInformation>(space_A);

    ompl::base::StateSpacePtr space_B = CreateCubeStateSpace(2);
    space_B->setName("SpaceB");
    auto B = std::make_shared<FactoredSpaceInformation>(space_B);

    ompl::base::StateSpacePtr space_C = CreateCubeStateSpace(2);
    space_C->setName("SpaceC");
    auto C = std::make_shared<FactoredSpaceInformation>(space_C);

    ompl::base::StateSpacePtr space_D = CreateCubeStateSpace(5);
    space_D->setName("SpaceD");
    auto D = std::make_shared<FactoredSpaceInformation>(space_D);

    const auto indicesB = std::vector<size_t>({0,3});
    const auto indicesC = std::vector<size_t>({1,8});
    const auto indicesD = std::vector<size_t>({2,4,5,6,7});

    auto projAB = std::make_shared<Projection_RN_RM>(space_A, space_B, indicesB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(space_A, space_C, indicesC);
    BOOST_CHECK(A->addChild(C, projAC));
    auto projAD = std::make_shared<Projection_RN_RM>(space_A, space_D, indicesD);
    BOOST_CHECK(A->addChild(D, projAD));

    ////Create states to lift
    auto stateA1 = A->allocState();
    auto stateA2 = A->allocState();

    auto stateB = B->allocState();
    auto stateC = C->allocState();
    auto stateD = D->allocState();

    auto samplerB = B->allocStateSampler();
    auto samplerC = C->allocStateSampler();
    auto samplerD = D->allocStateSampler();

    samplerB->sampleUniform(stateB);
    samplerC->sampleUniform(stateC);
    samplerD->sampleUniform(stateD);

    std::unordered_map<std::string, State*> baseStates;
    baseStates.insert({B->getName(), stateB});
    baseStates.insert({C->getName(), stateC});
    baseStates.insert({D->getName(), stateD});

    A->lift(baseStates, stateA1);

    OMPL_INFORM("Lifting leaf states.");
    A->liftLeafStates(baseStates, stateA2);

    const auto *stateA1_RN = stateA1->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateA2_RN = stateA2->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateB_RN = stateB->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateC_RN = stateC->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateD_RN = stateD->as<ompl::base::RealVectorStateSpace::StateType>();

    BOOST_CHECK_EQUAL(stateA1_RN->values[0], stateB_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA2_RN->values[3], stateB_RN->values[1]);

    BOOST_CHECK_EQUAL(stateA1_RN->values[1], stateC_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA1_RN->values[8], stateC_RN->values[1]);

    BOOST_CHECK_EQUAL(stateA1_RN->values[2], stateD_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA1_RN->values[4], stateD_RN->values[1]);
    BOOST_CHECK_EQUAL(stateA1_RN->values[5], stateD_RN->values[2]);
    BOOST_CHECK_EQUAL(stateA1_RN->values[6], stateD_RN->values[3]);
    BOOST_CHECK_EQUAL(stateA1_RN->values[7], stateD_RN->values[4]);
    for(size_t k = 0; k < A->getStateDimension(); k++) {
      BOOST_CHECK_EQUAL(stateA1_RN->values[k], stateA2_RN->values[k]);
    }

    A->freeState(stateA1);
    A->freeState(stateA2);
    B->freeState(stateB);
    C->freeState(stateC);
    D->freeState(stateD);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ComplexLeafNodeLift)
{
  /*        A(8)
   *      /     \
   *    B(3)   C(5)
   *    |      /   \
   *    D(3)  E(2)  F(3)
   *    |
   *    G(3)
   */

    auto A = CreateCubeSpaceInformation(8, "SpaceA");
    auto B = CreateCubeSpaceInformation(3, "SpaceB");
    auto C = CreateCubeSpaceInformation(5, "SpaceC");
    auto D = CreateCubeSpaceInformation(3, "SpaceD");
    auto E = CreateCubeSpaceInformation(2, "SpaceE");
    auto F = CreateCubeSpaceInformation(3, "SpaceF");
    auto G = CreateCubeSpaceInformation(3, "SpaceG");

    const auto indicesAB = std::vector<size_t>({5,6,7});
    const auto indicesAC = std::vector<size_t>({0,1,2,3,4});
    const auto indicesCE = std::vector<size_t>({1,3});
    const auto indicesCF = std::vector<size_t>({0,2,4});
    auto projAB = std::make_shared<Projection_RN_RM>(A, B, indicesAB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(A, C, indicesAC);
    BOOST_CHECK(A->addChild(C, projAC));
    auto projBD = std::make_shared<Projection_RN_RM>(B, D);
    BOOST_CHECK(B->addChild(D, projBD));
    auto projCE = std::make_shared<Projection_RN_RM>(C, E, indicesCE);
    BOOST_CHECK(C->addChild(E, projCE));
    auto projCF = std::make_shared<Projection_RN_RM>(C, F, indicesCF);
    BOOST_CHECK(C->addChild(F, projCF));
    auto projDG = std::make_shared<Projection_RN_RM>(D, G);
    BOOST_CHECK(D->addChild(G, projDG));

    ////Create states to lift
    auto stateG = AllocState(G, {0.7, 0.8, 0.9});
    auto stateE = AllocState(E, {0.3, 0.5});
    auto stateF = AllocState(F, {0.2, 0.4, 0.6});

    auto stateA = AllocState(A, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    std::unordered_map<std::string, ompl::base::State*> leafStates;
    leafStates.insert({G->getName(), stateG});
    leafStates.insert({E->getName(), stateE});
    leafStates.insert({F->getName(), stateF});

    A->liftLeafStates(leafStates, stateA);

    const auto *stateA_RN = stateA->as<ompl::base::RealVectorStateSpace::StateType>();

    //States should be ordered
    for(size_t k = 0; k < A->getStateDimension(); k++) {
      BOOST_CHECK_CLOSE(stateA_RN->values[k], 0.2+k*0.1, 1e-5);
    }

    A->freeState(stateA);

    G->freeState(stateG);
    E->freeState(stateE);
    F->freeState(stateF);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_ParallelLeafNodeLift)
{
  /*        A(8)
   *      /     \
   *     B(4)   C(4)
   *     |       |
   *     D(2)   E(2)
   */

  OMPL_INFORM("--------------------------------------------------------------start parallel leaf node lift");
    auto A = CreateCubeSpaceInformation(8, "SpaceA");
    auto B = CreateCubeSpaceInformation(4, "SpaceB");
    auto C = CreateCubeSpaceInformation(4, "SpaceC");
    auto D = CreateCubeSpaceInformation(2, "SpaceD");
    auto E = CreateCubeSpaceInformation(2, "SpaceE");

    const auto indicesAB = std::vector<size_t>({0,1,2,3});
    const auto indicesAC = std::vector<size_t>({4,5,6,7});
    auto projAB = std::make_shared<Projection_RN_RM>(A, B, indicesAB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(A, C, indicesAC);
    BOOST_CHECK(A->addChild(C, projAC));
    auto projBD = std::make_shared<Projection_RN_RM>(B, D);
    BOOST_CHECK(B->addChild(D, projBD));
    auto projCE = std::make_shared<Projection_RN_RM>(C, E);
    BOOST_CHECK(C->addChild(E, projCE));

    ////Create states to lift
    auto stateD = AllocState(D, {1.0, 1.0});
    auto stateE = AllocState(E, {0.0, 0.0});
    auto stateA = AllocState(A, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    std::unordered_map<std::string, ompl::base::State*> leafStates;
    leafStates.insert({D->getName(), stateD});
    leafStates.insert({E->getName(), stateE});

    A->liftLeafStates(leafStates, stateA);
    A->printState(stateA);

    // const auto *stateA_RN = stateA->as<ompl::base::RealVectorStateSpace::StateType>();
    // //States should be ordered
    // for(size_t k = 0; k < A->getStateDimension(); k++) {
    //   BOOST_CHECK_CLOSE(stateA_RN->values[k], 0.2+k*0.1, 1e-5);
    // }

    A->freeState(stateA);
    D->freeState(stateD);
    E->freeState(stateE);
}
