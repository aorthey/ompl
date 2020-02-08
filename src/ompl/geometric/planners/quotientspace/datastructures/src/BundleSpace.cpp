#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpace.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/util/Exception.h>

unsigned int ompl::geometric::BundleSpace::counter_ = 0;

ompl::geometric::BundleSpace::BundleSpace(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : base::Planner(si, "BundleSpace"), Bundle(si), parent_(parent_)
{
    id_ = counter_++;

    //############################################################################
    //Check for dynamic spaces
    //############################################################################
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(si_.get());
    if(siC==nullptr) {
      isDynamic_ = false;
    }else{
      isDynamic_ = true;
    }
    OMPL_DEVMSG1("BundleSpace %d%s", id_, (isDynamic_?" (dynamic)":""));
    //############################################################################

    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();

    //(1) Number of Components
    //(2) Type of Components
    //############################################################################

    int bundleSpaceComponents = GetNumberOfComponents(Bundle_space);

    if (!hasParent())
    {
      //TODO: still need to create one component subspace per component
        OMPL_DEVMSG1("NO_BUNDLE_STRUCTURE dimension: %d measure: %f", Bundle_space->getDimension(), Bundle_space->getMeasure());
        BundleSpaceComponentPtr component = componentFactory.MakeBundleSpaceComponent(Bundle_space, nullptr);
        components_.push_back(component);
    }
    else
    {
        parent_->setChild(this);

        Base = parent_->getSpaceInformation();
        const base::StateSpacePtr Base_space = Base->getStateSpace();

        base::StateSpacePtr Fiber_space = nullptr;
        if(bundleSpaceComponents > 1){
          OMPL_ERROR("NYI");
          // for(uint k = 0; k < bundleSpaceComponents; k++){
          //   base::CompoundStateSpace *compound = Bundle_space->as<base::CompoundStateSpace>();
          //   nrComponents = compound->getSubspaceCount();
          //   const std::vector<base::StateSpacePtr> decomposed = compound->getSubspaces();
          // }
        }else{
          BundleSpaceComponentPtr component = 
            componentFactory.MakeBundleSpaceComponent(Bundle_space, Base_space);
          Fiber_space = component->getFiberSpace();
        }

        if (Fiber_space != nullptr)
        {
            Fiber = std::make_shared<base::SpaceInformation>(Fiber_space);
            Fiber_sampler_ = Fiber->allocStateSampler();

            if (Base_space->getDimension() + Fiber_space->getDimension() != Bundle_space->getDimension())
            {
                throw ompl::Exception("BundleSpace Dimensions are wrong.");
            }
            OMPL_DEVMSG1("Base dimension: %d measure: %f", Base_space->getDimension(), Base_space->getMeasure());
            OMPL_DEVMSG1("Fiber dimension: %d measure: %f", Fiber_space->getDimension(), Fiber_space->getMeasure());
            OMPL_DEVMSG1("Bundle dimension: %d measure: %f", Bundle_space->getDimension(), Bundle_space->getMeasure());
            if ((Base_space->getMeasure() <= 0) || (Fiber_space->getMeasure() <= 0) || (Bundle_space->getMeasure() <= 0))
            {
                throw ompl::Exception("Zero-measure BundleSpace detected.");
            }
            checkSpaceHasFiniteMeasure(Fiber_space);
        }
        else
        {
            OMPL_DEVMSG1("Base dimension: %d measure: %f", Base_space->getDimension(), Base_space->getMeasure());
            OMPL_DEVMSG1("Bundle dimension: %d measure: %f", Bundle_space->getDimension(), Bundle_space->getMeasure());
        }
        checkSpaceHasFiniteMeasure(Base_space);
    }
    checkSpaceHasFiniteMeasure(Bundle_space);

    if (!Bundle_valid_sampler_)
    {
        Bundle_valid_sampler_ = Bundle->allocValidStateSampler();
    }
    if (!Bundle_sampler_)
    {
        Bundle_sampler_ = Bundle->allocStateSampler();
    }
    if (hasParent())
    {
        xBaseTmp_ = Base->allocState();
        if (getFiberDimension() > 0)
            xFiberTmp_ = Fiber->allocState();
    }
}

int ompl::geometric::BundleSpace::GetNumberOfComponents(base::StateSpacePtr space)
{
  int nrComponents = 0;

  if(space->isCompound()){
    base::CompoundStateSpace *compound = space->as<base::CompoundStateSpace>();
    nrComponents = compound->getSubspaceCount();
    if(nrComponents == 2)
    {
      int type = space->getType();

      if((type == base::STATE_SPACE_SE2) || (type == base::STATE_SPACE_SE3))
      {
        nrComponents = 1;
      }else{
        const std::vector<base::StateSpacePtr> decomposed = compound->getSubspaces();
        int t0 = decomposed.at(0)->getType();
        int t1 = decomposed.at(1)->getType();
        if(
            (t0 == base::STATE_SPACE_SO2 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
            (t0 == base::STATE_SPACE_SO3 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
            (t0 == base::STATE_SPACE_SE2 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
            (t0 == base::STATE_SPACE_SE3 && t1 == base::STATE_SPACE_REAL_VECTOR) 
          )
        {
          nrComponents = 1;
        }
      }
    }
  }else{
    nrComponents = 1;
  }
  return nrComponents;
}

ompl::geometric::BundleSpace::~BundleSpace()
{
    if (hasParent())
    {
        if (xBaseTmp_)
            Base->freeState(xBaseTmp_);
        if (Fiber && xFiberTmp_)
            Fiber->freeState(xFiberTmp_);
    }
}

bool ompl::geometric::BundleSpace::hasParent() const
{
    return !(parent_ == nullptr);
}

bool ompl::geometric::BundleSpace::hasChild() const
{
    return !(child_ == nullptr);
}

bool ompl::geometric::BundleSpace::isDynamic() const
{
    return isDynamic_;
}

void ompl::geometric::BundleSpace::setup()
{
    BaseT::setup();
    hasSolution_ = false;
    firstRun_ = true;
    if(pdef_) goal_ = pdef_->getGoal().get();
}

void ompl::geometric::BundleSpace::clear()
{
    BaseT::clear();

    hasSolution_ = false;
    firstRun_ = true;
    if (!hasParent() && getFiberDimension() > 0)
        Fiber_sampler_.reset();

    pdef_->clearSolutionPaths();
}

void ompl::geometric::BundleSpace::checkSpaceHasFiniteMeasure(const base::StateSpacePtr space) const
{
    if (space->getMeasure() >= std::numeric_limits<double>::infinity())
    {
        const base::StateSpacePtr Base_space = Base->getStateSpace();
        const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
        OMPL_ERROR("Base dimension: %d measure: %f", Base_space->getDimension(), Base_space->getMeasure());
        OMPL_ERROR("Bundle dimension: %d measure: %f", Bundle_space->getDimension(), Bundle_space->getMeasure());
        if (Fiber != nullptr)
        {
            const base::StateSpacePtr Fiber_space = Fiber->getStateSpace();
            OMPL_ERROR("Fiber dimension: %d measure: %f", Fiber_space->getDimension(), Fiber_space->getMeasure());
        }
        throw ompl::Exception("BundleSpace has no bounds");
    }
}

ompl::base::PlannerStatus ompl::geometric::BundleSpace::solve(const base::PlannerTerminationCondition &ptc)
{
    (void)ptc;
    throw ompl::Exception("A Quotient-Space cannot be solved alone. Use class MultiQuotient to solve Quotient-Spaces.");
}

void ompl::geometric::BundleSpace::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    BaseT::setProblemDefinition(pdef);

    if (pdef_->hasOptimizationObjective())
    {
        opt_ = pdef_->getOptimizationObjective();
    }
    else
    {
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
    }
}

void ompl::geometric::BundleSpace::resetCounter()
{
    BundleSpace::counter_ = 0;
}

// const ompl::base::StateSpacePtr 
// ompl::geometric::BundleSpace::computeFiberSpace(
//     const base::StateSpacePtr Bundle_in,
//     const base::StateSpacePtr Base_in, 
//     BundleType qtype)
// {
//     base::StateSpacePtr Fiber_local{nullptr};
//     switch (qtype)
//     {
//         case MULTIAGENT:
//             OMPL_ERROR("NYI");
//             throw ompl::Exception("NYI");
//         case EMPTY_SET_PROJECTION:
//             Fiber_local = Bundle_in;
//             break;
//         case IDENTITY_SPACE_RN:
//         case IDENTITY_SPACE_SE2:
//         case IDENTITY_SPACE_SE2RN:
//         case IDENTITY_SPACE_SO2RN:
//         case IDENTITY_SPACE_SE3:
//         case IDENTITY_SPACE_SE3RN:
//         {
//             Fiber_local = std::make_shared<base::RealVectorStateSpace>(0);
//             break;
//         }
//         case RN_RM:
//         {
//             unsigned int N1 = Bundle_in->getDimension();
//             unsigned int N0 = Base_in->getDimension();
//             unsigned int NX = N1 - N0;
//             Fiber_local = std::make_shared<base::RealVectorStateSpace>(NX);
//             base::RealVectorBounds Bundle_bounds = std::static_pointer_cast<base::RealVectorStateSpace>(Bundle_in)->getBounds();
//             std::vector<double> low;
//             low.resize(NX);
//             std::vector<double> high;
//             high.resize(NX);
//             base::RealVectorBounds Fiber_bounds(NX);
//             for (unsigned int k = 0; k < NX; k++)
//             {
//                 Fiber_bounds.setLow(k, Bundle_bounds.low.at(k + N0));
//                 Fiber_bounds.setHigh(k, Bundle_bounds.high.at(k + N0));
//             }
//             std::static_pointer_cast<base::RealVectorStateSpace>(Fiber_local)->setBounds(Fiber_bounds);

//             break;
//         }
//         case SE2_R2:
//         {
//             Fiber_local = std::make_shared<base::SO2StateSpace>();
//             break;
//         }
//         case SE3_R3:
//         {
//             Fiber_local = std::make_shared<base::SO3StateSpace>();
//             break;
//         }
//         case SE2RN_SE2:
//         case SE3RN_SE3:
//         case SO2RN_SO2:
//         {
//             base::CompoundStateSpace *Bundle_compound = Bundle_in->as<base::CompoundStateSpace>();
//             const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

//             unsigned int NX = Bundle_decomposed.at(1)->getDimension();

//             Fiber_local = std::make_shared<base::RealVectorStateSpace>(NX);
//             std::static_pointer_cast<base::RealVectorStateSpace>(Fiber_local)->setBounds(
//                 std::static_pointer_cast<base::RealVectorStateSpace>(Bundle_decomposed.at(1))->getBounds());

//             break;
//         }
//         case SE2RN_R2:
//         {
//             base::CompoundStateSpace *Bundle_compound = Bundle_in->as<base::CompoundStateSpace>();
//             const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
//             const std::vector<base::StateSpacePtr> Bundle_SE2_decomposed =
//                 Bundle_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

//             const base::RealVectorStateSpace *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
//             unsigned int N = Bundle_RN->getDimension();

//             base::StateSpacePtr SO2(new base::SO2StateSpace());
//             base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
//             RN->as<base::RealVectorStateSpace>()->setBounds(Bundle_RN->getBounds());

//             Fiber_local = SO2 + RN;
//             break;
//         }
//         case SE3RN_R3:
//         {
//             base::CompoundStateSpace *Bundle_compound = Bundle_in->as<base::CompoundStateSpace>();
//             const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
//             const std::vector<base::StateSpacePtr> Bundle_SE3_decomposed =
//                 Bundle_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

//             const base::RealVectorStateSpace *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
//             unsigned int N = Bundle_RN->getDimension();

//             base::StateSpacePtr SO3(new base::SO3StateSpace());
//             base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
//             RN->as<base::RealVectorStateSpace>()->setBounds(Bundle_RN->getBounds());

//             Fiber_local = SO3 + RN;
//             break;
//         }
//         case SE2RN_SE2RM:
//         case SO2RN_SO2RM:
//         case SE3RN_SE3RM:
//         {
//             base::CompoundStateSpace *Bundle_compound = Bundle_in->as<base::CompoundStateSpace>();
//             const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
//             base::CompoundStateSpace *Base_compound = Base_in->as<base::CompoundStateSpace>();
//             const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

//             unsigned int N = Bundle_decomposed.at(1)->getDimension();
//             unsigned int M = Base_decomposed.at(1)->getDimension();
//             unsigned int NX = N - M;
//             Fiber_local = std::make_shared<base::RealVectorStateSpace>(NX);

//             base::RealVectorBounds Bundle_bounds =
//                 std::static_pointer_cast<base::RealVectorStateSpace>(Bundle_decomposed.at(1))->getBounds();
//             std::vector<double> low;
//             low.resize(NX);
//             std::vector<double> high;
//             high.resize(NX);
//             base::RealVectorBounds Fiber_bounds(NX);
//             for (unsigned int k = 0; k < NX; k++)
//             {
//                 Fiber_bounds.setLow(k, Bundle_bounds.low.at(k + M));
//                 Fiber_bounds.setHigh(k, Bundle_bounds.high.at(k + M));
//             }
//             std::static_pointer_cast<base::RealVectorStateSpace>(Fiber_local)->setBounds(Fiber_bounds);
//             break;
//         }
//         default:
//         {
//             OMPL_ERROR("Unknown BundleSpace type: %d", qtype);
//             throw ompl::Exception("Unknown type");
//         }
//     }
//     return Fiber_local;
// }

//const ompl::base::StateSpacePtr 
//ompl::geometric::BundleSpace::computeFiberSpace(
//    const base::StateSpacePtr Bundle_in, 
//    const base::StateSpacePtr Base_in)
//{

//    if (Base_in->getDimension() == 0 || Bundle_in->getDimension() == 0)
//    {
//        OMPL_ERROR("Base has dimension %d.", Base_dimension_);
//        OMPL_ERROR("Bundle has dimension %d.", Bundle_dimension_);
//        throw ompl::Exception("Detected Zero-dimensional BundleSpace.");
//    }

//    type_ = identifyBundleType(Bundle_in, Base_in);

//    if(type_ == MULTIAGENT){
//        //split spaces into types, then compute quotient for each type. then
//        //combine them into Fiber
//        base::CompoundStateSpace *Bundle_compound = Bundle_in->as<base::CompoundStateSpace>();
//        const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
//        base::CompoundStateSpace *Base_compound = Base_in->as<base::CompoundStateSpace>();
//        const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
//        base::StateSpacePtr Fiber_space = std::make_shared<base::CompoundStateSpace>();

//        for(uint k = 0; k < Bundle_decomposed.size(); k++){
//            base::StateSpacePtr Bundlek = Bundle_decomposed.at(k);
//            base::StateSpacePtr Basek = Base_decomposed.at(k);

//            BundleType typek = identifyBundleType(Bundlek, Basek);
//            base::StateSpacePtr Fiberk = computeFiberSpace(Bundlek, Basek, typek);

//            double weight = (Fiberk->getDimension() > 0 ? 1.0 : 0.0);
//            std::static_pointer_cast<base::CompoundStateSpace>(Fiber_space)->addSubspace(Fiberk, weight);
//            types_.push_back(typek);
//        }
//        return Fiber_space;

//    }else{
//        return computeFiberSpace(Bundle_in, Base_in, type_);
//    }
//}


void ompl::geometric::BundleSpace::mergeStates(const base::State *xBase, const base::State *xFiber, base::State *xBundle) const
{
    unsigned int M = components_.size();

    if( M > 1){
        for(uint m = 0; m < M; m++){
            const base::State *xmBase = xBase->as<base::CompoundState>()->as<base::State>(m);
            const base::State *xmFiber = xFiber->as<base::CompoundState>()->as<base::State>(m);
            base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
            components_.at(m)->mergeStates(xmBase, xmFiber, xmBundle);
        }
    }else{
        components_.front()->mergeStates(xBase, xFiber, xBundle);
    }
}

//  if(type_ == MULTIAGENT){
//      const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
//      const base::StateSpacePtr Fiber_space = Fiber->getStateSpace();
//      const base::StateSpacePtr Base_space = Base->getStateSpace();

//      base::CompoundStateSpace *Bundle_compound = Bundle_space->as<base::CompoundStateSpace>();
//      const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
//      base::CompoundStateSpace *Base_compound = Base_space->as<base::CompoundStateSpace>();
//      const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
//      base::CompoundStateSpace *Fiber_compound = Fiber_space->as<base::CompoundStateSpace>();
//      const std::vector<base::StateSpacePtr> Fiber_decomposed = Fiber_compound->getSubspaces();

//      //splitState
//      for(uint k = 0; k < Bundle_decomposed.size(); k++){
//        base::StateSpacePtr Basek = Base_decomposed.at(k);
//        base::StateSpacePtr Fiberk = Fiber_decomposed.at(k);
//        base::StateSpacePtr Bundlek = Bundle_decomposed.at(k);

//        BundleType typek = types_.at(k);

//        const base::State *qkBase = qBase->as<base::CompoundState>()->as<base::State>(k);
//        const base::State *qkFiber = qFiber->as<base::CompoundState>()->as<base::State>(k);
//        base::State *qkBundle = qBundle->as<base::CompoundState>()->as<base::State>(k);

//        switch(typek)
//        {
//          case EMPTY_SET_PROJECTION:
//          {
//            //Bundle = Fiber (eq. to fiber space)
//            Bundlek->copyState(qkBundle, qkFiber);
//            break;
//          }
//          case IDENTITY_SPACE_RN:
//          case IDENTITY_SPACE_SE2:
//          case IDENTITY_SPACE_SE2RN:
//          case IDENTITY_SPACE_SO2RN:
//          case IDENTITY_SPACE_SE3:
//          case IDENTITY_SPACE_SE3RN:
//          {
//            //Bundle = Base (eq. to base space)
//            Bundlek->copyState(qkBundle, qkBase);
//            break;
//          }
//          case RN_RM:
//          {
//            base::RealVectorStateSpace::StateType *sBundle = qkBundle->as<base::RealVectorStateSpace::StateType>();
//            const base::RealVectorStateSpace::StateType *sBase = qkBase->as<base::RealVectorStateSpace::StateType>();
//            const base::RealVectorStateSpace::StateType *sFiber = qkFiber->as<base::RealVectorStateSpace::StateType>();

//            for (unsigned int k = 0; k < Basek->getDimension(); k++)
//            {
//                sBundle->values[k] = sBase->values[k];
//            }
//            for (unsigned int k = Basek->getDimension(); k < Bundlek->getDimension(); k++)
//            {
//                sBundle->values[k] = sFiber->values[k - Basek->getDimension()];
//            }
//            break;
//          }
//          case SE2RN_R2:
//          {
//              base::SE2StateSpace::StateType *sBundle_SE2 =
//                  qBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//              base::RealVectorStateSpace::StateType *sBundle_RN =
//                  qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//              const base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();
//              const base::SO2StateSpace::StateType *sFiber_SO2 =
//                  qFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
//              const base::RealVectorStateSpace::StateType *sFiber_RN =
//                  qFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//              sBundle_SE2->setX(sBase->values[0]);
//              sBundle_SE2->setY(sBase->values[1]);
//              sBundle_SE2->setYaw(sFiber_SO2->value);

//              for (unsigned int k = 0; k < Fiberk->getDimension() - 1; k++)
//              {
//                  sBundle_RN->values[k] = sFiber_RN->values[k];
//              }
//              break;
//          }
//          default:
//          {
//            mergeStates(qkBase, qkFiber, qkBundle, typek);
//            break;
//          }
//        }
//      }
//      return;
//  }else{
//    return mergeStates(qBase, qFiber, qBundle, type_);
//  }

//void ompl::geometric::BundleSpace::mergeStates(
//    const base::State *qBase, 
//    const base::State *qFiber, 
//    base::State *qBundle,
//    const BundleType qtype) const
//{

//    ////input : qBase \in Base, qFiber \in Fiber
//    ////output: qBundle = qBase \circ qFiber \in Bundle
//    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
//    const base::StateSpacePtr Fiber_space = Fiber->getStateSpace();
//    const base::StateSpacePtr Base_space = parent_->getSpaceInformation()->getStateSpace();

//    switch (qtype)
//    {
//        case EMPTY_SET_PROJECTION:
//        case MULTIAGENT:
//            OMPL_ERROR("NYI");
//            throw ompl::Exception("NYI");
//        case IDENTITY_SPACE_RN:
//        case IDENTITY_SPACE_SE2:
//        case IDENTITY_SPACE_SE2RN:
//        case IDENTITY_SPACE_SO2RN:
//        case IDENTITY_SPACE_SE3:
//        case IDENTITY_SPACE_SE3RN:
//        {
//            Bundle->copyState(qBundle, qBase);
//        }
//        case RN_RM:
//        {
//            base::RealVectorStateSpace::StateType *sBundle = qBundle->as<base::RealVectorStateSpace::StateType>();
//            const base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();
//            const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

//            for (unsigned int k = 0; k < Base_dimension_; k++)
//            {
//                sBundle->values[k] = sBase->values[k];
//            }
//            for (unsigned int k = Base_dimension_; k < Bundle_dimension_; k++)
//            {
//                sBundle->values[k] = sFiber->values[k - Base_dimension_];
//            }
//            break;
//        }
//        case SE2_R2:
//        {
//            base::SE2StateSpace::StateType *sBundle = qBundle->as<base::SE2StateSpace::StateType>();
//            const base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();
//            const base::SO2StateSpace::StateType *sFiber = qFiber->as<base::SO2StateSpace::StateType>();

//            sBundle->setXY(sBase->values[0], sBase->values[1]);
//            sBundle->setYaw(sFiber->value);

//            break;
//        }
//        case SE3_R3:
//        {
//            base::SE3StateSpace::StateType *sBundle = qBundle->as<base::SE3StateSpace::StateType>();
//            base::SO3StateSpace::StateType *sBundle_rotation = &sBundle->rotation();

//            const base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();
//            const base::SO3StateSpace::StateType *sFiber = qFiber->as<base::SO3StateSpace::StateType>();

//            sBundle->setXYZ(sBase->values[0], sBase->values[1], sBase->values[2]);

//            sBundle_rotation->x = sFiber->x;
//            sBundle_rotation->y = sFiber->y;
//            sBundle_rotation->z = sFiber->z;
//            sBundle_rotation->w = sFiber->w;

//            break;
//        }
//        case SE3RN_R3:
//        {
//            base::SE3StateSpace::StateType *sBundle_SE3 =
//                qBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//            base::SO3StateSpace::StateType *sBundle_SO3 = &sBundle_SE3->rotation();
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();
//            const base::SO3StateSpace::StateType *sFiber_SO3 =
//                qFiber->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
//            const base::RealVectorStateSpace::StateType *sFiber_RN =
//                qFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            sBundle_SE3->setXYZ(sBase->values[0], sBase->values[1], sBase->values[2]);
//            sBundle_SO3->x = sFiber_SO3->x;
//            sBundle_SO3->y = sFiber_SO3->y;
//            sBundle_SO3->z = sFiber_SO3->z;
//            sBundle_SO3->w = sFiber_SO3->w;

//            for (unsigned int k = 0; k < Fiber_dimension_ - 3; k++)
//            {
//                sBundle_RN->values[k] = sFiber_RN->values[k];
//            }

//            break;
//        }
//        case SE2RN_SE2:
//        {
//            base::SE2StateSpace::StateType *sBundle_SE2 =
//                qBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::SE2StateSpace::StateType *sBase = qBase->as<base::SE2StateSpace::StateType>();
//            const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

//            sBundle_SE2->setX(sBase->getX());
//            sBundle_SE2->setY(sBase->getY());
//            sBundle_SE2->setYaw(sBase->getYaw());

//            for (unsigned int k = 0; k < Fiber_dimension_; k++)
//            {
//                sBundle_RN->values[k] = sFiber->values[k];
//            }
//            break;
//        }
//        case SO2RN_SO2:
//        {
//            base::SO2StateSpace::StateType *sBundle_SO2 =
//                qBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::SO2StateSpace::StateType *sBase = qBase->as<base::SO2StateSpace::StateType>();
//            const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

//            sBundle_SO2->value = sBase->value;

//            for (unsigned int k = 0; k < Fiber_dimension_; k++)
//            {
//                sBundle_RN->values[k] = sFiber->values[k];
//            }
//            break;
//        }
//        case SO2RN_SO2RM:
//        {
//            base::SO2StateSpace::StateType *sBundle_SO2 =
//                qBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::SO2StateSpace::StateType *sBase_SO2 =
//                qBase->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
//            const base::RealVectorStateSpace::StateType *sBase_RM =
//                qBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

//            sBundle_SO2->value = sBase_SO2->value;

//            unsigned int M = Bundle_dimension_ - Fiber_dimension_ - 1;
//            unsigned int N = Fiber_dimension_;

//            for (unsigned int k = 0; k < M; k++)
//            {
//                sBundle_RN->values[k] = sBase_RM->values[k];
//            }
//            for (unsigned int k = M; k < M + N; k++)
//            {
//                sBundle_RN->values[k] = sFiber->values[k - M];
//            }
//            break;
//        }

//        case SE2RN_R2:
//        {
//            base::SE2StateSpace::StateType *sBundle_SE2 =
//                qBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();
//            const base::SO2StateSpace::StateType *sFiber_SO2 =
//                qFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
//            const base::RealVectorStateSpace::StateType *sFiber_RN =
//                qFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            sBundle_SE2->setX(sBase->values[0]);
//            sBundle_SE2->setY(sBase->values[1]);
//            sBundle_SE2->setYaw(sFiber_SO2->value);

//            for (unsigned int k = 0; k < Fiber_dimension_ - 1; k++)
//            {
//                sBundle_RN->values[k] = sFiber_RN->values[k];
//            }
//            break;
//        }
//        case SE2RN_SE2RM:
//        {
//            base::SE2StateSpace::StateType *sBundle_SE2 =
//                qBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::SE2StateSpace::StateType *sBase_SE2 =
//                qBase->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//            const base::RealVectorStateSpace::StateType *sBase_RM =
//                qBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

//            sBundle_SE2->setX(sBase_SE2->getX());
//            sBundle_SE2->setY(sBase_SE2->getY());
//            sBundle_SE2->setYaw(sBase_SE2->getYaw());

//            //[X Y YAW] [1...M-1][M...N-1]
//            // SE2               RN
//            unsigned int M = Bundle_dimension_ - Fiber_dimension_ - 3;
//            unsigned int N = Fiber_dimension_;

//            for (unsigned int k = 0; k < M; k++)
//            {
//                sBundle_RN->values[k] = sBase_RM->values[k];
//            }
//            for (unsigned int k = M; k < M + N; k++)
//            {
//                sBundle_RN->values[k] = sFiber->values[k - M];
//            }
//            break;
//        }
//        case SE3RN_SE3:
//        {
//            base::SE3StateSpace::StateType *sBundle_SE3 =
//                qBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//            base::SO3StateSpace::StateType *sBundle_SE3_rotation = &sBundle_SE3->rotation();
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::SE3StateSpace::StateType *sBase = qBase->as<base::SE3StateSpace::StateType>();
//            const base::SO3StateSpace::StateType *sBase_rotation = &sBase->rotation();
//            const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

//            sBundle_SE3->setXYZ(sBase->getX(), sBase->getY(), sBase->getZ());
//            sBundle_SE3_rotation->x = sBase_rotation->x;
//            sBundle_SE3_rotation->y = sBase_rotation->y;
//            sBundle_SE3_rotation->z = sBase_rotation->z;
//            sBundle_SE3_rotation->w = sBase_rotation->w;

//            for (unsigned int k = 0; k < Fiber_dimension_; k++)
//            {
//                sBundle_RN->values[k] = sFiber->values[k];
//            }

//            break;
//        }
//        case SE3RN_SE3RM:
//        {
//            base::SE3StateSpace::StateType *sBundle_SE3 =
//                qBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//            base::SO3StateSpace::StateType *sBundle_SE3_rotation = &sBundle_SE3->rotation();
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::SE3StateSpace::StateType *sBase_SE3 =
//                qBase->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//            const base::SO3StateSpace::StateType *sBase_SE3_rotation = &sBase_SE3->rotation();
//            const base::RealVectorStateSpace::StateType *sBase_RM =
//                qBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

//            sBundle_SE3->setXYZ(sBase_SE3->getX(), sBase_SE3->getY(), sBase_SE3->getZ());
//            sBundle_SE3_rotation->x = sBase_SE3_rotation->x;
//            sBundle_SE3_rotation->y = sBase_SE3_rotation->y;
//            sBundle_SE3_rotation->z = sBase_SE3_rotation->z;
//            sBundle_SE3_rotation->w = sBase_SE3_rotation->w;

//            //[X Y Z YAW PITCH ROLL] [1...M-1][M...N-1]
//            // SE3                                        RN
//            unsigned int M = Bundle_dimension_ - Fiber_dimension_ - 6;
//            unsigned int N = Fiber_dimension_;

//            for (unsigned int k = 0; k < M; k++)
//            {
//                sBundle_RN->values[k] = sBase_RM->values[k];
//            }
//            for (unsigned int k = M; k < M + N; k++)
//            {
//                sBundle_RN->values[k] = sFiber->values[k - M];
//            }
//            break;
//        }
//        default:
//        {
//            OMPL_ERROR("Type %d not implemented.", qtype);
//            throw ompl::Exception("Cannot merge states.");
//        }
//    }
//}

void ompl::geometric::BundleSpace::projectFiber(const base::State *xBundle, base::State *xFiber) const
{
    unsigned int M = components_.size();

    if( M > 1){
        for(uint m = 0; m < M; m++){
            const base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
            base::State *xmFiber = xFiber->as<base::CompoundState>()->as<base::State>(m);
            components_.at(m)->projectFiber(xmBundle, xmFiber);
        }
    }else{
        components_.front()->projectFiber(xBundle, xFiber);
    }


    // switch (type_)
    // {
    //     case RN_RM:
    //     {
    //         const base::RealVectorStateSpace::StateType *sBundle = q->as<base::RealVectorStateSpace::StateType>();
    //         base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

    //         for (unsigned int k = Base_dimension_; k < Bundle_dimension_; k++)
    //         {
    //             sFiber->values[k - Base_dimension_] = sBundle->values[k];
    //         }
    //         break;
    //     }
    //     case SE2_R2:
    //     {
    //         const base::SE2StateSpace::StateType *sBundle = q->as<base::SE2StateSpace::StateType>();
    //         base::SO2StateSpace::StateType *sFiber = qFiber->as<base::SO2StateSpace::StateType>();
    //         sFiber->value = sBundle->getYaw();
    //         break;
    //     }
    //     case SE3_R3:
    //     {
    //         const base::SE3StateSpace::StateType *sBundle = q->as<base::SE3StateSpace::StateType>();
    //         const base::SO3StateSpace::StateType *sBundle_SO3 = &sBundle->rotation();

    //         base::SO3StateSpace::StateType *sFiber_SO3 = qFiber->as<base::SO3StateSpace::StateType>();

    //         sFiber_SO3->x = sBundle_SO3->x;
    //         sFiber_SO3->y = sBundle_SO3->y;
    //         sFiber_SO3->z = sBundle_SO3->z;
    //         sFiber_SO3->w = sBundle_SO3->w;

    //         break;
    //     }
    //     case SE3RN_R3:
    //     {
    //         const base::SE3StateSpace::StateType *sBundle_SE3 =
    //             q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    //         const base::SO3StateSpace::StateType *sBundle_SO3 = &sBundle_SE3->rotation();
    //         const base::RealVectorStateSpace::StateType *sBundle_RN =
    //             q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    //         base::SO3StateSpace::StateType *sFiber_SO3 =
    //             qFiber->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    //         base::RealVectorStateSpace::StateType *sFiber_RN =
    //             qFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    //         sFiber_SO3->x = sBundle_SO3->x;
    //         sFiber_SO3->y = sBundle_SO3->y;
    //         sFiber_SO3->z = sBundle_SO3->z;
    //         sFiber_SO3->w = sBundle_SO3->w;
    //         for (unsigned int k = 0; k < Fiber_dimension_ - 3; k++)
    //         {
    //             sFiber_RN->values[k] = sBundle_RN->values[k];
    //         }

    //         break;
    //     }
    //     case SE2RN_R2:
    //     {
    //         const base::SE2StateSpace::StateType *sBundle_SE2 =
    //             q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    //         const base::RealVectorStateSpace::StateType *sBundle_RN =
    //             q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    //         base::SO2StateSpace::StateType *sFiber_SO2 =
    //             qFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    //         base::RealVectorStateSpace::StateType *sFiber_RN =
    //             qFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    //         sFiber_SO2->value = sBundle_SE2->getYaw();
    //         for (unsigned int k = 0; k < Fiber_dimension_ - 1; k++)
    //         {
    //             sFiber_RN->values[k] = sBundle_RN->values[k];
    //         }
    //         break;
    //     }
    //     case SE2RN_SE2RM:
    //     case SO2RN_SO2RM:
    //     {
    //         const base::RealVectorStateSpace::StateType *sBundle_RN =
    //             q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    //         const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

    //         unsigned int N = Bundle_dimension_ - Fiber_dimension_ - 3;
    //         for (unsigned int k = N; k < Bundle_dimension_ - 3; k++)
    //         {
    //             sFiber->values[k - N] = sBundle_RN->values[k];
    //         }
    //         break;
    //     }
    //     case SE2RN_SE2:
    //     case SE3RN_SE3:
    //     case SO2RN_SO2:
    //     {
    //         const base::RealVectorStateSpace::StateType *sBundle_RN =
    //             q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
    //         base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

    //         for (unsigned int k = 0; k < Fiber_dimension_; k++)
    //         {
    //             sFiber->values[k] = sBundle_RN->values[k];
    //         }

    //         break;
    //     }
    //     case SE3RN_SE3RM:
    //     {
    //         const base::RealVectorStateSpace::StateType *sBundle_RN =
    //             q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    //         const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

    //         unsigned int N = Bundle_dimension_ - Fiber_dimension_ - 6;
    //         for (unsigned int k = N; k < Bundle_dimension_ - 6; k++)
    //         {
    //             sFiber->values[k - N] = sBundle_RN->values[k];
    //         }
    //         break;
    //     }
    //     default:
    //     {
    //         OMPL_ERROR("Type %d not implemented.", type_);
    //         throw ompl::Exception("Cannot project onto Fiber.");
    //     }
    // }
}

void ompl::geometric::BundleSpace::projectBase(const base::State *xBundle, base::State *xBase) const
{

    unsigned int M = components_.size();

    if( M > 1){
        for(uint m = 0; m < M; m++){
            const base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
            base::State *xmBase = xBase->as<base::CompoundState>()->as<base::State>(m);
            components_.at(m)->projectBase(xmBundle, xmBase);
        }
    }else{
        components_.front()->projectBase(xBundle, xBase);
    }

    //if (type_ == MULTIAGENT)
    //{
    //    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
    //    const base::StateSpacePtr Base_space = parent_->getSpaceInformation()->getStateSpace();

    //    base::CompoundStateSpace *Bundle_compound = Bundle_space->as<base::CompoundStateSpace>();
    //    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    //    base::CompoundStateSpace *Base_compound = Base_space->as<base::CompoundStateSpace>();
    //    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    //    //splitState
    //    for(uint k = 0; k < Bundle_decomposed.size(); k++){
    //      base::StateSpacePtr Bundlek = Bundle_decomposed.at(k);
    //      base::StateSpacePtr Basek = Base_decomposed.at(k);
    //      BundleType typek = types_.at(k);
    //      const base::State *qk = q->as<base::CompoundState>()->as<base::State>(k);
    //      base::State *qkBase = qBase->as<base::CompoundState>()->as<base::State>(k);
    //      switch(typek)
    //      {
    //        case IDENTITY_SPACE_RN:
    //        case IDENTITY_SPACE_SE2:
    //        case IDENTITY_SPACE_SE2RN:
    //        case IDENTITY_SPACE_SO2RN:
    //        case IDENTITY_SPACE_SE3:
    //        case IDENTITY_SPACE_SE3RN:
    //        {
    //          Bundlek->copyState(qkBase, qk);
    //          break;
    //        }
    //        case RN_RM:
    //        {
    //            const base::RealVectorStateSpace::StateType *sBundle = qk->as<base::RealVectorStateSpace::StateType>();
    //            base::RealVectorStateSpace::StateType *sBase = qkBase->as<base::RealVectorStateSpace::StateType>();
    //            for (unsigned int k = 0; k < Basek->getDimension(); k++)
    //            {
    //                sBase->values[k] = sBundle->values[k];
    //            }
    //          break;
    //        }
    //        default:
    //        {
    //          projectBase(qk, qkBase, typek);
    //          break;
    //        }
    //      }
    //    }
    //    return;
    //}else{
    //    return projectBase(q, qBase, type_);
    //}
}

void ompl::geometric::BundleSpace::allocIdentityState(base::State *s, base::StateSpacePtr space) const
{
  if(space->isCompound()){
    base::CompoundStateSpace *cspace = space->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> compounds = cspace->getSubspaces();
    for(unsigned int k = 0; k < compounds.size(); k++){
      base::StateSpacePtr spacek = compounds.at(k);
      base::State *sk = s->as<base::CompoundState>()->as<base::State>(k);
      allocIdentityState(sk, spacek);
    }
  }else{
    int stype = space->getType();
    switch (stype) 
    {
      case base::STATE_SPACE_SO3:
      {
        static_cast<base::SO3StateSpace::StateType *>(s)->setIdentity();
        break;
      }
      case base::STATE_SPACE_SO2:
      {
        static_cast<base::SO2StateSpace::StateType *>(s)->setIdentity();
        break;
      }
      case base::STATE_SPACE_REAL_VECTOR:
      {
        base::RealVectorStateSpace::StateType *sRN = s->as<base::RealVectorStateSpace::StateType>();
        for(uint k = 0; k < space->getDimension(); k++){
          sRN->values[k] = 0;
        }
        break;
      }
      default:
      {
        OMPL_ERROR("Type: %d not recognized.", stype);
        exit(0);
      }
    }
  }
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityState(base::StateSpacePtr space) const
{
  if(space != nullptr){
    base::State *s = space->allocState();
    allocIdentityState(s, space);
    return s;
  }else{
    return nullptr;
  }
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityStateFiber() const
{
  return allocIdentityState(Fiber->getStateSpace());
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityStateBundle() const
{
  return allocIdentityState(Bundle->getStateSpace());
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityStateBase() const
{
  return allocIdentityState(Base->getStateSpace());
}

// void ompl::geometric::BundleSpace::projectBase(
//     const base::State *q, 
//     base::State *qBase,
//     BundleType type) const
// {
//     switch (type)
//     {
//         case MULTIAGENT:
//             OMPL_ERROR("NYI");
//             throw ompl::Exception("NYI");
//         case EMPTY_SET_PROJECTION:
//             break;
//         case IDENTITY_SPACE_RN:
//         case IDENTITY_SPACE_SE2:
//         case IDENTITY_SPACE_SE2RN:
//         case IDENTITY_SPACE_SO2RN:
//         case IDENTITY_SPACE_SE3:
//         case IDENTITY_SPACE_SE3RN:
//         {
//             // Identity function
//             Bundle->getStateSpace()->copyState(qBase, q);
//             break;
//         }
//         case RN_RM:
//         {
//             const base::RealVectorStateSpace::StateType *sBundle = q->as<base::RealVectorStateSpace::StateType>();
//             base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();

//             for (unsigned int k = 0; k < Base_dimension_; k++)
//             {
//                 sBase->values[k] = sBundle->values[k];
//             }
//             break;
//         }
//         case SE2_R2:
//         {
//             const base::SE2StateSpace::StateType *sBundle = q->as<base::SE2StateSpace::StateType>();
//             base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();
//             sBase->values[0] = sBundle->getX();
//             sBase->values[1] = sBundle->getY();
//             break;
//         }
//         case SE2RN_R2:
//         {
//             const base::SE2StateSpace::StateType *sBundle =
//                 q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//             base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();
//             sBase->values[0] = sBundle->getX();
//             sBase->values[1] = sBundle->getY();
//             break;
//         }
//         case SE2RN_SE2:
//         {
//             const base::SE2StateSpace::StateType *sBundle_SE2 =
//                 q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//             base::SE2StateSpace::StateType *sBase_SE2 = qBase->as<base::SE2StateSpace::StateType>();

//             sBase_SE2->setX(sBundle_SE2->getX());
//             sBase_SE2->setY(sBundle_SE2->getY());
//             sBase_SE2->setYaw(sBundle_SE2->getYaw());

//             break;
//         }
//         case SO2RN_SO2:
//         {
//             const base::SO2StateSpace::StateType *sBundle_SO2 =
//                 q->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
//             base::SO2StateSpace::StateType *sBase_SO2 = qBase->as<base::SO2StateSpace::StateType>();

//             sBase_SO2->value = sBundle_SO2->value;

//             break;
//         }
//         case SO2RN_SO2RM:
//         {
//             const base::SO2StateSpace::StateType *sBundle_SO2 =
//                 q->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
//             const base::RealVectorStateSpace::StateType *sBundle_RN =
//                 q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//             base::SO2StateSpace::StateType *sBase_SO2 =
//                 qBase->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
//             base::RealVectorStateSpace::StateType *sBase_RM =
//                 qBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//             sBase_SO2->value = sBundle_SO2->value;

//             for (unsigned int k = 0; k < Base_dimension_ - 1; k++)
//             {
//                 sBase_RM->values[k] = sBundle_RN->values[k];
//             }
//             break;
//         }

//         case SE2RN_SE2RM:
//         {
//             const base::SE2StateSpace::StateType *sBundle_SE2 =
//                 q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//             const base::RealVectorStateSpace::StateType *sBundle_RN =
//                 q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//             base::SE2StateSpace::StateType *sBase_SE2 =
//                 qBase->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
//             base::RealVectorStateSpace::StateType *sBase_RN =
//                 qBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//             sBase_SE2->setX(sBundle_SE2->getX());
//             sBase_SE2->setY(sBundle_SE2->getY());
//             sBase_SE2->setYaw(sBundle_SE2->getYaw());

//             for (unsigned int k = 0; k < Base_dimension_ - 3; k++)
//             {
//                 sBase_RN->values[k] = sBundle_RN->values[k];
//             }
//             break;
//         }
//         case SE3_R3:
//         {
//             const base::SE3StateSpace::StateType *sBundle = q->as<base::SE3StateSpace::StateType>();
//             base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();

//             sBase->values[0] = sBundle->getX();
//             sBase->values[1] = sBundle->getY();
//             sBase->values[2] = sBundle->getZ();

//             break;
//         }
//         case SE3RN_R3:
//         {
//             const base::SE3StateSpace::StateType *sBundle_SE3 =
//                 q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//             base::RealVectorStateSpace::StateType *sBase = qBase->as<base::RealVectorStateSpace::StateType>();

//             sBase->values[0] = sBundle_SE3->getX();
//             sBase->values[1] = sBundle_SE3->getY();
//             sBase->values[2] = sBundle_SE3->getZ();

//             break;
//         }
//         case SE3RN_SE3:
//         {
//             const base::SE3StateSpace::StateType *sBundle_SE3 =
//                 q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//             const base::SO3StateSpace::StateType *sBundle_SE3_rotation = &sBundle_SE3->rotation();

//             base::SE3StateSpace::StateType *sBase = qBase->as<base::SE3StateSpace::StateType>();
//             base::SO3StateSpace::StateType *sBase_rotation = &sBase->rotation();

//             sBase->setXYZ(sBundle_SE3->getX(), sBundle_SE3->getY(), sBundle_SE3->getZ());
//             sBase_rotation->x = sBundle_SE3_rotation->x;
//             sBase_rotation->y = sBundle_SE3_rotation->y;
//             sBase_rotation->z = sBundle_SE3_rotation->z;
//             sBase_rotation->w = sBundle_SE3_rotation->w;

//             break;
//         }
//         case SE3RN_SE3RM:
//         {
//             const base::SE3StateSpace::StateType *sBundle_SE3 =
//                 q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//             const base::SO3StateSpace::StateType *sBundle_SE3_rotation = &sBundle_SE3->rotation();
//             const base::RealVectorStateSpace::StateType *sBundle_RN =
//                 q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//             base::SE3StateSpace::StateType *sBase_SE3 =
//                 qBase->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//             base::SO3StateSpace::StateType *sBase_rotation = &sBase_SE3->rotation();
//             base::RealVectorStateSpace::StateType *sBase_RN =
//                 qBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//             sBase_SE3->setXYZ(sBundle_SE3->getX(), sBundle_SE3->getY(), sBundle_SE3->getZ());
//             sBase_rotation->x = sBundle_SE3_rotation->x;
//             sBase_rotation->y = sBundle_SE3_rotation->y;
//             sBase_rotation->z = sBundle_SE3_rotation->z;
//             sBase_rotation->w = sBundle_SE3_rotation->w;

//             for (unsigned int k = 0; k < Base_dimension_ - 6; k++)
//             {
//                 sBase_RN->values[k] = sBundle_RN->values[k];
//             }
//             break;
//         }
//         default:
//         {
//             OMPL_ERROR("Cannot project onto Base. Type %d not implemented.", type_);
//             throw ompl::Exception("Cannot project onto Base.");
//         }
//     }
// }

const ompl::base::SpaceInformationPtr &ompl::geometric::BundleSpace::getFiber() const
{
    return Fiber;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::BundleSpace::getBundle() const
{
    return Bundle;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::BundleSpace::getBase() const
{
    return Base;
}

unsigned int ompl::geometric::BundleSpace::getFiberDimension() const
{
    return getFiber()->getStateDimension();
}

unsigned int ompl::geometric::BundleSpace::getBaseDimension() const
{
    return getBase()->getStateDimension();
}

unsigned int ompl::geometric::BundleSpace::getDimension() const
{
    return getBundle()->getStateDimension();
}

const ompl::base::StateSamplerPtr &ompl::geometric::BundleSpace::getFiberSamplerPtr() const
{
    return Fiber_sampler_;
}

const ompl::base::StateSamplerPtr &ompl::geometric::BundleSpace::getBundleSamplerPtr() const
{
    return Bundle_sampler_;
}

bool ompl::geometric::BundleSpace::hasSolution()
{
    if (!hasSolution_)
    {
        base::PathPtr path;
        hasSolution_ = getSolution(path);
    }
    return hasSolution_;
}

// unsigned int ompl::geometric::BundleSpace::getTotalNumberOfSamples() const
// {
//     return totalNumberOfSamples_;
// }

// unsigned int ompl::geometric::BundleSpace::getTotalNumberOfFeasibleSamples() const
// {
//     return totalNumberOfFeasibleSamples_;
// }

ompl::geometric::BundleSpace *ompl::geometric::BundleSpace::getParent() const
{
    return parent_;
}

ompl::geometric::BundleSpace *ompl::geometric::BundleSpace::getChild() const
{
    return child_;
}

void ompl::geometric::BundleSpace::setChild(ompl::geometric::BundleSpace *child)
{
    child_ = child;
}

void ompl::geometric::BundleSpace::setParent(ompl::geometric::BundleSpace *parent)
{
    parent_ = parent;
}

unsigned int ompl::geometric::BundleSpace::getLevel() const
{
    return level_;
}

void ompl::geometric::BundleSpace::setLevel(unsigned int level)
{
    level_ = level;
}

ompl::base::OptimizationObjectivePtr ompl::geometric::BundleSpace::getOptimizationObjectivePtr() const
{
    return opt_;
}

bool ompl::geometric::BundleSpace::sampleBase(base::State *xRandom)
{
    Bundle_sampler_->sampleUniform(xRandom);
    return true;
}

bool ompl::geometric::BundleSpace::sample(base::State *xRandom)
{
    bool valid = false;
    if (!hasParent())
    {
        // return Bundle_valid_sampler->sample(q_random);
        Bundle_sampler_->sampleUniform(xRandom);
        valid = Bundle->isValid(xRandom);
    }
    else
    {
        if (getFiberDimension() > 0)
        {
            // Adjusted sampling function: Sampling in G0 x Fiber
            Fiber_sampler_->sampleUniform(xFiberTmp_);
            parent_->sampleBase(xBaseTmp_);
            mergeStates(xBaseTmp_, xFiberTmp_, xRandom);
        }
        else
        {
            parent_->sampleBase(xRandom);
        }
        valid = Bundle->isValid(xRandom);
    }
    // totalNumberOfSamples_++;
    // if (valid)
    // {
    //     totalNumberOfFeasibleSamples_++;
    // }

    return valid;
}

void ompl::geometric::BundleSpace::print(std::ostream &out) const
{
    unsigned int M = components_.size();
    for(unsigned int m = 0; m < M; m++){
        out << components_.at(m)->getTypeAsString() << (m<M-1?"|":"");
    }
    out << std::endl;
}

// void ompl::geometric::BundleSpace::print(std::ostream &out) const
// {
//     out << "[BundleSpace: id" << id_ << " |lvl" << level_ << "] ";
//     unsigned int sublevel = std::max(1U, level_);
//     if (!hasParent())
//     {
//         out << "X" << sublevel << "=Q" << sublevel << ": ";
//         if (Bundle->getStateSpace()->getType() == base::STATE_SPACE_SE2)
//         {
//             out << "SE(2)";
//         }
//         else if (Bundle->getStateSpace()->getType() == base::STATE_SPACE_SE3)
//         {
//             out << "SE(3)";
//         }
//         else if (Bundle->getStateSpace()->getType() == base::STATE_SPACE_REAL_VECTOR)
//         {
//             out << "R^" << Bundle->getStateDimension();
//         }
//         else
//         {
//             out << "unknown";
//         }
//     }
//     else
//     {
//         out << "X" << sublevel << "=Q" << sublevel << ": ";
//         switch (type_)
//         {
//             case BundleSpace::IDENTITY_SPACE_RN:
//             {
//                 out << "R^" << Base_dimension_ << " | Q" << level_ + 1 << ": R^" << Bundle_dimension_;
//                 break;
//             }
//             case BundleSpace::IDENTITY_SPACE_SE2:
//             {
//                 out << "SE(2)"
//                     << " | Q" << level_ + 1 << ": SE(2)";
//                 break;
//             }
//             case BundleSpace::IDENTITY_SPACE_SE2RN:
//             {
//                 out << "SE(2)xR^" << Base_dimension_ << " | Q" << level_ + 1 << ": SE(2)xR^" << Bundle_dimension_;
//                 break;
//             }
//             case BundleSpace::IDENTITY_SPACE_SO2RN:
//             {
//                 out << "SO(2)xR^" << Base_dimension_ << " | Q" << level_ + 1 << ": SO(2)xR^" << Bundle_dimension_;
//                 break;
//             }
//             case BundleSpace::IDENTITY_SPACE_SE3:
//             {
//                 out << "SE(3)"
//                     << " | Q" << level_ + 1 << ": SE(3)";
//                 break;
//             }
//             case BundleSpace::IDENTITY_SPACE_SE3RN:
//             {
//                 out << "SE(3)xR^" << Base_dimension_ << " | Q" << level_ + 1 << ": SE(3)xR^" << Bundle_dimension_;
//                 break;
//             }
//             case BundleSpace::RN_RM:
//             {
//                 out << "R^" << Base_dimension_ << " | Q" << level_ + 1 << ": R^" << Bundle_dimension_ << " | X" << level_ + 1
//                     << ": R^" << Bundle_dimension_ - Base_dimension_;
//                 break;
//             }
//             case BundleSpace::SE2_R2:
//             {
//                 out << "R^2 | Q" << level_ + 1 << ": SE(2) | X" << level_ + 1 << ": SO(2)";
//                 break;
//             }
//             case BundleSpace::SE3_R3:
//             {
//                 out << "R^3 | Q" << level_ + 1 << ": SE(3) | X" << level_ + 1 << ": SO(3)";
//                 break;
//             }
//             case BundleSpace::SE2RN_SE2:
//             {
//                 out << "SE(2) | Q" << level_ + 1 << ": SE(2)xR^" << Fiber_dimension_ << " | X" << level_ + 1 << ": R^"
//                     << Fiber_dimension_;
//                 break;
//             }
//             case BundleSpace::SO2RN_SO2:
//             {
//                 out << "SO(2) | Q" << level_ + 1 << ": SO(2)xR^" << Fiber_dimension_ << " | X" << level_ + 1 << ": R^"
//                     << Fiber_dimension_;
//                 break;
//             }
//             case BundleSpace::SE3RN_SE3:
//             {
//                 out << "SE(3) | Q" << level_ + 1 << ": SE(3)xR^" << Fiber_dimension_ << " | X" << level_ + 1 << ": R^"
//                     << Fiber_dimension_;
//                 break;
//             }
//             case BundleSpace::SE2RN_SE2RM:
//             {
//                 out << "SE(2)xR^" << Base_dimension_ - 3 << " | Q" << level_ + 1 << ": SE(2)xR^" << Bundle_dimension_ - 3
//                     << " | X" << level_ + 1 << ": R^" << Fiber_dimension_;
//                 break;
//             }
//             case BundleSpace::SO2RN_SO2RM:
//             {
//                 out << "SO(2)xR^" << Base_dimension_ - 1 << " | Q" << level_ + 1 << ": SO(2)xR^" << Bundle_dimension_ - 1
//                     << " | X" << level_ + 1 << ": R^" << Fiber_dimension_;
//                 break;
//             }
//             case BundleSpace::SE3RN_SE3RM:
//             {
//                 out << "SE(3)xR^" << Base_dimension_ - 6 << " | Q" << level_ + 1 << ": SE(3)xR^" << Bundle_dimension_ - 6
//                     << " | X" << level_ + 1 << ": R^" << Fiber_dimension_;
//                 break;
//             }
//             case BundleSpace::MULTIAGENT:
//             {
//                 out << "Multiagent(" << types_.size() << ")";
//                 break;
//             }
//             default:
//             {
//                 out << "unknown type_: " << type_;
//             }
//         }
//     }
//     out << std::endl << " --[Importance:" << getImportance() << "]";
//     out << std::endl << " --[Measure   :" << Bundle->getSpaceMeasure() << "]";
// }

namespace ompl
{
    namespace geometric
    {
        std::ostream &operator<<(std::ostream &out, const BundleSpace &quotient_)
        {
            quotient_.print(out);
            return out;
        }
    }  // namespace geometric
}  // namespace ompl
