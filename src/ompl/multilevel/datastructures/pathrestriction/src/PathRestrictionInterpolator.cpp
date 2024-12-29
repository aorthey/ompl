#include <ompl/multilevel/datastructures/pathrestriction/PathRestrictionInterpolator.h>

#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

namespace ompl {
namespace multilevel {

PathSectionPtr interpolateL1FiberFirst(const PathRestrictionPtr& restriction, const HeadPtr& head)
{
    PathSectionPtr section = std::make_shared<PathSection>(restriction);
    auto projection = std::static_pointer_cast<FiberedProjection>(restriction->getProjection());

    auto base = projection->getBase();
    auto bundle = projection->getBundle();

    int size = head->getNumberOfRemainingStatesOnBasePath() + 1;

    if (projection->getCoDimension() > 0)
    {
        const ompl::base::State *xFiberStart = head->getStateFiber();
        const ompl::base::State *xFiberGoal = head->getStateTargetFiber();

        section->resize(size + 1);

        projection->lift(head->getBaseStateAt(0), xFiberStart, section->frontNonConst());

        section->addBaseStateIndex(head->getBaseStateIndexAt(0));

        for (unsigned int k = 1; k < section->size(); k++)
        {
            projection->lift(head->getBaseStateAt(k - 1), xFiberGoal, section->atNonConst(k));
            section->addBaseStateIndex(head->getBaseStateIndexAt(k - 1));
        }
    }
    else
    {
        section->resize(size);

        for (int k = 0; k < size; k++)
        {
            bundle->copyState(section->atNonConst(k), head->getBaseStateAt(k));
            section->addBaseStateIndex(head->getBaseStateIndexAt(k));
        }
    }
    return section;
}

PathSectionPtr interpolateL1FiberLast(const PathRestrictionPtr& restriction, const HeadPtr& head)
{
  PathSectionPtr section = std::make_shared<PathSection>(restriction);

  auto projection = std::static_pointer_cast<FiberedProjection>(restriction->getProjection());
  const auto bundle = projection->getBundle();
  const auto base = projection->getBase();

  int size = head->getNumberOfRemainingStatesOnBasePath() + 1; //remaining + current state

  if (projection->getCoDimension() > 0)
  {
      const ompl::base::State *xFiberStart = head->getStateFiber();
      const ompl::base::State *xFiberGoal = head->getStateTargetFiber();

      section->resize(size + 1);

      for (int k = 0; k < size; k++)
      {
          projection->lift(head->getBaseStateAt(k), xFiberStart, section->atNonConst(k));
          section->addBaseStateIndex(head->getBaseStateIndexAt(k));
      }
      projection->lift(head->getBaseStateAt(size - 1), xFiberGoal, section->backNonConst());
      section->addBaseStateIndex(head->getBaseStateIndexAt(size - 1));
  }
  else
  {
      section->resize(size);
      for (int k = 0; k < size; k++)
      {
          bundle->copyState(section->atNonConst(k), head->getBaseStateAt(k));
          section->addBaseStateIndex(head->getBaseStateIndexAt(k));
      }
  }
  return section;
}

PathSectionPtr interpolateL2(const PathRestrictionPtr& restriction, const HeadPtr& head)
{
    PathSectionPtr section = std::make_shared<PathSection>(restriction);

    auto projection = std::static_pointer_cast<FiberedProjection>(restriction->getProjection());
    auto bundle = projection->getBundle();
    const auto& basePath = restriction->getBasePath();

    int size = head->getNumberOfRemainingStatesOnBasePath() + 1;

    section->resize(size);

    if (projection->getCoDimension() > 0)
    {
        const ompl::base::State *xFiberStart = head->getStateFiber();
        const ompl::base::State *xFiberGoal = head->getStateTargetFiber();

        auto fiber = projection->getFiber();
        auto xFiberTmp = fiber->allocState();

        double totalLengthBasePath = restriction->getLengthBasePath();

        for (unsigned int k = 0; k < restriction->size(); k++)
        {
            double lengthCurrent = restriction->getLengthBasePathUntil(k);
            double step = lengthCurrent / totalLengthBasePath;

            fiber->interpolate(xFiberStart, xFiberGoal, step, xFiberTmp);

            projection->lift(restriction->getBaseStateAt(k), xFiberTmp, section->atNonConst(k));

            section->addBaseStateIndex(head->getBaseStateIndexAt(k));
        }
        fiber->freeState(xFiberTmp);
    }
    else
    {
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            bundle->copyState(section->atNonConst(k), basePath.at(k));
            section->addBaseStateIndex(head->getBaseStateIndexAt(k));
        }
    }
    return section;
}
}
}
