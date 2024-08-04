/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

Head::Head(const PathRestrictionPtr& restriction, TreeNode* xCurrent, const ompl::base::State *xTarget)
{
    restriction_ = restriction;
    xCurrent_ = xCurrent;

    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(restriction_->getProjection());

    auto bundle = projection->getBundle();
    xTarget_ = bundle->allocState();
    bundle->copyState(xTarget_, xTarget);

    if (projection->getBaseDimension() > 0)
    {
        auto base = projection->getBase();
        xBaseCurrent_ = base->allocState();
        projection->project(xCurrent_->getState(), xBaseCurrent_);
    }
    if (projection->getCoDimension() > 0)
    {
        base::StateSpacePtr fiber = projection->getFiberSpace();
        xFiberCurrent_ = fiber->allocState();
        xFiberTarget_ = fiber->allocState();
        projection->projectFiber(xCurrent_->getState(), xFiberCurrent_);
        projection->projectFiber(xTarget_, xFiberTarget_);
    }
}

Head::Head(const Head &rhs)
{
    xCurrent_ = rhs.getTreeNode();
    xTarget_ = rhs.getTargetState();
    restriction_ = rhs.getRestriction();

    locationOnBasePath_ = rhs.getLocationOnBasePath();
    lastValidIndexOnBasePath_ = rhs.getLastValidBasePathIndex();

    xFiberCurrent_ = rhs.getStateFiberNonConst();
    xBaseCurrent_ = rhs.getStateBaseNonConst();
    xFiberTarget_ = rhs.getStateTargetFiberNonConst();
}

Head::~Head()
{
    std::stringstream buffer;
    buffer << *this;
    OMPL_DEVMSG1("Last head before termination: %s.", buffer.str().c_str());

    auto projection = restriction_->getProjection();
    if (projection->getCoDimension() > 0)
    {
        FiberedProjectionPtr fibered_projection = std::static_pointer_cast<FiberedProjection>(projection);
        base::StateSpacePtr fiber = fibered_projection->getFiberSpace();
        fiber->freeState(xFiberCurrent_);
        fiber->freeState(xFiberTarget_);
    }

    if (projection->getBaseDimension() > 0)
    {
        auto base = projection->getBase();
        base->freeState(xBaseCurrent_);
    }

    //auto bundle = projection->getBundle();
    //bundle->freeState(xCurrent_->getState());
    //bundle->freeState(xTarget_);

}

PathRestrictionPtr Head::getRestriction() const
{
    return restriction_;
}

ompl::base::State* Head::getState() const
{
    return xCurrent_->getState();
}

TreeNode* Head::getTreeNode() const
{
    return xCurrent_;
}

const ompl::base::State* Head::getStateFiber() const
{
    return xFiberCurrent_;
}

const ompl::base::State* Head::getStateBase() const
{
    return xBaseCurrent_;
}

ompl::base::State* Head::getStateFiberNonConst() const
{
    return xFiberCurrent_;
}

ompl::base::State* Head::getStateBaseNonConst() const
{
    return xBaseCurrent_;
}

ompl::base::State* Head::getTargetState() const
{
    return xTarget_;
}

const ompl::base::State *Head::getStateTargetFiber() const
{
    return xFiberTarget_;
}

ompl::base::State *Head::getStateTargetFiberNonConst() const
{
    return xFiberTarget_;
}

void Head::setCurrent(TreeNode* newCurrent, double location)
{
    auto projection = restriction_->getProjection();

    xCurrent_ = newCurrent;
    projection->getBundle()->copyState(xCurrent_->getState(), newCurrent->getState());

    locationOnBasePath_ = location;

    lastValidIndexOnBasePath_ = restriction_->getBasePathLastIndexFromLocation(location);

    if (projection->getBaseDimension() > 0)
    {
        auto base = projection->getBase();
        projection->project(xCurrent_->getState(), xBaseCurrent_);
    }
    if (projection->getCoDimension() > 0)
    {
        FiberedProjectionPtr fibered_projection = std::static_pointer_cast<FiberedProjection>(projection);
        fibered_projection->projectFiber(xCurrent_->getState(), xFiberCurrent_);
    }
}

int Head::getLastValidBasePathIndex() const
{
    return lastValidIndexOnBasePath_;
}

int Head::getNextValidBasePathIndex() const
{
    int Nlast = getRestriction()->size() - 1;
    if (lastValidIndexOnBasePath_ < Nlast)
    {
        return lastValidIndexOnBasePath_ + 1;
    }
    else
    {
        return Nlast;
    }
}

double Head::getLocationOnBasePath() const
{
    return locationOnBasePath_;
}

int Head::getNumberOfRemainingStatesOnBasePath()
{
    //----- | ---------------X-------|---------|
    //    lastValid        xCurrent
    //    would result in three (including current head)

    if (getLocationOnBasePath() >= restriction_->getLengthBasePath())
    {
        return 1;
    }

    int Nstates = restriction_->getBasePath().size();
    return std::max(1, Nstates - (lastValidIndexOnBasePath_ + 1));
}

void Head::setLocationOnBasePath(double d)
{
    locationOnBasePath_ = d;
}

void Head::setLastValidBasePathIndex(int k)
{
    lastValidIndexOnBasePath_ = k;
}

const ompl::base::State *Head::getBaseStateAt(int k) const
{
    //----- | ---------------X-------|---------
    //    lastValid        xCurrent    basePath(lastValid + 1)
    if (k <= 0)
    {
        return xBaseCurrent_;
    }
    else
    {
        int idx = std::min(restriction_->size() - 1, (unsigned int)lastValidIndexOnBasePath_ + k);
        return restriction_->getBasePath().at(idx);
    }
}

int Head::getBaseStateIndexAt(int k) const
{
    //----- | ---------------X-------|---------
    //    lastValid        xCurrent    basePath(lastValid + 1)

    unsigned int idx = lastValidIndexOnBasePath_ + k;
    if (restriction_->size() < 1)
    {
        throw ompl::Exception("EmptyRestriction");
    }
    if (idx > restriction_->size() - 1)
    {
        idx = restriction_->size() - 1;
    }
    return idx;
}

void Head::print(std::ostream &out) const
{
    auto projection = restriction_->getProjection();
    auto bundle = projection->getBundle();
    auto base = projection->getBase();

    out << std::endl << "[ Head at:";
    int idx = getLastValidBasePathIndex();
    bundle->printState(xCurrent_->getState(), out);
    out << "base location " << getLocationOnBasePath() << "/" << restriction_->getLengthBasePath() << " idx " << idx
        << "/" << restriction_->size() << std::endl;
    out << "last base state idx ";
    base->printState(restriction_->getBasePath().at(idx), out);
    out << "]";
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const Head &h)
        {
            h.print(out);
            return out;
        }
    }
}
