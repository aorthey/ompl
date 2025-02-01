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

#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>
#include <ompl/multilevel/datastructures/Tree.h>

using namespace ompl::multilevel;

void allocStates(const ompl::base::StateSpacePtr& space, std::vector<ompl::base::State*>& states) {
    for (auto &state : states){
        state = space->allocState();
    }
}

void freeStates(const ompl::base::StateSpacePtr& space, std::vector<ompl::base::State*>& states) {
    for (auto &state : states){
        space->freeState(state);
    }
}

PathSection::PathSection(const std::vector<ompl::base::State*>& states) : section_states_(states) {
}

PathSection::PathSection(const PathRestrictionPtr& restriction) : restriction_(restriction)
{
    auto projection = restriction_->getProjection();
    if (projection->getBaseDimension() > 0)
    {
        auto base = projection->getBase();
        xBaseTmp_ = base->allocState();
    }
    auto bundle = projection->getBundle();
    lastValid_.first = bundle->allocState();
}

PathSection::~PathSection()
{
    if(restriction_ == nullptr) {
        return;
    }
    auto projection = restriction_->getProjection();
    auto bundle = projection->getBundle();

    if (projection->getBaseDimension() > 0)
    {
        auto base = projection->getBase();
        base->freeState(xBaseTmp_);
    }

    freeStates(bundle, section_states_);
    bundle->freeState(lastValid_.first);
}

void PathSection::resize(unsigned int k) {
  section_states_.resize(k);
  allocStates(restriction_->getProjection()->getBundle(), section_states_);
}

PathRestrictionPtr PathSection::getRestriction() const {
  return restriction_;
}

std::vector<ompl::base::State*> PathSection::getStates() const {
  return section_states_;
}

const ompl::base::State* PathSection::at(int k) const
{
    return section_states_.at(k);
}

const ompl::base::State* PathSection::back() const
{
    return section_states_.back();
}

const ompl::base::State* PathSection::front() const
{
    return section_states_.front();
}

ompl::base::State* PathSection::atNonConst(int k) const
{
    return section_states_.at(k);
}

ompl::base::State* PathSection::backNonConst() const
{
    return section_states_.back();
}

ompl::base::State* PathSection::frontNonConst() const
{
    return section_states_.front();
}

void PathSection::addBaseStateIndex(const int index) {
    sectionBaseStateIndices_.push_back(index);
}

void PathSection::addEdgeToSection(ompl::base::State* /*xLast*/, ompl::base::State* xNext)
{
    section_states_.push_back(xNext);
}

void PathSection::sanityCheck()
{
    ProjectionPtr projection = restriction_->getProjection();
    auto bundle = projection->getBundle();
    bool feasible = true;
    for (unsigned int k = 1; k < section_states_.size(); k++)
    {
        base::State *sk1 = section_states_.at(k - 1);
        base::State *sk2 = section_states_.at(k);
        if (!restriction_->getSpaceInformation()->checkMotion(sk1, sk2))
        {
            feasible = false;
            OMPL_ERROR("Error between states %d and %d.", k - 1, k);
            bundle->printState(sk1);
            bundle->printState(sk2);
        }
    }

    if (!feasible)
    {
        throw Exception("Reported feasible path section, \
        but path section is infeasible.");
    }
}

unsigned int PathSection::size() const
{
    return section_states_.size();
}

std::pair<bool, HeadPtr> PathSection::checkMotion(const TreePtr& tree, const HeadPtr &head)
{
    ProjectionPtr projection = restriction_->getProjection();

    auto bundle = projection->getBundle();
    auto base = projection->getBase();

    auto parentNode = head->getTreeNode();

    HeadPtr newHead(head);
    for (unsigned int k = 1; k < section_states_.size(); k++)
    {
        if (restriction_->getSpaceInformation()->checkMotion(head->getState(), section_states_.at(k), lastValid_))
        {
            auto xNext = tree->addNodeAndParent(section_states_.at(k), newHead->getTreeNode());
            double locationOnBasePath = restriction_->getLengthBasePathUntil(sectionBaseStateIndices_.at(k));
            newHead->setCurrent(xNext, locationOnBasePath);

            if (k < section_states_.size() - 1) {
              parentNode = xNext;
              continue;
            }
            return std::make_pair(true, newHead);
        }

        lastValidIndexOnBasePath_ = sectionBaseStateIndices_.at(k - 1);

        base::State *lastValidBaseState = restriction_->getBasePath().at(lastValidIndexOnBasePath_);

        projection->project(lastValid_.first, xBaseTmp_);

        double distBaseSegment = base->distance(lastValidBaseState, xBaseTmp_);

        double locationOnBasePath =
            restriction_->getLengthBasePathUntil(lastValidIndexOnBasePath_) + distBaseSegment;

        //############################################################################
        // Get Last valid
        //############################################################################

        if (lastValid_.second > 0)
        {
            auto xLast = tree->addNodeAndParent(lastValid_.first, newHead->getTreeNode());
            newHead->setCurrent(xLast, locationOnBasePath);
            //head->setCurrent(xLast, locationOnBasePath);
        } else {
            newHead->setCurrent(parentNode, locationOnBasePath);
        }
        break;
    }
    return std::make_pair(false, newHead);
}


void PathSection::print(std::ostream &out) const
{
    ProjectionPtr projection = restriction_->getProjection();
    auto bundle = projection->getBundle();
    auto base = projection->getBase();

    out << std::string(80, '-') << std::endl;
    out << "Path Section" << std::endl;
    out << std::string(80, '-') << std::endl;

    out << section_states_.size() << " states over " << restriction_->size() << " base states." << std::endl;

    int maxDisplay = 5;  // display first and last N elements
    for (int k = 0; k < (int)section_states_.size(); k++)
    {
        if (k > maxDisplay && k < std::max(0, (int)section_states_.size() - maxDisplay))
            continue;
        int idx = sectionBaseStateIndices_.at(k);
        out << "State " << k << ": ";
        bundle->printState(section_states_.at(k));
        out << "Over Base state (idx " << idx << ") ";
        base->printState(restriction_->getBasePath().at(idx));
        out << std::endl;
    }

    out << std::string(80, '-') << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const PathSection &s)
        {
            s.print(out);
            return out;
        }
    }
}
