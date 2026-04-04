/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021,
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

#include <ompl/multilevel/datastructures/ProjectionFactory.h>

#include <ompl/multilevel/datastructures/projections/NoneProjection.h>
#include <ompl/multilevel/datastructures/projections/EmptySetProjection.h>
#include <ompl/multilevel/datastructures/projections/IdentityProjection.h>
#include <ompl/multilevel/datastructures/projections/RelaxationProjection.h>

#include <ompl/multilevel/datastructures/projections/FiberedSubspaceProjection.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

#include <ompl/multilevel/datastructures/projections/SE2RNToR2Projection.h>
#include <ompl/multilevel/datastructures/projections/XSE2RNToXR2Projection.h>
#include <ompl/multilevel/datastructures/projections/SE3RNToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/SE3ToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/SE2ToR2Projection.h>
#include <ompl/multilevel/datastructures/projections/RNToRMProjection.h>

#include <ompl/multilevel/datastructures/projections/R3R2SO2ToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/R3SO2ToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/XR3R2SO2ToXR3Projection.h>
#include <ompl/multilevel/datastructures/projections/XR3SO2ToXR3Projection.h>

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

#include <ompl/util/Exception.h>

using namespace ompl::multilevel;
using namespace ompl::base;

ProjectionPtr ProjectionFactory::makeProjection(const SpaceInformationPtr &Bundle)
{
    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
    int nrProjections = GetNumberOfComponents(Bundle_space);

    if (nrProjections > 1)
    {
        std::vector<ProjectionPtr> components;
        const base::CompoundStateSpace *Bundle_compound = Bundle_space->as<base::CompoundStateSpace>();
        const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

        for (int m = 0; m < nrProjections; m++)
        {
            const base::StateSpacePtr BundleM = Bundle_decomposed.at(m);
            ProjectionPtr componentM = makeProjection(BundleM);
            components.push_back(componentM);
        }
        return std::make_shared<CompoundProjection>(Bundle_space, nullptr, components);
    }
    else
    {
        ProjectionPtr component = makeProjection(Bundle_space);
        return component;
    }
}

ProjectionPtr ProjectionFactory::makeProjection(const SpaceInformationPtr &Bundle, const SpaceInformationPtr &Base)
{
    if (Base == nullptr)
    {
        return makeProjection(Bundle);
    }

    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
    int bundleSpaceComponents = GetNumberOfComponents(Bundle_space);
    const base::StateSpacePtr Base_space = Base->getStateSpace();
    int baseSpaceComponents = GetNumberOfComponents(Base_space);

    OMPL_DEBUG("Making Projection from Bundle Space %s to Base Space %s.",
        Bundle->getStateSpace()->getName().c_str(), 
        Base->getStateSpace()->getName().c_str());

    // if (baseSpaceComponents != bundleSpaceComponents)
    // {
    //     Base->printSettings();
    //     OMPL_ERROR("Bundle Space %s has %d, but Base Space %s has %d components.", 
    //         Bundle->getStateSpace()->getName().c_str(), 
    //         bundleSpaceComponents, Base->getStateSpace()->getName().c_str(), baseSpaceComponents);
    //     throw Exception("Different Number Of Components");
    // }

    // Check if planning spaces are equivalent, i.e. if (X, \phi) == (Y, \phi)
    bool areValidityCheckersEquivalent = false;
    if (*(Base->getStateValidityChecker().get()) == *(Bundle->getStateValidityChecker().get()))
    {
        areValidityCheckersEquivalent = true;
    }

    // if (bundleSpaceComponents > 1 && baseSpaceComponents > 1)
    // {
    //     std::vector<ProjectionPtr> components;

    //     base::CompoundStateSpace *Bundle_compound = Bundle_space->as<base::CompoundStateSpace>();
    //     base::CompoundStateSpace *Base_compound = Base_space->as<base::CompoundStateSpace>();

    //     const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    //     const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    //     for (int m = 0; m < bundleSpaceComponents; m++)
    //     {
    //         base::StateSpacePtr BaseM = Base_decomposed.at(m);
    //         base::StateSpacePtr BundleM = Bundle_decomposed.at(m);
    //         ProjectionPtr componentM = makeProjection(BundleM, BaseM, areValidityCheckersEquivalent);
    //         components.push_back(componentM);
    //     }

    //     return std::make_shared<CompoundProjection>(Bundle_space, Base_space, components);
    // }
    ProjectionPtr component = makeProjection(Bundle_space, Base_space, areValidityCheckersEquivalent);
    OMPL_DEBUG(">>> Created projection %s", component->getTypeAsString().c_str());
    return component;
}

ProjectionPtr ProjectionFactory::makeProjection(const StateSpacePtr &Bundle)
{
    return makeProjection(Bundle, nullptr, false);
}

ProjectionPtr ProjectionFactory::makeProjection(const StateSpacePtr &Bundle, const StateSpacePtr &Base,
                                                bool areValidityCheckersEquivalent)
{
    ProjectionType type = identifyProjectionType(Bundle, Base);
    if (type == PROJECTION_IDENTITY && !areValidityCheckersEquivalent)
    {
        type = PROJECTION_CONSTRAINED_RELAXATION;
    }

    ProjectionPtr component;

    if (type == PROJECTION_NONE)
    {
        component = std::make_shared<NoneProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_EMPTY_SET)
    {
        component = std::make_shared<EmptySetProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_SUBSPACE)
    {
        component = std::make_shared<FiberedSubspaceProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_IDENTITY)
    {
        component = std::make_shared<IdentityProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_CONSTRAINED_RELAXATION)
    {
        component = std::make_shared<RelaxationProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_RN_RM)
    {
        component = std::make_shared<RNToRMProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_RNSO2_RN)
    {
        component = std::make_shared<RNSO2ToRNProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_SE2_R2)
    {
        component = std::make_shared<SE2ToR2Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_SE2RN_R2)
    {
        component = std::make_shared<SE2RNToR2Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_XSE2RN_XR2)
    {
        component = std::make_shared<XSE2RNToXR2Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_SE2RN_SE2)
    {
        component = std::make_shared<SE2RNToSE2Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_SE2RN_SE2RM)
    {
        component = std::make_shared<SE2RNToSE2RMProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_SO2RN_SO2)
    {
        component = std::make_shared<SO2RNToSO2Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_SO2RN_SO2RM)
    {
        component = std::make_shared<SO2RNToSO2RMProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_SO3RN_SO3)
    {
        component = std::make_shared<SO3RNToSO3Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_SO3RN_SO3RM)
    {
        component = std::make_shared<SO3RNToSO3RMProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_SE3_R3)
    {
        component = std::make_shared<SE3ToR3Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_SE3RN_R3)
    {
        component = std::make_shared<SE3RNToR3Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_SE3RN_SE3)
    {
        component = std::make_shared<SE3RNToSE3Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_SE3RN_SE3RM)
    {
        component = std::make_shared<SE3RNToSE3RMProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_SO2N_SO2M)
    {
        component = std::make_shared<SO2NToSO2MProjection>(Bundle, Base);
    }
    else if (type == PROJECTION_R3R2SO2_R3)
    {
        component = std::make_shared<R3R2SO2ToR3Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_R3SO2_R3)
    {
        component = std::make_shared<R3SO2ToR3Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_XR3SO2_XR3)
    {
        component = std::make_shared<XR3SO2ToXR3Projection>(Bundle, Base);
    }
    else if (type == PROJECTION_XR3R2SO2_XR3)
    {
        component = std::make_shared<XR3R2SO2ToXR3Projection>(Bundle, Base);
    }
    else
    {
        OMPL_ERROR("NYI: %d", type);
        throw Exception("BundleSpaceType not yet implemented.");
    }
    auto componentFiber = std::dynamic_pointer_cast<FiberedProjection>(component);
    if (componentFiber != nullptr)
    {
        componentFiber->makeFiberSpace();
    }
    return component;
}

ProjectionType ProjectionFactory::identifyProjectionType(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (Base == nullptr)
    {
        return PROJECTION_NONE;
    }

    if (isMapping_Identity(Bundle, Base))
    {
        return PROJECTION_IDENTITY;
    }

    if (isMapping_EmptyProjection(Bundle, Base))
    {
        return PROJECTION_EMPTY_SET;
    }

    if (isMapping_ToFiberedSubspace(Bundle, Base))
    {
        return PROJECTION_SUBSPACE;
    }

    // RN ->
    if (isMapping_RN_to_RM(Bundle, Base))
    {
        return PROJECTION_RN_RM;
    }
    if (isMapping_RNSO2_to_RN(Bundle, Base))
    {
        return PROJECTION_RNSO2_RN;
    }

    // SE3 ->
    if (isMapping_SE3_to_R3(Bundle, Base))
    {
        return PROJECTION_SE3_R3;
    }
    if (isMapping_SE3RN_to_SE3(Bundle, Base))
    {
        return PROJECTION_SE3RN_SE3;
    }
    if (isMapping_SE3RN_to_R3(Bundle, Base))
    {
        return PROJECTION_SE3RN_R3;
    }
    if (isMapping_SE3RN_to_SE3RM(Bundle, Base))
    {
        return PROJECTION_SE3RN_SE3RM;
    }

    // SE2 ->
    if (isMapping_SE2_to_R2(Bundle, Base))
    {
        return PROJECTION_SE2_R2;
    }
    if (isMapping_SE2RN_to_SE2(Bundle, Base))
    {
        return PROJECTION_SE2RN_SE2;
    }
    if (isMapping_SE2RN_to_R2(Bundle, Base))
    {
        return PROJECTION_SE2RN_R2;
    }
    if (isMapping_XSE2RN_to_XR2(Bundle, Base))
    {
        return PROJECTION_XSE2RN_XR2;
    }
    if (isMapping_SE2RN_to_SE2RM(Bundle, Base))
    {
        return PROJECTION_SE2RN_SE2RM;
    }

    // SO2 ->
    if (isMapping_SO2RN_to_SO2(Bundle, Base))
    {
        return PROJECTION_SO2RN_SO2;
    }
    if (isMapping_SO2RN_to_SO2RM(Bundle, Base))
    {
        return PROJECTION_SO2RN_SO2RM;
    }
    if (isMapping_SO2N_to_SO2M(Bundle, Base))
    {
        return PROJECTION_SO2N_SO2M;
    }

    // SO3 ->
    if (isMapping_SO3RN_to_SO3(Bundle, Base))
    {
        return PROJECTION_SO3RN_SO3;
    }
    if (isMapping_SO3RN_to_SO3RM(Bundle, Base))
    {
        return PROJECTION_SO3RN_SO3RM;
    }
    if (isMapping_ToFiberedSubspace(Bundle, Base))
    {
        return PROJECTION_SO3RN_SO3RM;
    }

    //XR3 ->
    if (isMapping_R3R2SO2_to_R3(Bundle, Base))
    {
        return PROJECTION_R3R2SO2_R3;
    }
    if (isMapping_R3SO2_to_R3(Bundle, Base))
    {
        return PROJECTION_R3SO2_R3;
    }
    if (isMapping_XR3SO2_to_XR3(Bundle, Base))
    {
        return PROJECTION_XR3SO2_XR3;
    }
    if (isMapping_XR3R2SO2_to_XR3(Bundle, Base))
    {
        return PROJECTION_XR3R2SO2_XR3;
    }

    OMPL_ERROR("Fiber Bundle unknown from %s to %s.", Bundle->getName().c_str(), Base->getName().c_str());
    return PROJECTION_UNKNOWN;
}

bool ProjectionFactory::isMapping_Identity(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (Base->getType() != Bundle->getType())
    {
        return false;
    }
    if (Base->getDimension() != Bundle->getDimension())
    {
        return false;
    }
    if (Bundle->isCompound())
    {
        if (Base->isCompound())
        {
            base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
            base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

            if (Bundle_decomposed.size() == Base_decomposed.size())
            {
                for (unsigned int k = 0; k < Bundle_decomposed.size(); k++)
                {
                    if (!isMapping_Identity(Bundle_decomposed.at(k), Base_decomposed.at(k)))
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }
    else
    {
        if ((Base->getType() == Bundle->getType()) && (Base->getDimension() == Bundle->getDimension()))
        {
            return true;
        }
    }
    return false;
}

bool ProjectionFactory::isMapping_ToFiberedSubspace(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    if( Bundle_decomposed.size() > 0) {
      if (Bundle_decomposed.at(0)->getType() == Base->getType()) {
        unsigned int n = Bundle_decomposed.at(0)->getDimension();
        unsigned int m = Base->getDimension();
        if(n == m) {
            return true;
        }
      }
    }
    return false;
}


bool ProjectionFactory::isMapping_RN_to_RM(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (Bundle->isCompound())
        return false;

    if (Bundle->getType() == base::STATE_SPACE_REAL_VECTOR)
    {
        unsigned int n = Bundle->getDimension();
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            unsigned int m = Base->getDimension();
            if (n > m && m > 0)
            {
                return true;
            }
        }
    }
    return false;
}

bool ProjectionFactory::isMapping_SE3_to_R3(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
        return false;

    if (Bundle->getType() == base::STATE_SPACE_SE3)
    {
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getDimension() == 3)
            {
                return true;
            }
        }
    }
    return false;
}
bool ProjectionFactory::isMapping_SE3RN_to_R3(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = Base->getDimension();
                if (m == 3)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ProjectionFactory::isMapping_SE2_to_R2(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
        return false;

    if (Bundle->getType() == base::STATE_SPACE_SE2)
    {
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getDimension() == 2)
            {
                return true;
            }
        }
    }
    return false;
}

bool ProjectionFactory::isMapping_RNSO2_to_RN(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_REAL_VECTOR &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_SO2)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int n = Bundle_decomposed.at(0)->getDimension();
                unsigned int m = Base->getDimension();
                if (m == n)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ProjectionFactory::isMapping_SE2RN_to_R2(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = Base->getDimension();
                if (m == 2)
                {
                    return true;
                }
            }
        }
    }
    return false;
}
bool ProjectionFactory::isMapping_XR3R2SO2_to_XR3(const base::StateSpacePtr &Bundle, const base::StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
    {
        return false;
    }
    if (!Base->isCompound())
    {
        return false;
    }

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    unsigned int n = Bundle_decomposed.size();
    unsigned int m = Base_decomposed.size();

    if( n != m) 
    {
        return false;
    }
    if( n < 2 ) 
    {
        return false;
    }

    const auto& last = Bundle_decomposed.at(n - 1);
    if (!last->isCompound()) 
    {
        return false;
    }
    base::CompoundStateSpace *last_compound = last->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> last_decomposed = last_compound->getSubspaces();
    if (last_decomposed.size() != 3) 
    {
        return false;
    }

    if (last_decomposed.at(0)->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (last_decomposed.at(0)->getDimension() != 3)
    {
        return false;
    }
    if (last_decomposed.at(1)->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (last_decomposed.at(1)->getDimension() != 2)
    {
        return false;
    }
    if (last_decomposed.at(2)->getType() != base::STATE_SPACE_SO2)
    {
        return false;
    }

    auto base_last = Base_decomposed.back();
    if (base_last->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (base_last->getDimension() != 3)
    {
        return false;
    }

    return true;
}

bool ProjectionFactory::isMapping_XSE2RN_to_XR2(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
    {
        return false;
    }
    if (!Base->isCompound())
    {
        return false;
    }

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    unsigned int n = Bundle_decomposed.size();
    unsigned int m = Base_decomposed.size();

    if( n != m) 
    {
        return false;
    }
    if( n < 2 ) 
    {
        return false;
    }

    const auto& last = Bundle_decomposed.at(n - 1);
    if (!last->isCompound()) 
    {
        return false;
    }
    base::CompoundStateSpace *last_compound = last->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> last_decomposed = last_compound->getSubspaces();
    if (last_decomposed.size() != 2) 
    {
        return false;
    }

    if (last_decomposed.at(0)->getType() != base::STATE_SPACE_SE2)
    {
        return false;
    }
    if (last_decomposed.at(1)->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }

    auto base_last = Base_decomposed.back();
    if (base_last->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (base_last->getDimension() != 2)
    {
        return false;
    }

    return true;
}

bool ProjectionFactory::isMapping_SE2RN_to_SE2(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SE2);
}

bool ProjectionFactory::isMapping_SE3RN_to_SE3(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SE3);
}

bool ProjectionFactory::isMapping_SO2RN_to_SO2(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SO2);
}

bool ProjectionFactory::isMapping_SO3RN_to_SO3(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SO3);
}

bool ProjectionFactory::isMapping_SE2RN_to_SE2RM(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SE2);
}

bool ProjectionFactory::isMapping_SE3RN_to_SE3RM(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SE3);
}

bool ProjectionFactory::isMapping_SO2RN_to_SO2RM(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SO2);
}

bool ProjectionFactory::isMapping_SO3RN_to_SO3RM(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SO3);
}

bool ProjectionFactory::isMapping_SO2N_to_SO2M(const StateSpacePtr &Bundle, const StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();

    for (unsigned int k = 0; k < Bundle_subspaces; k++)
    {
        if (!(Bundle_decomposed.at(k)->getType() == base::STATE_SPACE_SO2))
        {
            return false;
        }
    }
    if (!Base->isCompound())
    {
        if (!(Base->getType() == base::STATE_SPACE_SO2))
        {
            return false;
        }
    }
    else
    {
        base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
        const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
        unsigned int Base_subspaces = Base_decomposed.size();

        for (unsigned int k = 0; k < Base_subspaces; k++)
        {
            if (!(Base_decomposed.at(k)->getType() == base::STATE_SPACE_SO2))
            {
                return false;
            }
        }
    }

    return true;
}

bool ProjectionFactory::isMapping_XRN_to_X(const StateSpacePtr &Bundle, const StateSpacePtr &Base,
                                           const StateSpaceType type)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == type &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == type)
            {
                return true;
            }
        }
    }
    return false;
}

bool ProjectionFactory::isMapping_XRN_to_XRM(const StateSpacePtr &Bundle, const StateSpacePtr &Base,
                                             const StateSpaceType type)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    if (Bundle_decomposed.size() == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == type &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (!Base->isCompound())
                return false;
            unsigned int n = Bundle_decomposed.at(1)->getDimension();

            base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
            if (Base_decomposed.size() == 2)
            {
                if (Base_decomposed.at(0)->getType() == type &&
                    Base_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                {
                    unsigned int m = Base_decomposed.at(1)->getDimension();
                    if (n > m && m > 0)
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool ProjectionFactory::isMapping_R3R2SO2_to_R3(const base::StateSpacePtr &Bundle, const base::StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
    {
        return false;
    }
    if (Base->isCompound())
    {
        return false;
    }

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    if (Bundle_decomposed.size() != 3)
    {
        return false;
    }
    if (Bundle_decomposed.at(0)->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (Bundle_decomposed.at(0)->getDimension() != 3)
    {
        return false;
    }
    if (Bundle_decomposed.at(1)->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (Bundle_decomposed.at(1)->getDimension() != 2)
    {
        return false;
    }
    if (Bundle_decomposed.at(2)->getType() != base::STATE_SPACE_SO2)
    {
        return false;
    }

    if (Base->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (Base->getDimension() != 3)
    {
        return false;
    }
    return true;
}

bool ProjectionFactory::isMapping_R3SO2_to_R3(const base::StateSpacePtr &Bundle, const base::StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
    {
        return false;
    }
    if (Base->isCompound())
    {
        return false;
    }

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    if (Bundle_decomposed.size() != 2)
    {
        return false;
    }
    if (Bundle_decomposed.at(0)->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (Bundle_decomposed.at(0)->getDimension() != 3)
    {
        return false;
    }
    if (Bundle_decomposed.at(2)->getType() != base::STATE_SPACE_SO2)
    {
        return false;
    }
    if (Base->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (Base->getDimension() != 3)
    {
        return false;
    }
    return true;
}

bool ProjectionFactory::isMapping_XR3SO2_to_XR3(const base::StateSpacePtr &Bundle, const base::StateSpacePtr &Base)
{
    if (!Bundle->isCompound())
    {
      std::cout << "Fail00" << std::endl;
        return false;
    }
    if (!Base->isCompound())
    {
      std::cout << "Fail00b" << std::endl;
        return false;
    }

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    auto n = Bundle_decomposed.size();
    auto m = Base_decomposed.size();

    if (n < 2) 
    {
        return false;
    }
    if(n != m) {
        return false;
    }

    auto last = Bundle_decomposed.at(n-1);
    base::CompoundStateSpace *last_compound = last->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> last_decomposed = last_compound->getSubspaces();
    if (last_decomposed.size() != 2)
    {
        return false;
    }
    if (last_decomposed.at(0)->getDimension() != 3)
    {
        return false;
    }
    if (last_decomposed.at(0)->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (last_decomposed.at(1)->getType() != base::STATE_SPACE_SO2)
    {
        return false;
    }

    auto base_last = Base_decomposed.back();
    if (base_last->getType() != base::STATE_SPACE_REAL_VECTOR)
    {
        return false;
    }
    if (base_last->getDimension() != 3)
    {
        return false;
    }

    return true;
}

bool ProjectionFactory::isMapping_EmptyProjection(const StateSpacePtr &, const StateSpacePtr &Base)
{
    if (Base == nullptr || Base->getDimension() <= 0)
    {
        return true;
    }
    return false;
}

int ProjectionFactory::GetNumberOfComponents(const StateSpacePtr &space)
{
    int nrComponents = 0;

    if (space->isCompound())
    {
        base::CompoundStateSpace *compound = space->as<base::CompoundStateSpace>();
        nrComponents = compound->getSubspaceCount();
        if (nrComponents == 2)
        {
            int type = space->getType();

            if ((type == base::STATE_SPACE_SE2) || (type == base::STATE_SPACE_SE3) ||
                (type == base::STATE_SPACE_DUBINS))
            {
                nrComponents = 1;
            }
            else
            {
                const std::vector<base::StateSpacePtr> decomposed = compound->getSubspaces();
                int t0 = decomposed.at(0)->getType();
                int t1 = decomposed.at(1)->getType();
                if ((t0 == base::STATE_SPACE_SO2 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
                    (t0 == base::STATE_SPACE_SO3 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
                    (t0 == base::STATE_SPACE_SE2 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
                    (t0 == base::STATE_SPACE_SE3 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
                    (t0 == base::STATE_SPACE_SO2 && t1 == base::STATE_SPACE_SO2))
                {
                    if (decomposed.at(1)->getDimension() > 0)
                    {
                        nrComponents = 1;
                    }
                }
            }
        }
    }
    else
    {
        nrComponents = 1;
    }
    return nrComponents;
}
