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

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_FINDSECTION_
#define OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_FINDSECTION_

#include <ompl/multilevel/datastructures/ParameterExponentialDecay.h>
#include <ompl/multilevel/datastructures/ParameterSmoothStep.h>
#include <ompl/util/ClassForward.h>
#include <ompl/base/State.h>
#include <optional>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Path */
        OMPL_CLASS_FORWARD(Path);
        /// @endcond
    }
    namespace geometric
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::PathGeometric */
        OMPL_CLASS_FORWARD(PathGeometric);
        /// @endcond
    }
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::multilevel::PathRestriction */
        OMPL_CLASS_FORWARD(PathRestriction);
        /** \brief Forward declaration of ompl::multilevel::FiberedProjection */
        OMPL_CLASS_FORWARD(FiberedProjection);
        /** \brief Forward declaration of ompl::multilevel::Head */
        OMPL_CLASS_FORWARD(Head);
        /** \brief Forward declaration of ompl::multilevel::PathSection */
        OMPL_CLASS_FORWARD(PathSection);
        /// @endcond

        class FindSection
        {
        public:
            FindSection() = delete;
            FindSection(const PathRestrictionPtr& restriction);

            virtual ~FindSection();

            virtual std::optional<PathSectionPtr> solve(const base::State* xBase, const base::State* xBundle) = 0;

            /** \brief Sample state on fiber while keeping base state fixed */
            bool findFeasibleStateOnFiber(const base::State* xBase, base::State* xBundle);

            /** \brief Triple step pattern */
            //bool tripleStep(HeadPtr &head, const base::State *sBundleGoal, double locationOnBasePathGoal);

        protected:
            /** \brief Pointer to associated bundle space */
            PathRestrictionPtr restriction_;

            base::State *xBaseTmp_{nullptr};
            base::State *xBundleTmp_{nullptr};

            base::State *xFiberStart_{nullptr};
            base::State *xFiberGoal_{nullptr};
            base::State *xFiberTmp_{nullptr};

        protected:
            /** \brief Radius of restriction neighborhood */
            ParameterExponentialDecay neighborhoodRadiusBaseSpace_;

            double neighborhoodRadiusBaseSpaceLambda_{1.0};

            double neighborhoodRadiusBaseSpaceTarget_{0.5};

            /** \brief Step size to check validity */
            double validBaseSpaceSegmentLength_;

            double validBundleSpaceSegmentLength_;

            double validFiberSpaceSegmentLength_;
        };
    }
}

#endif
