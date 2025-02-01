/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, TU Berlin
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
 *   * Neither the name of the TU Berlin nor the names
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

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PROJECTIONS_COMPONENT_TIMEBASEDPROJECTION__
#define OMPL_MULTILEVEL_DATASTRUCTURES_PROJECTIONS_COMPONENT_TIMEBASEDPROJECTION__
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

namespace ompl
{
    namespace multilevel
    {
        struct InternalTimeBasedProjection {
          InternalTimeBasedProjection(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace) :
            bundleSpace(bundleSpace), baseSpace(baseSpace) {}
          virtual ~InternalTimeBasedProjection(){};

          virtual void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const = 0;

          virtual void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                            ompl::base::State *xBundle) const = 0;
          virtual void verify() = 0;
          unsigned int time_component_index_{0};

          base::StateSpacePtr bundleSpace;
          base::StateSpacePtr baseSpace;
        };

        struct InternalTimeBasedCompoundProjection : public InternalTimeBasedProjection{
          using InternalTimeBasedProjection::InternalTimeBasedProjection;
          void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;
          void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                            ompl::base::State *xBundle) const override;
          void verify() override;

          unsigned int GetBaseIndexFromBundleIndex(unsigned int bundle_index) const;
        };
        struct InternalTimeBasedNonCompoundProjection : public InternalTimeBasedProjection{
          using InternalTimeBasedProjection::InternalTimeBasedProjection;
          void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;
          void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                            ompl::base::State *xBundle) const override;
          void verify() override;
        };


        class XTimeToXProjection : public FiberedProjection
        {
            using BaseT = FiberedProjection;

        public:
            XTimeToXProjection(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace);

            ~XTimeToXProjection() override = default;

            virtual void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const override;

            virtual void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

            virtual void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                              ompl::base::State *xBundle) const override;
        protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;

            std::shared_ptr<InternalTimeBasedProjection> internal_projection_;

        };
    }
}

#endif
