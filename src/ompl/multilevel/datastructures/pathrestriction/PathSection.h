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

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PATH_SECTION__
#define OMPL_MULTILEVEL_DATASTRUCTURES_PATHRESTRICTION_PATH_SECTION__

#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionTypes.h>
#include <ompl/util/ClassForward.h>

namespace ompl
{
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::multilevel::PathRestriction */
        OMPL_CLASS_FORWARD(PathRestriction);
        /** \brief Forward declaration of ompl::multilevel::Head */
        OMPL_CLASS_FORWARD(Head);
        /// @endcond

        /** \brief Representation of a path section (not necessarily feasible).
         *
         *  This class provides convenience methods to interpolate different
         *  path section over a given path restriction */

        class PathSection
        {
        public:
            PathSection() = delete;
            PathSection(const PathRestrictionPtr&);
            virtual ~PathSection();

            std::vector<base::State*> getStates() const;

            /** \brief Checks if section is feasible
             *
             *  @retval True if feasible and false if only partially feasible
             *  @retval Basepathheadptr Return last valid
             */
            bool checkMotion(HeadPtr &);

            /** \brief checks if section is feasible */
            void sanityCheck();
            void sanityCheck(HeadPtr &);

            /** \brief Methods to access sections like std::vector */
            const base::State* at(int k) const;
            const base::State* back() const;
            const base::State* front() const;

            base::State* atNonConst(int k) const;
            base::State* backNonConst() const;
            base::State* frontNonConst() const;

            unsigned int size() const;
            void resize(unsigned int);

            /** \brief Add vertex for sNext and edge to xLast by assuming motion
             * is valid  */
            //Configuration *addFeasibleSegment(Configuration *xLast, base::State *sNext);
            void addEdgeToSection(base::State* xLast, base::State* xNext);

            void addBaseStateIndex(const int);

            friend std::ostream &operator<<(std::ostream &, const PathSection &);

            void print(std::ostream &ostream = std::cout) const;

        protected:
            PathRestrictionPtr restriction_;

            /** \brief Interpolated section along restriction */
            std::vector<base::State *> section_states_;

            std::vector<std::pair<base::State*, base::State*>> edges_on_section_;

            std::vector<int> sectionBaseStateIndices_;

            /** \brief Last valid state on feasible segment */
            std::pair<base::State *, double> lastValid_;

            int lastValidIndexOnBasePath_;

            base::State *xBaseTmp_{nullptr};
            base::State *xBundleTmp_{nullptr};

            base::State *xFiberStart_{nullptr};
            base::State *xFiberGoal_{nullptr};
            base::State *xFiberTmp_{nullptr};
        };
    }
}
#endif
