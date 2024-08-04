#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOR_RESTRICTIONSAMPLER_
#define OMPL_MULTILEVEL_PLANNERS_FACTOR_RESTRICTIONSAMPLER_

#include "ompl/util/ClassForward.h"
#include "ompl/base/StateSampler.h"

namespace ompl {
    namespace multilevel {

        OMPL_CLASS_FORWARD(RestrictionSampler);
        OMPL_CLASS_FORWARD(FactoredSpaceInformation);
        OMPL_CLASS_FORWARD(FactoredPlanner);

        class RestrictionSampler : public ompl::base::StateSampler {
          public:
            /** \brief Constructor */
            RestrictionSampler(const FactoredSpaceInformationPtr& factor, const std::vector<FactoredPlannerPtr>& children_planner);

            ~RestrictionSampler();

            void sampleUniform(base::State *state) override;
            void sampleUniformNear(base::State *state, const base::State *near, double distance) override;
            void sampleGaussian(base::State *state, const base::State *mean, double stdDev) override;

          private:
            base::StateSamplerPtr baseSampler_;
            FactoredSpaceInformationPtr factor_;

            std::unordered_map<std::string, base::State*> statesChild_;
            std::unordered_map<std::string, FactoredPlannerPtr> childrenPlanner_;

            std::vector<std::string> childNames_;
        };
    }
}

#endif // OMPL_MULTILEVEL_PLANNERS_FACTOR_RESTRICTIONSAMPLER_

