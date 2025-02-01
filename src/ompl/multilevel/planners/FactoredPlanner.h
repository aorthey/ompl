#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
#define OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_

#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include "ompl/multilevel/planners/RRTtask.h"
#include "ompl/multilevel/datastructures/helpers/Expected.h"

#include <optional>

namespace ompl {
    namespace multilevel {

        OMPL_CLASS_FORWARD(FactoredPlanner);
        OMPL_CLASS_FORWARD(FactoredSpaceInformation);
        OMPL_CLASS_FORWARD(PathSection);

        // const double kDefaultPathRestrictionSamplingBias = 0.2;
        // const double kDefaultPathRestrictionSurroundingBias = 0.1;
        // const double kDefaultSamplingPerturbationValue  = 0.05;
        const double kDefaultPathRestrictionSamplingBias = 0.5;
        const double kDefaultPathRestrictionSurroundingBias = 0.1;
        const double kDefaultSamplingPerturbationValue  = 0.05;

        typedef ompl::multilevel::RRTtask BaseTypePlanner;

        class FactoredPlanner : public BaseTypePlanner {
          public:
            /** \brief Constructor */
            FactoredPlanner(const FactoredSpaceInformationPtr& si, const std::vector<FactoredPlannerPtr>& children_planner = {});

            void clear() override;
            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            Expected<PathSectionPtr, std::string> solveSection();

            ompl::base::State* MakeStartState() const;
            ompl::base::State* MakeGoalState() const;

            void sampleFromDatastructure(ompl::base::State* state);
            void sampleFromPath(const std::vector<base::State *>& path_states, ompl::base::State* state);

            size_t getNumberOfSamples() const;

            void setPathRestrictionSamplingBias(double path_restriction_sampling_bias);
            double getPathRestrictionSamplingBias() const;

            void setPathRestrictionSurroundingSamplingBias(double path_restriction_surrounding_sampling_bias);
            double getPathRestrictionSurroundingSamplingBias() const;

            void setSamplingPerturbationBias(double sampling_perturbation_bias);
            double getSamplingPerturbationBias() const;

            void setDisableSectionSearch();
            void setEnableSectionSearch();

            void setSeed(size_t seed);
          private:
            double path_restriction_sampling_bias_{kDefaultPathRestrictionSamplingBias};
            double path_restriction_surrounding_sampling_bias_{kDefaultPathRestrictionSurroundingBias};
            double sampling_perturbation_bias_{kDefaultSamplingPerturbationValue};

            ompl::base::StateSamplerPtr internal_space_sampler_;

            std::vector<FactoredPlannerPtr> children_planner_;

            bool use_section_search_{true};
        };
    }
}

#endif // OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
