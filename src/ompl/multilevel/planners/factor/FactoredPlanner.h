#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
#define OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_

#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTtask.h"
#include "ompl/geometric/planners/rlrt/RLRT.h"

#include <optional>
#include <boost/outcome.hpp>

template <class T, class E>
using Expected = boost::outcome_v2::basic_result<T, E, boost::outcome_v2::policy::default_policy<T, E, void>>;
using boost::outcome_v2::failure;
using boost::outcome_v2::success;

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

        typedef ompl::geometric::RRTtask BaseTypePlanner;

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

            void setSeed(size_t seed);
          private:
            double path_restriction_sampling_bias_{kDefaultPathRestrictionSamplingBias};
            double path_restriction_surrounding_sampling_bias_{kDefaultPathRestrictionSurroundingBias};
            double sampling_perturbation_bias_{kDefaultSamplingPerturbationValue};

            ompl::base::StateSamplerPtr internal_space_sampler_;
            bool firstRun_{true};

            std::vector<FactoredPlannerPtr> children_planner_;
        };
    }
}

#endif // OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
