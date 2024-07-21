#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOR_FIBRATIONRRT_
#define OMPL_MULTILEVEL_PLANNERS_FACTOR_FIBRATIONRRT_

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/util/RandomNumbers.h>
#include <unordered_map>
#include <ompl/multilevel/planners/factor/FactoredPlanner.h>

namespace ompl
{
    namespace multilevel
    {
        enum class SelectorFunctionType {
          kUniform = 0,
          kExponential = 1,
          kLastLevel = 2
        };

        OMPL_CLASS_FORWARD(FibrationRRT);
        OMPL_CLASS_FORWARD(FactoredSpaceInformation);
        OMPL_CLASS_FORWARD(FactoredPlanner);

        const size_t kNumberOfIterationsPerPlannerCall = 1;
        //const size_t kNumberOfIterationsPerPlannerCall = 3;
        const float kGlobalGoalTreshold = 0.1;

        class FibrationRRT : public base::Planner 
        {

          public:

            using base::Planner::solve;

            FibrationRRT(const FactoredSpaceInformationPtr &si, float goal_threshold = kGlobalGoalTreshold);

            ~FibrationRRT() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;
            void setup() override;
            void setSeed(size_t seed);

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;
            void getPlannerData(base::PlannerData &data) const override;

            void setSelectorFunctionType(const SelectorFunctionType& selector_function_type);

            const std::unordered_map<std::string, base::ProblemDefinitionPtr>& getProblemDefinitions() const;
            const std::unordered_map<std::string, base::PlannerStatus>& getPlannerStatus() const;

            base::ProblemDefinitionPtr getProblemDefinition(const std::string& name) const;

            //const FactoredSpaceInformationPtr& getFactoredSpaceInformation() const;

            std::string getIterationsProperty() const;
            std::string getBestCostProperty() const;

            size_t numFactors() const;

            //Parameters for individual planners
            void setRange(double range);
            void setGoalBias(double goal_bias);
            void setSmoothIntermediateSolutions(bool smooth_intermediate_solutions = true);

            void setSmoothIntermediateSolutions(const std::string& name, bool smooth_intermediate_solutions = true);
            void setRange(const std::string& name, double range);
            void setGoalBias(const std::string& name, double goal_bias);
            void setPathRestrictionSamplingBias(const std::string& name, double path_restriction_sampling_bias);
            void setPathRestrictionSurroundingSamplingBias(const std::string& name, double path_restriction_surrounding_sampling_bias);
            void setSamplingPerturbationBias(const std::string& name, double sampling_perturbation_bias);


          protected:
            bool shouldSmoothSolutionPath(const FactoredSpaceInformationPtr& factor);
            void smoothSolutionPath(const FactoredSpaceInformationPtr& factor);

            void grow_(const FactoredSpaceInformationPtr& factor);
            bool hasSolution_(const FactoredSpaceInformationPtr& factor) const;

            const FactoredSpaceInformationPtr& selectFactor_();
            const FactoredSpaceInformationPtr& selectFactorExponential_();
            const FactoredSpaceInformationPtr& selectFactorUniform_();
            const FactoredSpaceInformationPtr& selectFactorLastLevel_();

            void createPlannerForFactor_(const FactoredSpaceInformationPtr& factor);

            bool isActive_(const FactoredSpaceInformationPtr& factor) const;
            bool isSolved_(const FactoredSpaceInformationPtr& factor) const;
            bool allChildrenHaveSolutions_(const FactoredSpaceInformationPtr& factor) const;
            bool hasValidProblemDefinition_(const FactoredSpaceInformationPtr& factor) const;

            void createProblemDefinition_(const FactoredSpaceInformationPtr& factor, const base::State* parent_start, const base::GoalPtr& parent_goal);

            std::vector<FactoredPlannerPtr> getChildrenPlanner_(const FactoredSpaceInformationPtr& factor) const;

          private:
            RNG rng_;

            std::optional<size_t> seed_;

            std::vector<FactoredSpaceInformationPtr> active_factors_;
            std::unordered_map<std::string, FactoredPlannerPtr> active_planners_;
            std::unordered_map<std::string, bool> is_active_;
            std::unordered_map<std::string, bool> is_solved_;
            std::unordered_map<std::string, base::ProblemDefinitionPtr> problem_definitions_per_factor_;
            std::unordered_map<std::string, base::PlannerStatus> planner_status_per_factor_;

            //Parameters per planner
            std::unordered_map<std::string, double> range_;
            std::unordered_map<std::string, double> goal_bias_;
            std::unordered_map<std::string, double> path_restriction_sampling_bias_;
            std::unordered_map<std::string, double> path_restriction_surrounding_sampling_bias_;
            std::unordered_map<std::string, double> sampling_perturbation_bias_;
            std::unordered_map<std::string, bool> smooth_intermediate_solutions_;

            base::PlannerStatus planner_status_;

            unsigned int iterations_{0};
            float bestCost_{std::numeric_limits<float>::infinity()};
            std::optional<double> global_range_;
            //bool smoothing_enabled_{false};
            float goal_threshold_{kGlobalGoalTreshold};

            SelectorFunctionType selector_function_type_{SelectorFunctionType::kExponential};
        };

    }
}

#endif
