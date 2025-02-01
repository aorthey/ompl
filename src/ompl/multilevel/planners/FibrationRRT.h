#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOR_FIBRATIONRRT_
#define OMPL_MULTILEVEL_PLANNERS_FACTOR_FIBRATIONRRT_

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/util/RandomNumbers.h>
#include <unordered_map>
#include <ompl/multilevel/planners/FactoredPlanner.h>

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
        const float kGlobalGoalTreshold = 0.1;

        const double kGoalBiasDefault = 0.05;

        class FibrationRRT : public base::Planner 
        {
          public:
            using base::Planner::solve;

            FibrationRRT(const base::SpaceInformationPtr &si, float goal_threshold = kGlobalGoalTreshold);
            FibrationRRT(const FactoredSpaceInformationPtr &factor, float goal_threshold = kGlobalGoalTreshold);

            ~FibrationRRT() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;
            void setup() override;
            void setSeed(size_t seed);

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;
            void getPlannerData(base::PlannerData &data) const override;
            size_t getNumberOfIterations() const;

            const std::unordered_map<std::string, base::ProblemDefinitionPtr>& getProblemDefinitions() const;
            const std::unordered_map<std::string, base::PlannerStatus>& getPlannerStatus() const;

            base::ProblemDefinitionPtr getProblemDefinition(const std::string& name) const;
            FactoredPlannerPtr getPlanner(const std::string& name) const;

            std::string getIterationsProperty() const;
            std::string getBestCostProperty() const;

            size_t numFactors() const;

            //Global and Local Parameters 
            double getRange() const;
            void setRange(double range);
            void setLocalRange(const std::string& name, double range);

            double getGoalBias() const;
            void setGoalBias(double goal_bias);
            void setLocalGoalBias(const std::string& name, double goal_bias);

            double getPathRestrictionSamplingBias() const;
            void setPathRestrictionSamplingBias(double value);
            void setLocalPathRestrictionSamplingBias(const std::string& name, double path_restriction_sampling_bias);

            double getPathRestrictionSurroundingSamplingBias() const;
            void setPathRestrictionSurroundingSamplingBias(double value);
            void setLocalPathRestrictionSurroundingSamplingBias(const std::string& name, double path_restriction_surrounding_sampling_bias);

            double getSamplingPerturbationBias() const;
            void setSamplingPerturbationBias(double value);
            void setLocalSamplingPerturbationBias(const std::string& name, double sampling_perturbation_bias);

            void setSmoothIntermediateSolutions(bool smooth_intermediate_solutions = true);
            void setLocalSmoothIntermediateSolutions(const std::string& name, bool smooth_intermediate_solutions = true);

            void setSelectorFunctionType(const SelectorFunctionType& selector_function_type);
            void setDisableSectionSearch();
            void setEnableSectionSearch();


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
            bool hasNonSolvedSiblings_(const FactoredSpaceInformationPtr& factor) const;
            bool hasValidProblemDefinition_(const FactoredSpaceInformationPtr& factor) const;

            void createProblemDefinition_(const FactoredSpaceInformationPtr& factor, const base::State* parent_start, const base::GoalPtr& parent_goal);

            std::vector<FactoredPlannerPtr> getChildrenPlanner_(const FactoredSpaceInformationPtr& factor) const;
            std::optional<ompl::base::PlannerStatus> checkForInvalidPlannerStatus_() const;

          private:
            RNG rng_;

            std::optional<size_t> seed_;

            std::vector<FactoredSpaceInformationPtr> active_factors_;

            std::unordered_map<std::string, FactoredPlannerPtr> active_planners_;
            std::unordered_map<std::string, bool> is_active_;
            std::unordered_map<std::string, bool> is_solved_;
            std::unordered_map<std::string, base::ProblemDefinitionPtr> problem_definitions_per_factor_;
            std::unordered_map<std::string, base::PlannerStatus> planner_status_per_factor_;

            bool use_section_search_{true};

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
