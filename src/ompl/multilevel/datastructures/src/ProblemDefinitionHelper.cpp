#include <ompl/multilevel/datastructures/ProblemDefinitionHelper.h>

#include <ompl/base/Goal.h>
#include <ompl/base/goals/FactoredGoal.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/multilevel/datastructures/Projection.h>

namespace ompl
{
namespace multilevel
{

using namespace ompl::base;

std::unordered_map<std::string, base::ProblemDefinitionPtr>
createProblemDefinitionRecursive_(
    const FactoredSpaceInformationPtr& factor,
    const base::State* parent_start,
    const base::GoalPtr& parent_goal,
    double goal_threshold)
{
    std::unordered_map<std::string, base::ProblemDefinitionPtr> result;

    if (!factor) return result;

    base::ProblemDefinitionPtr pdef = std::make_shared<base::ProblemDefinition>(factor);

    const auto& projection = factor->getProjection();

    base::State* start = factor->allocState();
    projection->project(parent_start, start);
    pdef->addStartState(start);

    std::stringstream ss_start;
    factor->printState(start, ss_start);

    // === Handle goal ===
    const auto& type = parent_goal->getType();

    if (type == base::GoalType::GOAL_STATE) {
        const base::State* parent_goal_state = parent_goal->as<base::GoalState>()->getState();
        base::State* goal_state = factor->allocState();
        projection->project(parent_goal_state, goal_state);

        std::stringstream ss_goal;
        factor->printState(goal_state, ss_goal);

        OMPL_DEVMSG2("Project states onto factor %s \n Start %s Goal %s",
                     factor->getName().c_str(), ss_start.str().c_str(), ss_goal.str().c_str());

        auto goal = std::make_shared<base::GoalState>(factor);
        goal->setState(goal_state);
        goal->setThreshold(goal_threshold);   // assuming this member exists
        pdef->setGoal(goal);

    } else if (type == base::GoalType::FACTORED_GOAL) {
        const auto& factored_goal = parent_goal->as<base::FactoredGoal>();
        const auto maybe_factor_goal = factored_goal->getFactorGoal(factor->getName());

        if (!maybe_factor_goal.has_value()) {
            OMPL_ERROR("Could not find factor goal for factor %s", factor->getName().c_str());
            // Decide: throw or continue with empty goal?
            throw std::runtime_error("InvalidFactorGoal");
        }

        OMPL_DEVMSG2("Project states onto factor %s \n Start %s and Factored Goal",
                     factor->getName().c_str(), ss_start.str().c_str());

        pdef->setGoal(maybe_factor_goal.value());

    } else {
        OMPL_ERROR("FibrationRRT can only handle a single goal state or a factored goal region.");
        throw std::runtime_error("InvalidGoal");
    }

    result[factor->getName()] = pdef;

    // Recurse into children
    for (const auto& child : factor->getChildren()) {
        auto child_map = createProblemDefinitionRecursive_(child, start, pdef->getGoal(), goal_threshold);
        result.merge(std::move(child_map));
    }

    return result;
}

std::unordered_map<std::string, base::ProblemDefinitionPtr>
computeProblemDefinitions(
    const FactoredSpaceInformationPtr& root_factor,
    const base::ProblemDefinitionPtr& root_pdef,
    double goal_threshold)
{
    std::unordered_map<std::string, base::ProblemDefinitionPtr> result;

    if (!root_factor) {
        return result;
    }

    // Base case: insert the root's own problem definition
    result[root_factor->getName()] = root_pdef;

    // If no children, we're done
    if (!root_factor->hasChildren()) {
        return result;
    }

    // Safety checks (you can make these optional or pass as parameters)
    if (root_pdef->getStartStateCount() != 1) {
        OMPL_ERROR("FibrationRRT can handle only a single start state, but you have %d states.",
                   root_pdef->getStartStateCount());
        return result;  // or throw, depending on your preference
    }

    const base::State* root_start = root_pdef->getStartState(0);
    const base::GoalPtr& root_goal = root_pdef->getGoal();

    // Recursively create problem definitions for all children
    for (const auto& child : root_factor->getChildren()) {
        auto child_map = createProblemDefinitionRecursive_(child, root_start, root_goal, goal_threshold);
        result.merge(std::move(child_map));
    }
    return result;
}
}
}
