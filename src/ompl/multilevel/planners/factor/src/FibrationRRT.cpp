#include "ompl/multilevel/planners/factor/FibrationRRT.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/FactoredGoal.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/multilevel/planners/factor/FactoredPlanner.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

using namespace ompl::multilevel;

FibrationRRT::FibrationRRT(const FactoredSpaceInformationPtr &si, float goal_threshold) :
   ompl::base::Planner(si, "FibrationRRT"), goal_threshold_(goal_threshold)
{
  specs_.approximateSolutions = false;
  specs_.directed = true;
  specs_.optimizingPaths = true;
  addPlannerProgressProperty("iterations INTEGER", [this] { return getIterationsProperty(); });
  addPlannerProgressProperty("best cost REAL", [this] { return getBestCostProperty(); });

  Planner::declareParam<double>("range", this, &FibrationRRT::setRange, &FibrationRRT::getRange, "0.:1.:10000.");
  Planner::declareParam<bool>("smoothIntermediateSolutions", this, &FibrationRRT::setSmoothIntermediateSolutions, &FibrationRRT::getSmoothIntermediateSolutions, "0,1");
}

void FibrationRRT::setSmoothIntermediateSolutions(bool smoothing_enabled) {
  smoothing_enabled_ = smoothing_enabled;
}

bool FibrationRRT::getSmoothIntermediateSolutions() const {
  return smoothing_enabled_;
}

FibrationRRT::~FibrationRRT() {
  for(const auto& planner : active_planners_) {
    planner.second->clear();
  }
}

void FibrationRRT::getPlannerData(base::PlannerData &data) const {
  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  auto root_planner = active_planners_.find(root->getName());
  if(root_planner == active_planners_.end()) {
    return;
  }
  root_planner->second->getPlannerData(data);
}

const FactoredSpaceInformationPtr& FibrationRRT::getFactoredSpaceInformation() const {
  return std::static_pointer_cast<FactoredSpaceInformation>(si_);
}

const std::unordered_map<std::string, ompl::base::ProblemDefinitionPtr>& FibrationRRT::getProblemDefinitions() const {
  return problem_definitions_per_factor_;
}

const std::unordered_map<std::string, ompl::base::PlannerStatus>& FibrationRRT::getPlannerStatus() const {
  return planner_status_per_factor_;
}

ompl::base::ProblemDefinitionPtr FibrationRRT::getProblemDefinition(const std::string& name) const {
  auto pdef_iterator = problem_definitions_per_factor_.find(name);
  if(pdef_iterator == problem_definitions_per_factor_.end()) {
    OMPL_ERROR("Could not get problem definition for factor %s", name.c_str());
    throw "NotFound";
  }
  return pdef_iterator->second;
}

bool FibrationRRT::hasValidProblemDefinition_(const FactoredSpaceInformationPtr& factor) const {
  auto pdef_iterator = problem_definitions_per_factor_.find(factor->getName());
  if(pdef_iterator == problem_definitions_per_factor_.end()) {
    OMPL_ERROR("Could not get problem definition for factor %s", factor->getName().c_str());
    return false;
  }
  const auto& pdef = pdef_iterator->second;
  if(pdef->getStartStateCount() <= 0) {
    return false;
  }
  bool has_valid_start = false;
  if(factor->getStateValidityChecker() == nullptr) {
    OMPL_ERROR("No valid StateValidityCheckerPtr set in factor space %s.", factor->getName().c_str());
    return false;
  }
  for(size_t k = 0; k < pdef->getStartStateCount(); k++) {
    auto state = pdef->getStartState(k);
    if(factor->satisfiesBounds(state) && factor->isValid(state)) {
      has_valid_start = true;
    }
  }
  if(!has_valid_start) {
    OMPL_ERROR("No valid start state for factor %s (tried %d states).", factor->getName().c_str(), pdef->getStartStateCount());
    return false;
  }
  OMPL_DEBUG("Valid problem definition for factor %s", factor->getName().c_str());
  return true;
}

bool FibrationRRT::allChildrenHaveSolutions_(const FactoredSpaceInformationPtr& factor) const {
  if(!factor->hasChildren()) {
    return true;
  }
  const auto& children = factor->getChildren();

  //Debug
  OMPL_DEBUG("Factor %s has %d %s.", factor->getName().c_str(), children.size(), (children.size() > 1 ? "children" : "child"));
  for(const auto& child : children) {
    OMPL_DEBUG(" -- %s has %s.", child->getName().c_str(), (hasSolution_(child) ? "a solution" : "no solution"));
  }

  for(const auto& child : children) {
    if(!hasSolution_(child)) {
      return false;
    }
  }
  return true;
}

void FibrationRRT::setSeed(size_t seed) {
  rng_ = ompl::RNG(seed);
  seed_ = seed;
}

const FactoredSpaceInformationPtr& FibrationRRT::selectFactor_() {
  if(active_factors_.size() == 1) {
    return active_factors_.front();
  }
  //Exponential importance sampling
  PDF<int> pdf;

  for(size_t k = 0; k < active_factors_.size(); k++) {
    auto active_factor = active_factors_.at(k);
  //for(const auto& active_factor : active_factors_) {
    auto planner_iterator = active_planners_.find(active_factor->getName());
    if(planner_iterator == active_planners_.end()) {
      createPlannerForFactor_(active_factor);
    }
    planner_iterator = active_planners_.find(active_factor->getName());

    auto s = planner_iterator->second->getNumberOfSamples();
    auto N = active_factor->getStateDimension();
    auto importance = 1.0 / (pow(s, 1.0/double(N))+1.0);
    pdf.add(k, importance);
  }

  if (pdf.empty()) {
    OMPL_WARN("Could not construct PDF. Choosing random factor.");
    int index = rng_.uniformInt(0, active_factors_.size() - 1);
    return active_factors_.at(index);
  }
  auto index = pdf.sample(rng_.uniform01());
  return active_factors_.at(index);
}

void FibrationRRT::clear() {
  Planner::clear();
  iterations_ = 0;
}

void FibrationRRT::setup() {
  Planner::setup();
}

bool FibrationRRT::hasSolution_(const FactoredSpaceInformationPtr& factor) const {
  auto iterator = planner_status_per_factor_.find(factor->getName());
  if(iterator == planner_status_per_factor_.end()) {
    return false;
  }
  if( 
    planner_status_per_factor_.at(factor->getName())
    == base::PlannerStatus::StatusType::EXACT_SOLUTION) {
    return true;
  }
  auto pdef_iterator = problem_definitions_per_factor_.find(factor->getName());
  if(pdef_iterator == problem_definitions_per_factor_.end()) {
    OMPL_ERROR("Could not get problem definition for factor %s", factor->getName().c_str());
    return false;
  }
  return pdef_iterator->second->hasExactSolution();
}

bool FibrationRRT::isActive_(const FactoredSpaceInformationPtr& factor) const {
  auto iterator = is_active_.find(factor->getName());
  if(iterator == is_active_.end()) {
    return false;
  }
  return is_active_.at(factor->getName());
}

bool FibrationRRT::isSolved_(const FactoredSpaceInformationPtr& factor) const {
  auto iterator = is_solved_.find(factor->getName());
  if(iterator == is_solved_.end()) {
    return false;
  }
  return is_solved_.at(factor->getName());
}

size_t FibrationRRT::numFactors() const {
  return problem_definitions_per_factor_.size();
}

void FibrationRRT::grow_(const FactoredSpaceInformationPtr& factor) {
  std::string status_msg;
  if(hasSolution_(factor)) {
    status_msg = " [has solution]";
  }
  OMPL_DEBUG("Growing factor %s (%d/%d active factor%s)%s", factor->getName().c_str(), active_factors_.size(),
      numFactors(), (active_factors_.size() > 1 ? "s" : ""),
      status_msg.c_str());

  auto iterator = active_planners_.find(factor->getName());
  if(iterator == active_planners_.end()) {
    createPlannerForFactor_(factor);
  }
  auto& planner = active_planners_[factor->getName()];
  ompl::base::IterationTerminationCondition itc(kNumberOfIterationsPerPlannerCall);
  planner_status_per_factor_.insert({factor->getName(), planner->solve(itc)});
}

std::vector<FactoredPlannerPtr> FibrationRRT::getChildrenPlanner_(const FactoredSpaceInformationPtr& factor) const {
  std::vector<FactoredPlannerPtr> children_planner;
  if(!factor->hasChildren()) {
    return children_planner;
  }

  const auto& children = factor->getChildren();
  for(const auto& child : children) {
    child->getName();
    auto iterator = active_planners_.find(child->getName());
    if(iterator == active_planners_.end()) {
      OMPL_ERROR("Could not find a planner child with name %s for factor %s", child->getName().c_str(), factor->getName().c_str());
      throw "NotAPlanner";
    }
    children_planner.push_back(iterator->second);
  }
  return children_planner;
}

void FibrationRRT::setRange(double range) {
  range_ = range;
}

double FibrationRRT::getRange() const {
  if(range_.has_value()) {
    return range_.value();
  }
  return 0.0f;
}

void FibrationRRT::createPlannerForFactor_(const FactoredSpaceInformationPtr& factor) {
  const auto& name = factor->getName();

  if(!factor->hasChildren()) {
    active_planners_[name] = std::make_shared<FactoredPlanner>(factor);
  } else {
    auto children_planner = getChildrenPlanner_(factor);
    active_planners_[name] = std::make_shared<FactoredPlanner>(factor, children_planner);
  }
  if(range_.has_value()) {
    active_planners_[name]->setRange(range_.value());
  }
  OMPL_INFORM("Created new planner %s for factor %s", active_planners_[name]->getName().c_str(), name.c_str());

  auto iterator = problem_definitions_per_factor_.find(factor->getName());
  if(iterator == problem_definitions_per_factor_.end()) {
    OMPL_ERROR("Could not get problem definition for factor %s", name.c_str());
    return;
  }
  active_planners_[name]->setup();
  active_planners_[name]->setProblemDefinition(iterator->second);

  if(seed_.has_value()) {
    active_planners_[name]->setSeed(seed_.value());
  }
}

void FibrationRRT::setProblemDefinition(const base::ProblemDefinitionPtr &pdef) {
  Planner::setProblemDefinition(pdef);

  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);

  OMPL_INFORM("Set problem definition for factor %s", root->getName().c_str());

  problem_definitions_per_factor_.insert({root->getName(), pdef});
  if(!root->hasChildren()) {
    return;
  }

  if(pdef->getStartStateCount() != 1) {
    OMPL_ERROR("FibrationRRT can handle only a single start state, but you have %d states.", pdef->getStartStateCount());
    return;
  }

  const base::State* start = pdef->getStartState(0);
  const auto& goal_region = pdef->getGoal();

  for(const auto& child : root->getChildren()) {
    createProblemDefinition_(child, start, goal_region);
  }
}

void FibrationRRT::createProblemDefinition_(const FactoredSpaceInformationPtr& factor, const base::State* parent_start, const base::GoalPtr& parent_goal) {

  const auto& type = parent_goal->getType();

  if(type != base::GoalType::GOAL_STATE && type != base::GoalType::FACTORED_GOAL) {
    OMPL_ERROR("FibrationRRT can only handle a single goal state or a factored goal region.");
    throw "InvalidGoal";
  }

  base::ProblemDefinitionPtr pdef = std::make_shared<base::ProblemDefinition>(factor);
  const auto& projection = factor->getProjection();

  ////////////////////////////////////////////////////////////////////////////////
  // Project start state and add it to pdef
  ////////////////////////////////////////////////////////////////////////////////
  base::State* start = factor->allocState();
  projection->project(parent_start, start);

  std::stringstream ss_start;
  factor->printState(start, ss_start);

  pdef->addStartState(start);

  ////////////////////////////////////////////////////////////////////////////////
  // Project goal region and add it to pdef
  ////////////////////////////////////////////////////////////////////////////////
  if(type == base::GoalType::GOAL_STATE) {
    const base::State* parent_goal_state = parent_goal->as<base::GoalState>()->getState();
    base::State* goal_state = factor->allocState();
    projection->project(parent_goal_state, goal_state);
    std::stringstream ss_goal;
    factor->printState(goal_state, ss_goal);
    OMPL_INFORM("Project states onto factor %s \n Start %s Goal %s", 
        factor->getName().c_str(), ss_start.str().c_str(), ss_goal.str().c_str());

    auto goal = std::make_shared<base::GoalState>(factor);
    goal->setState(goal_state);
    goal->setThreshold(goal_threshold_);

    pdef->setGoal(goal);
  }
  if(type == base::GoalType::FACTORED_GOAL) {
    const auto& factored_goal = parent_goal->as<base::FactoredGoal>();
    const auto maybe_factor_goal = factored_goal->getFactorGoal(factor->getName());
    if(!maybe_factor_goal.has_value()) {
      OMPL_ERROR("Could not find factor goal for factor %s", factor->getName().c_str());
      throw "InvalidFactorGoal";
    }
    const auto factor_goal = maybe_factor_goal.value();
    OMPL_INFORM("Project states onto factor %s \n Start %s and Factored Goal", 
        factor->getName().c_str(), ss_start.str().c_str());
    pdef->setGoal(factor_goal);
  }

  problem_definitions_per_factor_[factor->getName()] = pdef;

  for(const auto& child : factor->getChildren()) {
    createProblemDefinition_(child, start, pdef->getGoal());
  }
}

std::string FibrationRRT::getIterationsProperty() const {
  return std::to_string(iterations_);
}

std::string FibrationRRT::getBestCostProperty() const {
  return std::to_string(bestCost_);
}

ompl::base::PlannerStatus FibrationRRT::solve(const ompl::base::PlannerTerminationCondition &ptc) {
    ////////////////////////////////////////////////////////////////////////////////
    const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
    active_factors_ = root->getLeafFactors();
    for(const auto& factor: active_factors_) {
      is_active_.insert({factor->getName(), true});
      is_solved_.insert({factor->getName(), false});
      if(!hasValidProblemDefinition_(factor)) {
        return base::PlannerStatus::INVALID_START;
      }
    }
    OMPL_INFORM("Solving FactoredSpaceInformation using %d active factors.", active_factors_.size());
    
    if(active_factors_.empty()) {
      OMPL_ERROR("Could not find any leaf nodes for factor %s", root->getName().c_str());
      return base::PlannerStatus(base::PlannerStatus::StatusType::ABORT);
    }

    planner_status_ = base::PlannerStatus(base::PlannerStatus::StatusType::TIMEOUT);
    while (!ptc) {
        iterations_++;

        OMPL_DEBUG("Iteration %d", iterations_);

        const auto& selectedFactor = selectFactor_();
        OMPL_DEBUG("Selected factor %s.", selectedFactor->getName().c_str());

        grow_(selectedFactor);

        if(hasSolution_(selectedFactor)) {
          if(!isSolved_(selectedFactor)) {
            is_solved_[selectedFactor->getName()] = true;
            OMPL_INFORM(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
            OMPL_INFORM(" >>> Solved factor %s.", selectedFactor->getName().c_str());
            OMPL_INFORM(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");

            if(shouldSmoothSolutionPath(selectedFactor)) {
              smoothSolutionPath(selectedFactor);
            }

            if(!selectedFactor->hasParent()) {
              const auto& name = selectedFactor->getName();
              OMPL_INFORM(" >>>>>> Found solution on root factor %s", name.c_str());
              planner_status_ = base::PlannerStatus::StatusType::EXACT_SOLUTION;
              continue;
            }
          }

          if(!selectedFactor->hasParent()) {
            continue;
          }

          const auto& parent = selectedFactor->getParent();
          if(!isActive_(parent) && allChildrenHaveSolutions_(parent)) {
            OMPL_INFORM("Add factor %s to active spaces.", parent->getName().c_str());
            active_factors_.push_back(parent);
            is_active_.insert({parent->getName(), true});
            is_solved_.insert({parent->getName(), false});
            if(!hasValidProblemDefinition_(parent)) {
              return base::PlannerStatus::INVALID_START;
            }
          }
        }
    }

    int num_all_factors = problem_definitions_per_factor_.size();
    int num_solved_factors = 0;
    for(const auto& active_factor : active_factors_) {
      if(is_solved_.at(active_factor->getName())) {
        num_solved_factors++;
      }
    }
    OMPL_INFORM(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
    OMPL_INFORM(" >>> Finished planning. Solved %d/%d factors (%d/%d active factors).", 
        num_solved_factors, num_all_factors, active_factors_.size(), num_all_factors);
    OMPL_INFORM(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
    if(pdef_->hasExactSolution()) {
      planner_status_ = base::PlannerStatus::StatusType::EXACT_SOLUTION;
      const auto pgeo = dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef_->getSolutionPath());
      if(pgeo) {
        OMPL_INFORM("Found exact solution of length %f with %d waypoints.", pgeo->length(), pgeo->getStateCount());
      }

    }
    return planner_status_;
}

bool FibrationRRT::shouldSmoothSolutionPath(const FactoredSpaceInformationPtr& factor) {
  if(smoothing_enabled_) {
    return true;
  }
  //Smoothing in higher dimension is often inefficient 
  if(factor->getStateDimension() > 4) {
    return false;
  }
  return true;
}

void FibrationRRT::smoothSolutionPath(const FactoredSpaceInformationPtr& factor) {
  auto pdef = getProblemDefinition(factor->getName());
  auto path = pdef->getSolutionPath();
  auto path_geometric = path->as<geometric::PathGeometric>();
  const size_t Nstates_before = path_geometric->getStateCount();
  auto simplifier = std::make_shared<geometric::PathSimplifier>(factor);
  simplifier->simplifyMax(*path_geometric);
  simplifier->simplifyMax(*path_geometric);
  const size_t Nstates_after = path_geometric->getStateCount();
  OMPL_DEBUG("Improved solution path from %d states to %d states.", Nstates_before, Nstates_after);
  for(const auto& state : path_geometric->getStates()) {
    factor->printState(state);
  }
  pdef->addSolutionPath(path);
}
