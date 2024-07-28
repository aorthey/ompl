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

FibrationRRT::FibrationRRT(const ompl::base::SpaceInformationPtr &si, float goal_threshold) :
  ompl::base::Planner(si, "FibrationRRT"), goal_threshold_(goal_threshold) {
  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.approximateSolutions = true;
  specs_.directed = true;
  //specs_.optimizingPaths = false;

  //addPlannerProgressProperty("iterations INTEGER", [this] { return getIterationsProperty(); });
  //addPlannerProgressProperty("best cost REAL", [this] { return getBestCostProperty(); });

  //Planner::declareParam<double>("range", this, &FibrationRRT::setRange, &FibrationRRT::getRange, "0.:1.:10000.");
  //Planner::declareParam<bool>("smoothIntermediateSolutions", this, &FibrationRRT::setSmoothIntermediateSolutions, &FibrationRRT::getSmoothIntermediateSolutions, "0,1");
}

FibrationRRT::FibrationRRT(const FactoredSpaceInformationPtr &factor, float goal_threshold) :
   FibrationRRT(static_pointer_cast<ompl::base::SpaceInformation>(factor), goal_threshold) {
}

FibrationRRT::~FibrationRRT() {
  clear();
}

void FibrationRRT::clear() {
  for(const auto& planner : active_planners_) {
    planner.second->clear();
  }
  active_planners_.clear();
  active_factors_.clear();
  is_active_.clear();
  is_solved_.clear();
  planner_status_per_factor_.clear();
  iterations_ = 0;
  
  planner_status_ = base::PlannerStatus::StatusType::UNKNOWN;
  bestCost_ = std::numeric_limits<float>::infinity();
  for(auto& problem_definition : problem_definitions_per_factor_) {
    problem_definition.second->clearSolutionPaths();
  }
  Planner::clear();
}

void FibrationRRT::setup() {
  Planner::setup();
}

void FibrationRRT::getPlannerData(base::PlannerData &data) const {
  auto si = data.getSpaceInformation();
  const auto factor = std::static_pointer_cast<FactoredSpaceInformation>(si);
  auto name = factor->getName();
  auto planner = active_planners_.find(name);
  if(planner == active_planners_.end()) {
    OMPL_ERROR("Could not get planner data for factor %s", name.c_str());
    return;
  }
  planner->second->getPlannerData(data);
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
    throw ompl::Exception("NotFound");
  }
  return pdef_iterator->second;
}

FactoredPlannerPtr FibrationRRT::getPlanner(const std::string& name) const {
  auto planner_iterator = active_planners_.find(name);
  if(planner_iterator == active_planners_.end()) {
    OMPL_ERROR("Could not get planner for factor %s", name.c_str());
    throw ompl::Exception("NotFound");
  }
  return planner_iterator->second;
}

bool FibrationRRT::hasValidProblemDefinition_(const FactoredSpaceInformationPtr& factor) const {
  auto pdef_iterator = problem_definitions_per_factor_.find(factor->getName());
  if(pdef_iterator == problem_definitions_per_factor_.end()) {
    OMPL_ERROR("Could not get problem definition for factor %s", factor->getName().c_str());
    return false;
  }
  const auto& pdef = pdef_iterator->second;
  if(pdef->getStartStateCount() <= 0) {
    OMPL_ERROR("No start states available in ProblemDefinition.");
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

const FactoredSpaceInformationPtr& FibrationRRT::selectFactorExponential_() {
  //Exponential importance sampling
  PDF<int> pdf;

  for(size_t k = 0; k < active_factors_.size(); k++) {
    auto active_factor = active_factors_.at(k);
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
    return selectFactorUniform_();
  }
  auto index = pdf.sample(rng_.uniform01());
  return active_factors_.at(index);
}

const FactoredSpaceInformationPtr& FibrationRRT::selectFactorUniform_() {
  int index = rng_.uniformInt(0, active_factors_.size() - 1);
  return active_factors_.at(index);
}

const FactoredSpaceInformationPtr& FibrationRRT::selectFactorLastLevel_() {
  return active_factors_.back();
}

const FactoredSpaceInformationPtr& FibrationRRT::selectFactor_() {
  if(active_factors_.size() == 1) {
    return active_factors_.front();
  }
  switch(selector_function_type_) {
    case SelectorFunctionType::kUniform: {
      return selectFactorUniform_(); 
    }
    case SelectorFunctionType::kExponential: {
      return selectFactorExponential_(); 
    }
    case SelectorFunctionType::kLastLevel: {
      return selectFactorLastLevel_(); 
    }
    default: {
      OMPL_WARN("No selector function specified. Using uniform.");
      return selectFactorUniform_(); 
    }
  }
}

void FibrationRRT::setSelectorFunctionType(const SelectorFunctionType& selector_function_type) {
  selector_function_type_ = selector_function_type;
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

void FibrationRRT::setSmoothIntermediateSolutions(bool smooth_intermediate_solutions) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setSmoothIntermediateSolutions(factor->getName(), smooth_intermediate_solutions);
  }
}

void FibrationRRT::setRange(double range) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setRange(factor->getName(), range);
  }
}

void FibrationRRT::setGoalBias(double goal_bias) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setGoalBias(factor->getName(), goal_bias);
  }
}

void FibrationRRT::setSmoothIntermediateSolutions(const std::string& name, bool smooth_intermediate_solutions) {
  smooth_intermediate_solutions_[name] = smooth_intermediate_solutions;
}

void FibrationRRT::setRange(const std::string& name, double range) {
  range_[name] = range;
}

void FibrationRRT::setGoalBias(const std::string& name, double goal_bias) {
  goal_bias_[name] = goal_bias;
}

void FibrationRRT::setPathRestrictionSamplingBias(const std::string& name, double path_restriction_sampling_bias) {
  path_restriction_sampling_bias_[name] = path_restriction_sampling_bias;
}

void FibrationRRT::setPathRestrictionSurroundingSamplingBias(const std::string& name, double path_restriction_surrounding_sampling_bias) {
  path_restriction_surrounding_sampling_bias_[name] = path_restriction_surrounding_sampling_bias;
}

void FibrationRRT::setSamplingPerturbationBias(const std::string& name, double sampling_perturbation_bias) {
  sampling_perturbation_bias_[name] = sampling_perturbation_bias;
}


template<typename T>
std::optional<T> getParameter(const std::string& name, const std::unordered_map<std::string, T>& data) {
  auto data_iterator = data.find(name);
  if(data_iterator == data.end()) {
    return std::nullopt;
  }
  return data_iterator->second;
}

void FibrationRRT::createPlannerForFactor_(const FactoredSpaceInformationPtr& factor) {
  const auto& name = factor->getName();

  if(!factor->hasChildren()) {
    active_planners_[name] = std::make_shared<FactoredPlanner>(factor);
  } else {
    auto children_planner = getChildrenPlanner_(factor);
    active_planners_[name] = std::make_shared<FactoredPlanner>(factor, children_planner);
  }

  //Setting local or global parameter values 
  auto maybe_range = getParameter(name, range_);
  if(maybe_range.has_value()) {
    active_planners_[name]->setRange(maybe_range.value());
  }
  auto maybe_goal_bias = getParameter(name, goal_bias_);
  if(maybe_goal_bias.has_value()) {
    active_planners_[name]->setGoalBias(maybe_goal_bias.value());
  }
  auto maybe_path_restriction_sampling_bias_= getParameter(name, path_restriction_sampling_bias_);
  if(maybe_path_restriction_sampling_bias_.has_value()) {
    active_planners_[name]->setPathRestrictionSamplingBias(maybe_path_restriction_sampling_bias_.value());
  }
  auto maybe_path_restriction_surrounding_sampling_bias_= getParameter(name, path_restriction_surrounding_sampling_bias_);
  if(maybe_path_restriction_surrounding_sampling_bias_.has_value()) {
    active_planners_[name]->setPathRestrictionSurroundingSamplingBias(maybe_path_restriction_surrounding_sampling_bias_.value());
  }
  auto maybe_sampling_perturbation_bias_= getParameter(name, sampling_perturbation_bias_);
  if(maybe_sampling_perturbation_bias_.has_value()) {
    active_planners_[name]->setSamplingPerturbationBias(maybe_sampling_perturbation_bias_.value());
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
        OMPL_ERROR("Factor %s has no valid problem definition.", factor->getName().c_str());
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
            const auto& name = selectedFactor->getName();
            is_solved_[name] = true;
            OMPL_INFORM(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
            OMPL_INFORM(" >>> Solved factor %s.", name.c_str());
            OMPL_INFORM(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");

            if(shouldSmoothSolutionPath(selectedFactor)) {
              smoothSolutionPath(selectedFactor);
            }
            //DEBUG
            // const auto pgeo = static_pointer_cast<ompl::geometric::PathGeometric>(getProblemDefinition(name)->getSolutionPath());
            // for(const auto& state : pgeo->getStates()) {
            //   selectedFactor->printState(state);
            // }
            //DEBUG


            if(!selectedFactor->hasParent()) {
              OMPL_INFORM(" >>>>>> Found solution on root factor %s", name.c_str());
              planner_status_ = base::PlannerStatus::StatusType::EXACT_SOLUTION;
              auto pdef = getProblemDefinition(name);
              auto path = pdef->getSolutionPath();
              pdef_->addSolutionPath(path);
              break;
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
      const auto pgeo = static_pointer_cast<ompl::geometric::PathGeometric>(pdef_->getSolutionPath());
      if(pgeo) {
        OMPL_INFORM("Found exact solution of length %f with %d waypoints.", pgeo->length(), pgeo->getStateCount());
      }
    }
    return planner_status_;
}

bool FibrationRRT::shouldSmoothSolutionPath(const FactoredSpaceInformationPtr& factor) {

  auto maybe_smooth_intermediate_solution = getParameter(factor->getName(), smooth_intermediate_solutions_);
  if(!maybe_smooth_intermediate_solution.has_value()) {
    //Enabled by default
    return true;
  }
  return maybe_smooth_intermediate_solution.value();
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
  // for(const auto& state : path_geometric->getStates()) {
  //   factor->printState(state);
  // }
  pdef->addSolutionPath(path, false, 0.0, getName());
}
