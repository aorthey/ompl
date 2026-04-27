#include <ompl/multilevel/planners/FibrationRRT.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/FactoredGoal.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/ProblemDefinitionHelper.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/util/Time.h>
#include <ompl/multilevel/planners/FactoredPlanner.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/tools/config/SelfConfig.h>

using namespace ompl::multilevel;

FibrationRRT::FibrationRRT(const ompl::base::SpaceInformationPtr &si, float goal_threshold) :
  ompl::base::Planner(si, "FibrationRRT"), goal_threshold_(goal_threshold) {
  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.approximateSolutions = true;
  specs_.directed = true;
  //specs_.optimizingPaths = false;

  //addPlannerProgressProperty("iterations INTEGER", [this] { return getIterationsProperty(); });
  //addPlannerProgressProperty("best cost REAL", [this] { return getBestCostProperty(); });

  Planner::declareParam<double>("range", this, 
      &FibrationRRT::setRange, &FibrationRRT::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, 
      &FibrationRRT::setGoalBias, &FibrationRRT::getGoalBias, "0.0:0.05:1.0");
  // Planner::declareParam<double>("path_restriction_sampling_bias", this, 
  //     &FibrationRRT::setPathRestrictionSamplingBias, &FibrationRRT::getPathRestrictionSamplingBias, "0.0:0.0:1.0");
  // Planner::declareParam<double>("path_restriction_surrounding_sampling_bias", this, 
  //     &FibrationRRT::setPathRestrictionSurroundingSamplingBias, &FibrationRRT::getPathRestrictionSurroundingSamplingBias, "0.0:0.0:1.0");
  // Planner::declareParam<double>("sampling_perturbation_bias", this, 
  //     &FibrationRRT::setSamplingPerturbationBias, &FibrationRRT::getSamplingPerturbationBias, "0.:0.0:10.0");

}

FibrationRRT::FibrationRRT(const FactoredSpaceInformationPtr &factor, float goal_threshold) :
   FibrationRRT(std::static_pointer_cast<ompl::base::SpaceInformation>(factor), goal_threshold) {
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
  tools::SelfConfig sc(si_, getName());

  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  auto name = root->getName();
  num_factors_ = root->getAllFactors().size();
  double range = 0.0;
  sc.configurePlannerRange(range);
  range_[name] = range;
  goal_bias_[name] = kGoalBiasDefault;
  path_restriction_sampling_bias_[name] = kDefaultPathRestrictionSamplingBias;
  path_restriction_surrounding_sampling_bias_[name] = kDefaultPathRestrictionSurroundingBias;
  sampling_perturbation_bias_[name] = kDefaultSamplingPerturbationValue;

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
    OMPL_ERROR(">>No valid start state for factor %s (tried %d states).", factor->getName().c_str(), pdef->getStartStateCount());
    for(size_t k = 0; k < pdef->getStartStateCount(); k++) {
      auto state = pdef->getStartState(k);
      if(!factor->satisfiesBounds(state)) {
        OMPL_ERROR("State does not satisfy bounds:");
        factor->printState(state);
      }
      if(!factor->isValid(state)) {
        OMPL_ERROR("State is in collision:");
        factor->printState(state);
      }
    }
    return false;
  }
  //OMPL_DEVMSG2("Valid problem definition for factor %s", factor->getName().c_str());
  return true;
}

bool FibrationRRT::hasNonSolvedSiblings_(const FactoredSpaceInformationPtr& factor) const {
  if(!factor->hasParent()) {
    //no parent, no siblings
    return false;
  }
  auto current = factor; 
  auto parent = factor->getParent();
  while(isSolved_(parent) && parent->hasParent()) {
    current = parent;
    parent = parent->getParent();
  }

  if(parent->getTotalNumChildren() == 1) {
    //only child has no siblings
    return false;
  }

  auto children = parent->getChildren();
  for(const auto& child : children) {
    if(child == current) {
      continue;
    }
    if(!isSolved_(child)) {
      return true;
    }
  }
  return false;
}

bool FibrationRRT::allChildrenHaveSolutions_(const FactoredSpaceInformationPtr& factor) const {
  if(!factor->hasChildren()) {
    return true;
  }
  const auto& children = factor->getChildren();

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
    if(isSolved_(active_factor) && hasNonSolvedSiblings_(active_factor)) {
      //ignore factors with non-solved siblings because we cannot continue if
      //not all siblings are solved
      continue;
    }
    auto planner_iterator = active_planners_.find(active_factor->getName());
    if(planner_iterator == active_planners_.end()) {
      createPlannerForFactor_(active_factor);
    }
    planner_iterator = active_planners_.find(active_factor->getName());

    auto p = 1.0 / active_factor->getStateDimension();
    auto N = (double) planner_iterator->second->getNumberOfSamples();
    auto Nd = powf(N, p);
    auto importance = 1.0 / (Nd + 1.0);
    //return 1.0 / (Nd + 1.0);
    // OMPL_DEBUG("Eval space %s : %f (Nd = %f, N = %f, p = %f).", active_factor->getName().c_str(), 
    //     importance, Nd, N, p);
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
  std::vector<size_t> non_solution_indices;
  for(size_t index = 0; index < active_factors_.size(); index++) {
    auto active_factor = active_factors_.at(index);
    if(!hasSolution_(active_factor)) {
      non_solution_indices.push_back(index);
    }
  }
  if(non_solution_indices.empty()) {
    OMPL_ERROR("No active factors without a solution.");
  }
  int index = rng_.uniformInt(0, non_solution_indices.size() - 1);
  return active_factors_.at(non_solution_indices.at(index));
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
  auto iterator = active_planners_.find(factor->getName());
  if(iterator == active_planners_.end()) {
    createPlannerForFactor_(factor);
  }
  auto& planner = active_planners_.at(factor->getName());
  ompl::base::IterationTerminationCondition itc(kNumberOfIterationsPerPlannerCall);
  auto planner_status = planner->solve(itc);

  auto name = factor->getName();
  if(planner_status_per_factor_.count(name)){
    planner_status_per_factor_.at(name) = planner_status;
  } else {
    planner_status_per_factor_.insert({name, planner_status});
  }
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
      throw std::runtime_error("Could not find planner " + child->getName());
    }
    children_planner.push_back(iterator->second);
  }
  return children_planner;
}

////////////////////////////////////////////////////////////////////////////////
/// Parameters
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//Range parameter
////////////////////////////////////////////////////////////////////////////////
double FibrationRRT::getRange() const {
  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  return range_.at(root->getName());
}

void FibrationRRT::setRange(double range) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setLocalRange(factor->getName(), range);
  }
}

void FibrationRRT::setLocalRange(const std::string& name, double range) {
  for(const auto& pair : range_) {
    OMPL_INFORM("%s, %f", pair.first.c_str(), pair.second);
  }
  OMPL_INFORM("Set local range %s to %f", name.c_str(), range);
  range_[name] = range;
}
////////////////////////////////////////////////////////////////////////////////
//Goal bias parameter
////////////////////////////////////////////////////////////////////////////////
double FibrationRRT::getGoalBias() const {
  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  return goal_bias_.at(root->getName());
}

void FibrationRRT::setGoalBias(double goal_bias) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setLocalGoalBias(factor->getName(), goal_bias);
  }
}

void FibrationRRT::setLocalGoalBias(const std::string& name, double goal_bias) {
  goal_bias_[name] = goal_bias;
}

////////////////////////////////////////////////////////////////////////////////
//Path restriction sampling bias
////////////////////////////////////////////////////////////////////////////////

double FibrationRRT::getPathRestrictionSamplingBias() const {
  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  return path_restriction_sampling_bias_.at(root->getName());
}

void FibrationRRT::setPathRestrictionSamplingBias(double value) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setLocalPathRestrictionSamplingBias(factor->getName(), value);
  }
}

void FibrationRRT::setLocalPathRestrictionSamplingBias(const std::string& name, double path_restriction_sampling_bias) {
  path_restriction_sampling_bias_[name] = path_restriction_sampling_bias;
}

////////////////////////////////////////////////////////////////////////////////
//Path restriction surrounding sampling bias
////////////////////////////////////////////////////////////////////////////////

double FibrationRRT::getPathRestrictionSurroundingSamplingBias() const {
  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  return path_restriction_surrounding_sampling_bias_.at(root->getName());
}

void FibrationRRT::setPathRestrictionSurroundingSamplingBias(double value) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setLocalPathRestrictionSurroundingSamplingBias(factor->getName(), value);
  }
}

void FibrationRRT::setLocalPathRestrictionSurroundingSamplingBias(const std::string& name, double path_restriction_surrounding_sampling_bias) {
  path_restriction_surrounding_sampling_bias_[name] = path_restriction_surrounding_sampling_bias;
}


////////////////////////////////////////////////////////////////////////////////
//Sampling perturbation bias
////////////////////////////////////////////////////////////////////////////////

double FibrationRRT::getSamplingPerturbationBias() const {
  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  return sampling_perturbation_bias_.at(root->getName());
}

void FibrationRRT::setSamplingPerturbationBias(double value) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setLocalSamplingPerturbationBias(factor->getName(), value);
  }
}

void FibrationRRT::setLocalSamplingPerturbationBias(const std::string& name, double sampling_perturbation_bias) {
  sampling_perturbation_bias_[name] = sampling_perturbation_bias;
}


////////////////////////////////////////////////////////////////////////////////
//Other parameters
////////////////////////////////////////////////////////////////////////////////
void FibrationRRT::setSmoothIntermediateSolutions(bool smooth_intermediate_solutions) {
  auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  for(const auto& factor : root->getAllFactors()) {
    setLocalSmoothIntermediateSolutions(factor->getName(), smooth_intermediate_solutions);
  }
}

void FibrationRRT::setDisableSectionSearch() {
  use_section_search_ = false;
}

void FibrationRRT::setEnableSectionSearch() {
  use_section_search_ = true;
}

void FibrationRRT::setLocalSmoothIntermediateSolutions(const std::string& name, bool smooth_intermediate_solutions) {
  smooth_intermediate_solutions_[name] = smooth_intermediate_solutions;
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

  if(seed_.has_value()) {
    active_planners_.at(name)->setSeed(seed_.value());
  }

  if(use_section_search_) {
    active_planners_.at(name)->setEnableSectionSearch();
  } else {
    active_planners_.at(name)->setDisableSectionSearch();
  }

  //Setting local or global parameter values 
  auto maybe_range = getParameter(name, range_);
  if(maybe_range.has_value()) {
    active_planners_.at(name)->setRange(maybe_range.value());
    OMPL_INFORM("Set planner range for %s to %f", name.c_str(), active_planners_.at(name)->getRange());
  }
  auto maybe_goal_bias = getParameter(name, goal_bias_);
  if(maybe_goal_bias.has_value()) {
    active_planners_.at(name)->setGoalBias(maybe_goal_bias.value());
  }
  auto maybe_path_restriction_sampling_bias_= getParameter(name, path_restriction_sampling_bias_);
  if(maybe_path_restriction_sampling_bias_.has_value()) {
    active_planners_.at(name)->setPathRestrictionSamplingBias(maybe_path_restriction_sampling_bias_.value());
  }
  auto maybe_path_restriction_surrounding_sampling_bias_= getParameter(name, path_restriction_surrounding_sampling_bias_);
  if(maybe_path_restriction_surrounding_sampling_bias_.has_value()) {
    active_planners_.at(name)->setPathRestrictionSurroundingSamplingBias(maybe_path_restriction_surrounding_sampling_bias_.value());
  }
  auto maybe_sampling_perturbation_bias_= getParameter(name, sampling_perturbation_bias_);
  if(maybe_sampling_perturbation_bias_.has_value()) {
    active_planners_.at(name)->setSamplingPerturbationBias(maybe_sampling_perturbation_bias_.value());
  }

  auto iterator = problem_definitions_per_factor_.find(factor->getName());
  if(iterator == problem_definitions_per_factor_.end()) {
    OMPL_ERROR("Could not get problem definition for factor %s", name.c_str());
    return;
  }
  active_planners_.at(name)->setup();
  active_planners_.at(name)->setProblemDefinition(iterator->second);

}

void FibrationRRT::setProblemDefinition(const base::ProblemDefinitionPtr &pdef) {
  Planner::setProblemDefinition(pdef);
  const auto root = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  problem_definitions_per_factor_ = computeProblemDefinitions(root, pdef, goal_threshold_);
}

std::string FibrationRRT::getIterationsProperty() const {
  return std::to_string(iterations_);
}

std::string FibrationRRT::getBestCostProperty() const {
  return std::to_string(bestCost_);
}

size_t FibrationRRT::getNumberOfIterations() const {
  return iterations_;
}

std::optional<ompl::base::PlannerStatus> FibrationRRT::checkForInvalidPlannerStatus_() const {
  for(const auto& planner_status : planner_status_per_factor_) {
    if(planner_status.second == base::PlannerStatus::UNKNOWN) {
      continue;
    }
    if(planner_status.second == base::PlannerStatus::APPROXIMATE_SOLUTION) {
      continue;
    }
    if(planner_status.second == base::PlannerStatus::EXACT_SOLUTION) {
      continue;
    }
    if(planner_status.second == base::PlannerStatus::TIMEOUT) {
      continue;
    }
    return planner_status.second;
  }
  return std::nullopt;
}

size_t FibrationRRT::getNumSolvedFactors() const{
  return std::count_if(is_solved_.cbegin(), is_solved_.cend(), [](const auto& entry) { return entry.second; });
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
    if(active_factors_.empty()) {
      OMPL_ERROR("Could not find any leaf nodes for factor %s", root->getName().c_str());
      return base::PlannerStatus(base::PlannerStatus::StatusType::ABORT);
    }

    planner_status_ = base::PlannerStatus(base::PlannerStatus::StatusType::TIMEOUT);

    ompl::time::point t_start = ompl::time::now();
    while (!ptc) {
        iterations_++;

        const auto& selectedFactor = selectFactor_();

        grow_(selectedFactor);

        auto maybe_invalid_planner_status = checkForInvalidPlannerStatus_();
        if(maybe_invalid_planner_status.has_value()) {
          planner_status_ = maybe_invalid_planner_status.value();
          OMPL_WARN("Planner %s has planner status %s", selectedFactor->getName().c_str(), planner_status_.asString().c_str());
          return planner_status_;
        }

        if(hasSolution_(selectedFactor)) {
          if(!isSolved_(selectedFactor)) {
            const auto& name = selectedFactor->getName();
            is_solved_.at(name) = true;

            double t_k_end = ompl::time::seconds(ompl::time::now() - t_start);
            OMPL_DEBUG("Solved factor %s (solved %d/%d factors) after %f seconds.", name.c_str(), getNumSolvedFactors(), num_factors_, t_k_end);

            if(shouldSmoothSolutionPath(selectedFactor)) {
              smoothSolutionPath(selectedFactor);
            }

            if(!selectedFactor->hasParent()) {
              OMPL_DEBUG("Found solution on root factor %s", name.c_str());
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
            active_factors_.push_back(parent);
            is_active_.insert({parent->getName(), true});
            is_solved_.insert({parent->getName(), false});
            if(!hasValidProblemDefinition_(parent)) {
              return base::PlannerStatus::INVALID_START;
            }
          }
        }
    }

    int num_solved_factors = 0;
    for(const auto& active_factor : active_factors_) {
      if(is_solved_.at(active_factor->getName())) {
        num_solved_factors++;
      }
    }
    OMPL_INFORM(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
    OMPL_INFORM(" >>> Finished planning. Solved %d/%d factors (%d/%d active factors).", 
        num_solved_factors, num_factors_, active_factors_.size(), num_factors_);
    OMPL_INFORM(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
    if(pdef_->hasExactSolution()) {
      planner_status_ = base::PlannerStatus::StatusType::EXACT_SOLUTION;
      const auto pgeo = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef_->getSolutionPath());
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
  pdef->addSolutionPath(path, false, 0.0, getName());
}
