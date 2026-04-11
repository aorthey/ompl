#include "ompl/multilevel/planners/FactoredPlanner.h"
#include "ompl/multilevel/planners/RestrictionSampler.h"
#include "ompl/multilevel/datastructures/FactoredSpaceInformation.h"
#include <ompl/multilevel/datastructures/helpers/SamplingHelper.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionSideStep.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestrictionInterpolator.h>
#include <ompl/multilevel/datastructures/pathrestriction/ParallelFibrationSectionSolver.h>
#include <ompl/multilevel/datastructures/pathrestriction/PartialFibrationSectionSolver.h>

#include <string>

using namespace ompl::multilevel;

FactoredPlanner::FactoredPlanner(const FactoredSpaceInformationPtr& si, const std::vector<FactoredPlannerPtr>& children_planner) 
  : BaseTypePlanner(si), children_planner_(children_planner)
{
  setName("PlannerOn" + si->getName());

  internal_space_sampler_ = si_->allocStateSampler();

  if(!children_planner.empty()) 
  {
    sampler_ = std::make_shared<RestrictionSampler>(si, children_planner);
  }
}

ompl::base::State* FactoredPlanner::MakeStartState() const {
  auto state = getProblemDefinition()->getStartState(0);
  return state;
}

ompl::base::State* FactoredPlanner::MakeGoalState() const {
  base::Goal *goal = getProblemDefinition()->getGoal().get();
  auto *goal_s = static_cast<base::GoalSampleableRegion *>(goal);
  auto state = getSpaceInformation()->allocState();
  goal_s->sampleGoal(state);
  return state;
}

Expected<PathSectionPtr, std::string> FactoredPlanner::solveSection() {
  auto factor = std::static_pointer_cast<FactoredSpaceInformation>(si_);
  if (!factor->hasChildren()) 
  {
    return failure("Factor has no children.");
  }
    
  auto children = factor->getChildren();
  if (children.size() == 0)
  {
    return failure("Factor has no children.");
  }

  auto qGoal = MakeGoalState();

////////////////////////////////////////////////////////////////////////////////
// Sequential/Partial fibration
////////////////////////////////////////////////////////////////////////////////
  if (children.size() == 1)
  {
    if (children_planner_.size() != 1)
    {
      return failure("Children planner size has to be equivalent to children size.");
    }

    auto child = children.front();

    auto projection = child->getProjection();
    if (projection == nullptr)
    {
      return failure("Child has no projection.");
    }

    auto child_planner = children_planner_.front();
    const auto& pdef = child_planner->getProblemDefinition();
    if(!pdef->hasSolution()) 
    {
      return failure("Base space has no solution.");
    }

    const auto base_path = pdef->getSolutionPath();

    auto path_restriction = std::make_shared<PathRestriction>(factor, projection);
    path_restriction->setBasePath(base_path);

    if(projection->isFibered())
    {
      auto find_section = std::make_shared<FindSectionSideStep>(path_restriction);
      ompl::time::point tStart = ompl::time::now();
      auto maybe_section = find_section->solve(tree_, qGoal);
      ompl::time::point tEnd = ompl::time::now();

      if(!maybe_section.has_value()) {
        return failure("Timeout after " + std::to_string(ompl::time::seconds(tEnd - tStart)) + "s");
      }
      return success(maybe_section.value());
    } else {
      auto maybe_section = partialFibrationSectionSolver(factor, tree_, path_restriction, qGoal);
      if(!maybe_section.has_value()) {
        return failure("Could not find section");
      }
      return success(maybe_section.value());
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Parallel fibration
  ////////////////////////////////////////////////////////////////////////////////
  std::unordered_map<std::string, PathRestrictionPtr> path_restrictions;

  for(const auto& child_planner : children_planner_) {
    auto child = std::static_pointer_cast<FactoredSpaceInformation>(child_planner->getSpaceInformation());
    auto projection = child->getProjection();
    if (projection == nullptr)
    {
      return failure("Child has no projection.");
    }
    auto pdef = child_planner->getProblemDefinition();
    if(!pdef->hasSolution()) 
    {
      return failure("Base space has no solution.");
    }
    const auto base_path = pdef->getSolutionPath();

    auto path_restriction = std::make_shared<PathRestriction>(factor, projection);
    path_restriction->setBasePath(base_path);
    path_restrictions[child->getName()] = path_restriction;
  }

  auto maybe_section = parallelFibrationSectionSolver(factor, tree_, path_restrictions);
  if(maybe_section.has_value()) {
    return success(maybe_section.value());
  }
  return failure("Could not find section");
}

ompl::base::PlannerStatus FactoredPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc) 
{
  if(first_run_) {
    first_run_ = false;

    checkValidity();
    makeRootNode();

    if (tree_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if(use_section_search_) {
      auto maybe_section = solveSection();
      if(maybe_section.has_value()) {
          base::Goal *goal = pdef_->getGoal().get();
          if(goal->isSatisfied(maybe_section.value()->back())) {
              auto nodes = tree_->getNodes();
              for(const auto& node : nodes) {
                  if(goal->isSatisfied(node->getState())) {
                      makeSolutionPath(node, false, 0.0);
                      return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
                  }
              }
          }
      }
    }
  }
  return BaseTypePlanner::solve(ptc);
}

void FactoredPlanner::clear() {
}

void FactoredPlanner::setPathRestrictionSamplingBias(double path_restriction_sampling_bias) {
  path_restriction_sampling_bias_ = path_restriction_sampling_bias;
}

double FactoredPlanner::getPathRestrictionSamplingBias() const {
  return path_restriction_sampling_bias_;
}

void FactoredPlanner::setPathRestrictionSurroundingSamplingBias(double path_restriction_surrounding_sampling_bias) {
  path_restriction_surrounding_sampling_bias_ = path_restriction_surrounding_sampling_bias;
}

double FactoredPlanner::getPathRestrictionSurroundingSamplingBias() const {
  return path_restriction_surrounding_sampling_bias_;
}

void FactoredPlanner::setSamplingPerturbationBias(double sampling_perturbation_bias) {
  sampling_perturbation_bias_ = sampling_perturbation_bias;
}

double FactoredPlanner::getSamplingPerturbationBias() const {
  return sampling_perturbation_bias_;
}

void FactoredPlanner::setEnableSectionSearch() {
  use_section_search_ = true;
}
void FactoredPlanner::setDisableSectionSearch() {
  use_section_search_ = false;
}

void FactoredPlanner::setSeed(size_t seed) 
{
  //ompl::RNG::setSeed(seed);
  rng_ = ompl::RNG(seed);
}


size_t FactoredPlanner::getNumberOfSamples() const {
  return tree_->size();
}

void FactoredPlanner::sampleFromDatastructure(ompl::base::State* state) {
  ompl::multilevel::DatastructureInformation info;
  info.path_restriction_surrounding_sampling_bias = getPathRestrictionSurroundingSamplingBias();
  info.path_restriction_sampling_bias = getPathRestrictionSamplingBias();
  info.sampling_perturbation_bias = getSamplingPerturbationBias();
  info.sampler = internal_space_sampler_;
  info.pdef = getProblemDefinition();
  info.si = si_;
  info.number_of_nodes = tree_->size();

  ::sampleFromDatastructure(info, *tree_, rng_, state);
}
