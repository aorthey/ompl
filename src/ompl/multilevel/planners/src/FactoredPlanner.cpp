#include "ompl/multilevel/planners/FactoredPlanner.h"
#include "ompl/multilevel/planners/RestrictionSampler.h"
#include "ompl/multilevel/datastructures/FactoredSpaceInformation.h"
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
      OMPL_ERROR("SectionSearch: Sequential Fibration");
      auto find_section = std::make_shared<FindSectionSideStep>(path_restriction);
      ompl::time::point tStart = ompl::time::now();
      auto maybe_section = find_section->solve(tree_, qGoal);
      ompl::time::point tEnd = ompl::time::now();

      if(!maybe_section.has_value()) {
        return failure("Timeout after " + std::to_string(ompl::time::seconds(tEnd - tStart)) + "s");
      }
      return success(maybe_section.value());
    } else {
      OMPL_ERROR("SectionSearch: Partial Fibration");
      auto maybe_section = partialFibrationSectionSolver(factor, tree_, path_restriction, qGoal);
      if(!maybe_section.has_value()) {
        OMPL_ERROR("Found no section");
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

    auto maybe_section = solveSection();
    if(maybe_section.has_value()) {
        OMPL_WARN("Found section");
        base::Goal *goal = pdef_->getGoal().get();
        if(goal->isSatisfied(maybe_section.value()->back())) {
          auto nodes = tree_->getNodes();
          for(const auto& node : nodes) {
            if(goal->isSatisfied(node->getState())) {
              makeSolutionPath(node, false, 0.0);
              OMPL_WARN("Return exact");
              return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
            }
          }
        } else {
          OMPL_WARN("Found section, but last state is not in goal");
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

void FactoredPlanner::setSeed(size_t seed) 
{
  //ompl::RNG::setSeed(seed);
  rng_ = ompl::RNG(seed);
}

void FactoredPlanner::sampleFromPath(const std::vector<base::State *>& path_states, ompl::base::State* state) 
{
  std::vector<double> distances;
  for (unsigned int i = 1; i < path_states.size(); i++)
  {
      const double d = si_->distance(path_states.at(i - 1), path_states.at(i));
      distances.push_back(d);
  }
  const double path_length = std::accumulate(distances.begin(), distances.end(), 0.0f);
  if(path_length < 1e-3) {
    OMPL_WARN("Path sampler uses path of length %f", path_length);
    si_->copyState(state, path_states.front());
    return;
  }
  const double random_position_on_path = rng_.uniformReal(0, path_length);

  double current_distance = 0.0;
  for (unsigned int i = 0; i < distances.size(); i++)
  {
      const double& d = distances.at(i);
      current_distance += d;
      if(current_distance > random_position_on_path) 
      {
        //r lies between path_states i+1 and i
        const auto s1 = path_states.at(i);
        const auto s2 = path_states.at(i + 1);

        //   |--------------| d
        //             |----| current_distance-random_position_on_path
        //   |---------|      d - (current_distance-random_position_on_path)
        //---|---------x----|------
        //   s1        r    s2
        if(d > 1e-3) 
        {
          const double s = (d - (current_distance -random_position_on_path))/d; //in [0,1]
          si_->getStateSpace()->interpolate(s1, s2, s, state);
        }else {
          si_->copyState(state, s1);
        }
        return;
      }
  }
  OMPL_ERROR("Path sampler reached end of method with length %f and random value %f", path_length, random_position_on_path);
}

size_t FactoredPlanner::getNumberOfSamples() const {
  return tree_->size();
}

void FactoredPlanner::sampleFromDatastructure(ompl::base::State* state) 
{

    //Path restriction sampling
    if(path_restriction_sampling_bias_ > 0.0) {
      if(rng_.uniform01() < path_restriction_sampling_bias_) {
        const auto& pdef = getProblemDefinition();
        if(!pdef->hasSolution()) 
        {
          OMPL_ERROR("Cannot sample from space without a solution.");
          return;
        }
        const auto path = pdef->getSolutionPath()->as<geometric::PathGeometric>();
        const std::vector<base::State *>& path_states = path->getStates();
        sampleFromPath(path_states, state);
        if(path_restriction_surrounding_sampling_bias_ > 0.0) {
          internal_space_sampler_->sampleUniformNear(state, state, path_restriction_surrounding_sampling_bias_);
        }
        return;
      }
    }

    //Tree restriction sampling (vertex version). RRTConnect.
    //const size_t N = tStart_->size() + tGoal_->size();
    //const size_t R = rng_.uniformInt(0, N-1);

    //std::vector<Motion*> data;
    //if(R < tStart_->size()) 
    //{
    //  //sample from start tree
    //  tStart_->list(data);
    //  si_->copyState(state, data.at(R)->state);
    //} else {
    //  //sample from goal tree
    //  tGoal_->list(data);
    //  si_->copyState(state, data.at(R - tStart_->size())->state);
    //}

    //Tree restriction sampling (vertex version). RRT style
    const size_t N = tree_->size();
    const size_t R = rng_.uniformInt(0, N-1);
    si_->getStateSpace()->copyState(state, tree_->getNodes().at(R)->getState());

    //if(random_config->parent == nullptr) {
    //  si_->copyState(state, random_config->state);
    //  return;
    //}

    //const double t = rng_.uniform01();
    //const auto random_state = random_config->state;
    //const auto parent_state = random_config->parent->state;
    //si_->getStateSpace()->interpolate(parent_state, random_state, t, state);
    //// OMPL_WARN("Sample state");
    //// si_->printState(state);

    ////Randomly perturbate state
    if(sampling_perturbation_bias_ > 0.0) {
      internal_space_sampler_->sampleUniformNear(state, state, sampling_perturbation_bias_);
    }

}
