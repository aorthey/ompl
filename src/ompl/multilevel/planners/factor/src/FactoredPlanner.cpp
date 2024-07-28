#include "ompl/multilevel/planners/factor/FactoredPlanner.h"
#include "ompl/multilevel/planners/factor/RestrictionSampler.h"
#include "ompl/multilevel/datastructures/FactoredSpaceInformation.h"
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionSideStep.h>

#include <string>

using namespace ompl::multilevel;

FactoredPlanner::FactoredPlanner(const FactoredSpaceInformationPtr& si, const std::vector<FactoredPlannerPtr>& children_planner) 
  : BaseTypePlanner(si, false), children_planner_(children_planner)
{
  setName("PlannerOn" + si->getName());

  internal_space_sampler_ = si_->allocStateSampler();

  if(!children_planner.empty()) 
  {
    sampler_ = std::make_shared<RestrictionSampler>(si, children_planner);
  }
  setIntermediateStates(false);
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

  if (children.size() != 1)
  {
    return failure("Factor has more than 1 child. NYI.");
  }
  if (children_planner_.size() != 1)
  {
    return failure("Children planners has more than 1 child. NYI.");
  }

  auto child = children.front();

  auto projection = child->getProjection();
  if (projection == nullptr)
  {
    return failure("Child has no projection.");
  }

  if(!projection->isFibered())
  {
    return failure("Projection is not fibered.");
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

  auto find_section = std::make_shared<FindSectionSideStep>(path_restriction);

  auto qStart = MakeStartState();
  auto qGoal = MakeGoalState();

  ompl::time::point tStart = ompl::time::now();
  auto maybe_section = find_section->solve(qStart, qGoal);
  ompl::time::point tEnd = ompl::time::now();

  if(!maybe_section.has_value()) {
    return failure("Timeout after " + std::to_string(ompl::time::seconds(tEnd - tStart)) + "s");
  }

  auto section = maybe_section.value();
  OMPL_WARN("Found section with %d states.", section->size());
  for(const auto& state : section->getStates()) {
    factor->printState(state);
  }
  return success(maybe_section.value());
}

ompl::base::PlannerStatus FactoredPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc) 
{
  if(firstRun_) {
    firstRun_ = false;
    auto maybe_section = solveSection();
    if(!maybe_section.has_value()) {
      OMPL_WARN("No valid section found. Reason: %s", maybe_section.error().c_str());
    }
  }
  return BaseTypePlanner::solve(ptc);
}

void FactoredPlanner::clear() {
  firstRun_ = false;
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
  ompl::RNG::setSeed(seed);
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
  return nodes_.size();
}

void FactoredPlanner::sampleFromDatastructure(ompl::base::State* state) 
{

    //Path restriction sampling
    if(path_restriction_sampling_bias_ > 0.0) {
      if(path_restriction_sampling_bias_ >= 1.0 || rng_.uniform01() < path_restriction_sampling_bias_) {
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
    const size_t N = nodes_.size();
    const size_t R = rng_.uniformInt(0, N-1);
    si_->getStateSpace()->copyState(state, nodes_.at(R)->state);

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
