#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_HELPERS_SAMPLINGHELPER_
#define OMPL_MULTILEVEL_DATASTRUCTURES_HELPERS_SAMPLINGHELPER_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Exception.h>
#include <ompl/multilevel/datastructures/helpers/Parameter.h>
#include <ompl/multilevel/datastructures/Tree.h>

namespace ompl
{
  namespace multilevel
  {
      void sampleFromPath(const ompl::base::SpaceInformationPtr& si, RNG& rng, const std::vector<base::State *>& path_states, ompl::base::State* state);

      struct DatastructureInformation 
      {
          ompl::base::SpaceInformationPtr si;
          ompl::base::ProblemDefinitionPtr pdef;
          ompl::base::StateSamplerPtr sampler;
          double path_restriction_surrounding_sampling_bias;
          double path_restriction_sampling_bias;
          double sampling_perturbation_bias;
          size_t number_of_nodes;
      };

      template<class DataStructure>
      void sampleFromDatastructure(const DatastructureInformation& info, const DataStructure& datastructure, RNG& rng, ompl::base::State* state)
      {

          //Path restriction sampling
          if(info.path_restriction_sampling_bias > 0.0) {
            if(rng.uniform01() < info.path_restriction_sampling_bias) {
              if(!info.pdef->hasSolution()) 
              {
                OMPL_WARN("Cannot sample from space without a solution.");
                return;
              }
              const auto path = info.pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
              const std::vector<base::State *>& path_states = path->getStates();
              sampleFromPath(info.si, rng, path_states, state);
              if(info.path_restriction_surrounding_sampling_bias > 0.0) {
                info.sampler->sampleUniformNear(state, state, info.path_restriction_surrounding_sampling_bias);
              }
              return;
            }
          }

          //Tree restriction sampling (vertex version). RRT style
          //const size_t N = datastructure.size();
          const size_t N = info.number_of_nodes;
          const size_t R = rng.uniformInt(0, N-1);

          if constexpr (std::is_same_v<DataStructure, ompl::multilevel::Tree>) {
              //info.si->getStateSpace()->copyState(state, datastructure[R]->getState());
              info.si->getStateSpace()->copyState(state, datastructure[R]->getState());
          }
          else {
              auto* config = datastructure[R];           // bundled property access
              info.si->getStateSpace()->copyState(state, config->state);
          }

          ////Randomly perturbate state
          if(info.sampling_perturbation_bias > 0.0) {
            info.sampler->sampleUniformNear(state, state, info.sampling_perturbation_bias);
          }

      }

  }
}
#endif

