
#include "ompl/multilevel/datastructures/helpers/SamplingHelper.h"

namespace ompl
{
namespace multilevel
{

void sampleFromPath(const ompl::base::SpaceInformationPtr& si, RNG& rng, const std::vector<base::State *>& path_states, ompl::base::State* state) 
{
  std::vector<double> distances;
  for (unsigned int i = 1; i < path_states.size(); i++)
  {
      const double d = si->distance(path_states.at(i - 1), path_states.at(i));
      distances.push_back(d);
  }
  const double path_length = std::accumulate(distances.begin(), distances.end(), 0.0f);
  if(path_length < 1e-3) {
    OMPL_WARN("Path sampler uses path of length %f", path_length);
    si->copyState(state, path_states.front());
    return;
  }
  const double random_position_on_path = rng.uniformReal(0, path_length);

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
          si->getStateSpace()->interpolate(s1, s2, s, state);
        }else {
          si->copyState(state, s1);
        }
        return;
      }
  }
  OMPL_ERROR("Path sampler reached end of method with length %f and random value %f", path_length, random_position_on_path);
}

}
}
