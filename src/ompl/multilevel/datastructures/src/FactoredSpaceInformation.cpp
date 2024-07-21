#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>
#include <ompl/util/RandomNumbers.h>

using namespace ompl::multilevel;

ompl::multilevel::FactoredSpaceInformation::FactoredSpaceInformation(const ompl::base::StateSpacePtr& space) : 
  ompl::base::SpaceInformation(space) 
{
  OMPL_INFORM("Create factor for space %s (dimensionality %d)", space->getName().c_str(), space->getDimension());
}

std::string FactoredSpaceInformation::getName() const {
  return getStateSpace()->getName();
}

const FactoredSpaceInformationPtr& FactoredSpaceInformation::getParent() const {
  return parent_;
}

bool FactoredSpaceInformation::hasParent() const {
  return parent_ != nullptr;
}

const std::vector<FactoredSpaceInformationPtr>& FactoredSpaceInformation::getChildren() const {
  return children_;
}

bool FactoredSpaceInformation::hasChildren() const {
  return !children_.empty();
}

const ProjectionPtr& FactoredSpaceInformation::getProjection() const {
  return projection_to_parent_;
}

void FactoredSpaceInformation::setParent(FactoredSpaceInformationPtr factor_si) {
  parent_ = factor_si;
}

std::vector<FactoredSpaceInformationPtr> FactoredSpaceInformation::getAllFactors() {
  std::vector<FactoredSpaceInformationPtr> factors_flatten;

  factors_flatten.push_back(shared_from_this());

  if(hasChildren()) {
    for(const auto& child : children_ ) {
      const auto next = child->getAllFactors();
      factors_flatten.insert(factors_flatten.end(), next.begin(), next.end());
    }
  }
  return factors_flatten;
}

std::vector<FactoredSpaceInformationPtr> FactoredSpaceInformation::getLeafFactors() {
  std::vector<FactoredSpaceInformationPtr> leaf_factors;
  for(const auto& factor : getAllFactors()) {
    if(!factor->hasChildren()) {
      leaf_factors.push_back(factor);
    }
  }
  return leaf_factors;
}

bool FactoredSpaceInformation::isEquivalentTo(const FactoredSpaceInformationPtr& rhs) const 
{
    return getName() == rhs->getName();
}

bool FactoredSpaceInformation::projectionHasValidIndices(const FactoredSpaceInformationPtr& factor, const ProjectionPtr& projection) const 
{
  if(!projection->isFibered()) {
    return true;
  }
  const auto& fibered_projection = std::static_pointer_cast<FiberedProjection>(projection);
  const std::vector<size_t> indices = fibered_projection->getInclusionIndices();
  const auto N = projection->getDimension();
  for(const auto& index : indices) 
  {
    if(index >= N) {
      OMPL_ERROR("Index %d / %d is out of bounds for factor space %s.", index, N, factor->getName().c_str());
      return false;
    }
  }
  return true;
}

bool FactoredSpaceInformation::projectionOverlapsWithExistingProjections(const FactoredSpaceInformationPtr& factor, const ProjectionPtr& projection) const 
{
  if(!projection->isFibered()) {
    return false;
  }
  const auto& fibered_projection = std::static_pointer_cast<FiberedProjection>(projection);
  std::vector<size_t> indices = fibered_projection->getInclusionIndices();
  std::sort(indices.begin(), indices.end());

  auto hasIndexIntersection = [factor, indices](const auto& child) {
        const auto& child_projection = child->getProjection();
        if(!child_projection->isFibered()) 
        {
          return false;
        }
        const auto& fibered_child_projection = std::static_pointer_cast<FiberedProjection>(child_projection);
        std::vector<size_t> child_indices = fibered_child_projection->getInclusionIndices();
        std::sort(child_indices.begin(), child_indices.end());

        std::vector<size_t> intersected_indices;
        std::set_intersection(indices.begin(), indices.end(),
                              child_indices.begin(), child_indices.end(),
                              back_inserter(intersected_indices));

        if(!intersected_indices.empty()) 
        {
          std::string error_msg = "Found overlap from factor " + factor->getName() 
            + " to factor " + child->getName() + " at: \n";
          for(const auto& index : intersected_indices) 
          {
            error_msg += " - Index " + std::to_string(index) + "\n";
          }
          error_msg += " Note: Indices from factor " + factor->getName() + " are ";
          for(const auto& index : indices) 
          {
            error_msg += std::to_string(index) + " ";
          }
          error_msg += "\n Note: Indices from child " + child->getName() + " are ";
          for(const auto& index : child_indices) 
          {
            error_msg += std::to_string(index) + " ";
          }

          OMPL_ERROR("%s", error_msg.c_str());
          return true;
        }
        return false;//!intersected_indices.empty();
      };

  return std::any_of(children_.begin(), children_.end(), hasIndexIntersection);
}

bool FactoredSpaceInformation::childExists(const FactoredSpaceInformationPtr& factor) const {
  return std::any_of(children_.begin(), children_.end(),
      [factor](const auto& child) {
        return child->isEquivalentTo(factor);
      });
}

bool FactoredSpaceInformation::hasChild(const std::string& name) const {
  return std::any_of(children_.begin(), children_.end(),
      [&name](const auto& child) {
        return child->getName() == name;
      });
}

bool FactoredSpaceInformation::projectionHasCorrectImage(const FactoredSpaceInformationPtr& child, const ProjectionPtr& projection) const
{
    if(projection->getBaseDimension() != child->getStateDimension())
    {
      return false;
    }

    return projection->getBase()->getName() == child->getName();
}

bool FactoredSpaceInformation::projectionHasCorrectPreimage(const ProjectionPtr& projection) const
{
    if(projection->getDimension() != this->getStateDimension())
    {
      return false;
    }

    return projection->getBundle()->getName() == this->getName();
}

bool FactoredSpaceInformation::addChild(FactoredSpaceInformationPtr child, ProjectionPtr projection, bool compute_fiber_space) {

  if(this->isEquivalentTo(child)) 
  {
    OMPL_ERROR("Cannot add the same factor as child for factor %s.", getName().c_str());
    return false;
  }

  if(childExists(child)) 
  {
      OMPL_ERROR("Child with name %s already exists. Please choose unique names for each StateSpace.", child->getName().c_str());
      return false;
  }

  if(!projectionHasCorrectPreimage(projection))
  {
      OMPL_ERROR("Projection for child %s does not have correct preimage.", child->getName().c_str());
      return false;
  }

  if(!projectionHasCorrectImage(child, projection))
  {
      OMPL_ERROR("Projection for child %s does not have correct image.", child->getName().c_str());
      return false;
  }

  if(!projectionHasValidIndices(child, projection)) 
  {
      OMPL_ERROR("Projection for child %s has invalid indices.", child->getName().c_str());
      return false;
  }

  if(projectionOverlapsWithExistingProjections(child, projection)) 
  {
      OMPL_ERROR("Projection for child %s overlaps with existing projection.", child->getName().c_str());
      return false;
  }

  child->setProjectionToParent(projection);
  child->setParent(shared_from_this());
  children_.push_back(child);

  if(compute_fiber_space && projection->isFibered()) {
    OMPL_INFORM("Create fiber space for projection from %s to %s", getName().c_str(), child->getName().c_str());
    std::static_pointer_cast<FiberedProjection>(projection)->makeFiberSpace();
  }
  return true;
}

void FactoredSpaceInformation::setProjectionToParent(ProjectionPtr projection) {
  projection_to_parent_ = projection;
}

void FactoredSpaceInformation::setup() {
  SpaceInformation::setup();
}

const FactoredSpaceInformationPtr& FactoredSpaceInformation::getChild(const std::string& name) const {
  auto iterator = std::find_if(children_.begin(), children_.end(), 
        [name](const auto& child) {
          return child->getName() == name;
        });
  if(iterator == children_.end()) {
    OMPL_ERROR("No child with name %s", name.c_str());
    throw "NoChildError";
  }
  return *iterator;
}

void FactoredSpaceInformation::lift(const std::unordered_map<std::string, base::State*>& childStates_, base::State* state) const {
  if(childStates_.size() <= 1) {
    const auto& name = childStates_.begin()->first;
    const auto& childState = childStates_.begin()->second;
    const auto& child = getChild(name);
    const auto& projection = child->getProjection();
    projection->lift(childState, state);
    return;
  }

  for(const auto& name_and_state: childStates_) {
    const auto& name = name_and_state.first;
    const auto& childState = name_and_state.second;
    const auto& child = getChild(name);
    const auto& projection = std::static_pointer_cast<FiberedProjection>(child->getProjection());
    projection->inclusionMap(childState, state);
  }
}

/** \brief project: Map a state to its children factors store the result in childStates */
void FactoredSpaceInformation::project(const base::State* state, const std::unordered_map<std::string, base::State*>& childStates) const {
  if(!hasChildren()) {
    return;
  }
  if(children_.size() == 1) {
    if(childStates.size() != 1) {
      OMPL_ERROR("Number of child states is %d, which is different from children (%d).", childStates.size(), children_.size());
      throw "InvalidStates";
    }
    const auto& child = children_.front();
    const auto& projection = child->getProjection();

    const auto& name = childStates.begin()->first;
    if(name != child->getName()) {
      OMPL_ERROR("Name of child state is %s, which is different from child (%s).", name.c_str(), child->getName().c_str());
      throw "InvalidChildName";
    }
    const auto& childState = childStates.begin()->second;
    projection->project(state, childState);
    return;
  }

  for(const auto& name_and_state: childStates) {
    const auto& name = name_and_state.first;
    const auto& childState = name_and_state.second;
    const auto& child = getChild(name);
    const auto& projection = child->getProjection();
    projection->project(state, childState);
  }
}

std::unordered_map<std::string, ompl::base::State*> FactoredSpaceInformation::allocChildStates() const {
  std::unordered_map<std::string, ompl::base::State*> childStates;
  for(const auto& child : children_) {
    childStates.insert({child->getName(), child->allocState()});
  }
  return childStates;
}

void FactoredSpaceInformation::freeChildStates(std::unordered_map<std::string, ompl::base::State*>& childStates) const {
  for(const auto& name_and_state: childStates) {
    const auto& name = name_and_state.first;
    const auto& childState = name_and_state.second;
    const auto& child = getChild(name);
    child->freeState(childState);
  }
  childStates.clear();
}

size_t FactoredSpaceInformation::getTotalNumParents() const {
  size_t count = 0;

  if(!hasParent()) {
    return count;
  }

  count++;
  auto parent = getParent();

  while(parent->hasParent()) {
    count++;
    parent = parent->getParent();
  }
  return count;
}

void FactoredSpaceInformation::printFactorization(std::ostream &out) const
{
  out << getName() << " (" << stateTypeToString(getStateSpace()) << ", dimensionality " << getStateDimension() << ")" << std::endl;
  if (!hasChildren()) {
    return;
  }

  const auto children = getChildren();

  const std::string prefix = "└────";
  const std::string whitespaces = std::string(5, ' ');

  for(const auto& child : children) {
    for(size_t k = 0; k < child->getTotalNumParents() - 1; k++) {
      out << whitespaces;
    }
    out << prefix;
    child->printFactorization(out);
  }
}

void FactoredSpaceInformation::printSettings(std::ostream &out) const
{
    //SpaceInformation::printSettings(out);
    out << "Factorization of " << getName() << " has ";
    if(hasChildren()) {
      const auto& children = getChildren();
      const auto N = children.size();
      out << N << (N > 1 ? " children" : " child") << " (";
      for (auto iter = children.begin(); iter != children.end(); iter++) {
        if (iter != children.begin()) out << " | ";
        out << (*iter)->getName();
      }
      out << ")";
    } else {
      out << "no children";
    }

    out << " and ";

    if(hasParent()) {
      out << "1 parent (" << getParent()->getName() << ")";
    } else {
      out << "no parents";
    }
    out << "." << std::endl;
}

/** \brief lift: Map states from all leaf factors to this factor space and store the result in state */
void FactoredSpaceInformation::liftLeafStates(const std::unordered_map<std::string, ompl::base::State*>& leaf_node_states, ompl::base::State* state) {
  typedef std::pair<std::string, ompl::base::State*> NodeState;
  //////////////////////////////////////////////////////////////////////////////////
  //(1) Check that all leaf nodes are covered
  //////////////////////////////////////////////////////////////////////////////////
  auto leaf_factors = getLeafFactors();
  for(const auto& leaf : leaf_factors) {
    const auto& name = leaf->getName();
    auto it = leaf_node_states.find(name);
    if(it == leaf_node_states.end()) {
      OMPL_ERROR("Could not find leaf node %s in states. Please specify all leaf node states.", name.c_str());
      throw "LeafNotFoundInState";
    }
  }

  for(const auto& leaf_node : leaf_node_states) {
    const auto& name = leaf_node.first;
    auto it = std::find_if(leaf_factors.begin(), leaf_factors.end(), 
        [&name](const FactoredSpaceInformationPtr& factor) {
          return factor->getName() == name;
        });
    if(it == leaf_factors.end()) {
      OMPL_ERROR("Could not find leaf node %s in factors.", name.c_str());
      throw "LeafNotFoundInState";
    }
  }
  //////////////////////////////////////////////////////////////////////////////////
  //(2) Put all leaf node states into a queue, and try to project them upwards,
  //thereby merging them
  //////////////////////////////////////////////////////////////////////////////////
  const auto all_factors = getAllFactors();

  std::vector<NodeState> node_states;
  for(const auto& leaf_node_state : leaf_node_states) {
    OMPL_WARN("Add leaf node %s", leaf_node_state.first.c_str());
    node_states.push_back(std::make_pair(leaf_node_state.first, leaf_node_state.second));
  }

  ompl::RNG rng(0);
  while(true) {
    int next_node_state_index = rng.uniformInt(0, node_states.size()-1);
    auto node_state = node_states.at(next_node_state_index);
    auto name = node_state.first;

    auto current_state = node_state.second;

    //////////////////////////////////////////////////////////////////////////////////
    //Get factor to node
    //////////////////////////////////////////////////////////////////////////////////
    auto iterator_factor = std::find_if(all_factors.begin(), all_factors.end(), 
        [&name](const FactoredSpaceInformationPtr& factor) {
          return factor->getName() == name;
        });
    if(iterator_factor == all_factors.end()) {
      OMPL_ERROR("Could not find leaf node %s in factors.", name.c_str());
      throw "LeafNotFoundInState";
    }
    const auto& current_factor = *iterator_factor;

    //////////////////////////////////////////////////////////////////////////////////
    //Check if we are the root node. In that case, just copy the state and
    //return. Otherwise create a new state and continue.
    //////////////////////////////////////////////////////////////////////////////////
    auto parent = current_factor->getParent();
    if(parent == nullptr) {
      //Ensure that root node and current name match up
      if(name != current_factor->getName()) {
        OMPL_ERROR("Factor %s has no parent.", name.c_str());
        throw "NoParent";
      }
      current_factor->copyState(state, current_state);
      current_factor->freeState(current_state);
      return;
    }

    auto next_state = parent->allocState();
    auto child_states = parent->allocChildStates();
    //////////////////////////////////////////////////////////////////////////////////
    //Extract all states which belong to the children of the parent node
    //////////////////////////////////////////////////////////////////////////////////
    bool liftable = true;
    for(const auto& child : parent->getChildren()) {
      auto name = child->getName();

      auto it = std::find_if(node_states.begin(), node_states.end(), [&name](const NodeState& node_state) {
            return node_state.first == name;
          }
      );
      if(it == node_states.end()) {
        // OMPL_INFORM("Unliftable because of space %s. Node states contain %d states.", name.c_str(), node_states.size());
        // for(const auto& node_state : node_states) {
        //   std::cout << node_state.first << std::endl;
        // }
        liftable = false;
        break;
      }
      auto cit = child_states.find(name);
      if(cit == child_states.end()) {
        OMPL_ERROR("Could not find child node %s in states.", name.c_str());
        throw "ChildNotNotFoundInState";
      }
      child->copyState(cit->second, it->second);
    }
    if(!liftable) {
      continue;
    }
    parent->lift(child_states, next_state);

    //////////////////////////////////////////////////////////////////////////////////
    //Remove lifted states from vector, and add new one to node_states
    //////////////////////////////////////////////////////////////////////////////////
    for(const auto& child : parent->getChildren()) {
      auto name = child->getName();
      auto new_end = std::remove_if(node_states.begin(), node_states.end(),
                              [&name](const NodeState& node_state)
                              { 
                                return node_state.first == name;
                              });
      node_states.erase(new_end, node_states.end());
    }
    node_states.push_back(std::make_pair(parent->getName(), next_state));
    parent->freeChildStates(child_states);
  }
}

void FactoredSpaceInformation::interpolate(const base::State *from, const base::State *to, double t, base::State *state) const {
}
