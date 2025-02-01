#define BOOST_TEST_MODULE "FactoredPathSectionPlanning"
#include <boost/test/unit_test.hpp>
#include <boost/mpl/vector.hpp>
#include <ompl/multilevel/datastructures/helpers/BoundedUniquePermutations.h>

struct SingleObject {
  auto operator()() { 
    std::unordered_map<std::string, int> single_object{{"one", 1}};
    return single_object;
  };
};

struct TwoObject {
  auto operator()() { 
    std::unordered_map<std::string, int> single_object{{"one", 1}, {"two", 2}};
    return single_object;
  };
};

struct ThreeObject {
  auto operator()() { 
    std::unordered_map<std::string, int> single_object{{"one", 1}, {"two", 2}, {"three", 3}};
    return single_object;
  };
};

struct FourObject {
  auto operator()() { 
    std::unordered_map<std::string, int> single_object{{"one", 1}, {"two", 2}, {"three", 3}, {"four", 4}};
    return single_object;
  };
};

struct FiveObject {
  auto operator()() { 
    std::unordered_map<std::string, int> single_object{{"one", 1}, {"two", 2}, {"three", 3}, {"four", 4}, {"five", 5}};
    return single_object;
  };
};

struct LargeObject {
  auto operator()() { 
    std::unordered_map<std::string, int> single_object;
    for(size_t k = 0; k < 100; k++) {
      single_object.insert({std::to_string(k), k});
    }
    return single_object;
  };
};

typedef boost::mpl::vector<
  SingleObject,
  TwoObject,
  ThreeObject,
  FourObject,
  FiveObject,
  LargeObject
> ObjectsToBePermutated;

const size_t kMaxPermutations = 100;
BOOST_AUTO_TEST_CASE_TEMPLATE(BoundedUniquePermutations_CheckOrdering, T, ObjectsToBePermutated) {
  T templated_object;
  auto object = templated_object();

  std::set<std::vector<std::string>> unique_permutations;
  for(const auto& permutation : getBoundedUniquePermutations(object, kMaxPermutations)) {
    BOOST_CHECK_EQUAL(permutation.size(), object.size());
    for(const auto& name : permutation) {
      BOOST_CHECK(object.find(name) != object.end());
    }
    BOOST_CHECK(unique_permutations.find(permutation) == unique_permutations.end());
    unique_permutations.insert(permutation);
  }
  BOOST_CHECK_CLOSE(unique_permutations.size(), std::min(std::tgamma(object.size() + 1), double(kMaxPermutations)), 1e-6);
}
