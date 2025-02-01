#pragma once

#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <random>
#include <unordered_map>
#include <set>
#include <functional>

template<typename T>
class BoundedUniquePermutations {
public:
    BoundedUniquePermutations(const std::unordered_map<std::string, T>& unordered_map, const size_t& max_iterations) :
      max_iterations_(max_iterations) {
        for (const auto& pair : unordered_map) {
            keys.push_back(pair.first);
        }
    }

    class BoundedUniquePermutationsIterator {

        BoundedUniquePermutationsIterator(BoundedUniquePermutations<T>* parent, bool end = false) : parent_(parent), is_end(end) {
            if (end) {
              return;
            }
            current_permutation = parent_->keys;
            parent_->unique_permutations.insert(current_permutation);
        }

        friend class BoundedUniquePermutations;

      public:
        const std::vector<std::string>& operator*() const { return current_permutation; }
        const std::vector<std::string>* operator->() const { return &current_permutation; }

        BoundedUniquePermutationsIterator& operator++() {
            if(num_of_iterations > parent_->max_iterations_ - 2) {
              is_end = true;
              return *this;
            }
            static std::random_device rd;
            static std::mt19937 g(0);

            size_t counter = 0;
            while (!parent_->isUniquePermutation(current_permutation)) {
              std::shuffle(current_permutation.begin(), current_permutation.end(), g);
              counter++;
              if(counter > parent_->max_tries_) {
                is_end = true;
                return *this;
              }
            }
            parent_->unique_permutations.insert(current_permutation);
            num_of_iterations++;
            return *this;
        }

        bool operator!=(const BoundedUniquePermutationsIterator& other) const { return is_end != other.is_end; }

      private:
        BoundedUniquePermutations<T>* parent_;
        std::vector<std::string> current_permutation;
        size_t num_of_iterations = 0;
        bool is_end;
    };

    BoundedUniquePermutationsIterator begin() { 
      return BoundedUniquePermutationsIterator(this); 
    }
    BoundedUniquePermutationsIterator end() { 
      return BoundedUniquePermutationsIterator(this, true); 
    }

private:
    std::vector<std::string> keys;
    std::set<std::vector<std::string>> unique_permutations;
    size_t max_iterations_;
    size_t max_tries_{100};

    // Helper function to check if a permutation is unique
    bool isUniquePermutation(const std::vector<std::string>& perm) const {
        return unique_permutations.find(perm) == unique_permutations.end();
    }

};

template<typename T>
BoundedUniquePermutations<T> getBoundedUniquePermutations(const std::unordered_map<std::string, T>& unordered_map, size_t max_iterations) {
    return BoundedUniquePermutations<T>(unordered_map, max_iterations);
}

