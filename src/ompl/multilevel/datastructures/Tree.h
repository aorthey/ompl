/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_TREE_
#define OMPL_MULTILEVEL_DATASTRUCTURES_TREE_

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/multilevel/datastructures/TreeNode.h"

namespace ompl
{
    namespace multilevel {

        OMPL_CLASS_FORWARD(Tree);

        class Tree {
          public:
            Tree(const base::SpaceInformationPtr& si);
            ~Tree();

            base::SpaceInformationPtr getSpaceInformation() const;

            void clear();
            size_t size() const;
            std::vector<TreeNode*> getNodes() const;
            TreeNode* addNodeAndParent(const ompl::base::State* state, TreeNode* parent);
            TreeNode* nearest(TreeNode* parent) const;
            TreeNode* getRoot() const;

            TreeNode* At(const int& n) const;
            TreeNode* operator[](const int& n) const;

          private:
            base::SpaceInformationPtr si_;

            TreeNode* root_;

            std::shared_ptr<NearestNeighbors<TreeNode *>> nn_;

            std::vector<TreeNode*> nodes_;
        };
    }
}
#endif

