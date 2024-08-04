/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_DATASTRUCTURES_TREENODE_
#define OMPL_MULTILEVEL_DATASTRUCTURES_TREENODE_

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

namespace ompl
{
    namespace multilevel {

        class TreeNode {

          public:
            TreeNode() = delete;
            TreeNode(const base::SpaceInformationPtr &si);
            TreeNode(const base::SpaceInformationPtr &si, const ompl::base::State* state);

            ~TreeNode();

            ompl::base::State* getState() const;
            void setState(ompl::base::State* state);
            void setParent(TreeNode* parent);
            TreeNode* getParent() const;

            void addChild(TreeNode* node);
            std::vector<TreeNode*> getChildren() const;

          private:

            ompl::base::State* state_;
            size_t id_{0};

            TreeNode* parent_;
            std::vector<TreeNode*> children_;
        };

        double distance(const base::SpaceInformationPtr& si, const TreeNode *a, const TreeNode *b);
    }
}
#endif


