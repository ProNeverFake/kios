#include "behavior_tree/condition_node/condition_node.hpp"

namespace Insertion
{
    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    //* HAS OBJECT

    HasObject::HasObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : HyperMetaNode<BT::ConditionNode>(name, config, tree_state_ptr, task_state_ptr)
    {
    }
    BT::NodeStatus HasObject::tick()
    {
        if (has_object())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    bool HasObject::has_object()
    {
        // ! TODO
        return true;
    }

    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    //* AT POSITION
    AtPosition::AtPosition(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : HyperMetaNode<BT::ConditionNode>(name, config, tree_state_ptr, task_state_ptr)
    {
    }

    BT::NodeStatus AtPosition::tick()
    {
        if (at_position())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    bool AtPosition::at_position()
    {
        // ! TODO
        return false;
    }
} // namespace Insertion