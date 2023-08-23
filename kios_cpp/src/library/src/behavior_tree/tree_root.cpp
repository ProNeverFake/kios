#include "behavior_tree/tree_root.hpp"

namespace Insertion
{
    TreeRoot::TreeRoot(std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
    {
        tree_state_ptr_ = tree_state_ptr;
        task_state_ptr_ = task_state_ptr;
        initialize_tree();
    }

    void TreeRoot::initialize_tree()
    {
        // * register nodes
        // static GripperInterface grip_singleton;
        // factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
        // factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &grip_singleton));
        factory_.registerNodeType<Approach>("Approach", tree_state_ptr_, task_state_ptr_);
        factory_.registerNodeType<HasObject>("HasObject", tree_state_ptr_, task_state_ptr_);
        factory_.registerNodeType<AtPosition>("AtPosition", tree_state_ptr_, task_state_ptr_);
        // factory_.registerNodeType<Contact>("Contact", tree_state_ptr_, task_state_ptr_);
        // factory_.registerNodeType<Wiggle>("Wiggle", tree_state_ptr_, task_state_ptr_);
        // factory.registerNodeType<Reach>("Reach");
        // * generate tree
        tree_ = factory_.createTreeFromText(test_tree);
    }
    std::shared_ptr<kios::TreeState> TreeRoot::get_tree_state_ptr()
    {
        return tree_state_ptr_;
    }
    std::shared_ptr<kios::TaskState> TreeRoot::get_task_state_ptr()
    {
        return task_state_ptr_;
    }

    /**
     * @brief only tick once, return running immediately if a node is running
     *
     * @return BT::NodeStatus
     */
    BT::NodeStatus TreeRoot::tick_once()
    {
        return tree_.tickOnce();
    }

    /**
     * @brief tick the tree, block until it return success
     *
     * @return BT::NodeStatus
     */
    BT::NodeStatus TreeRoot::tick_while_running()
    {
        return tree_.tickWhileRunning();
    }

    bool TreeRoot::is_switch_action()
    {
        if (current_action_name_ != get_tree_state_ptr()->action_name)
        {
            current_action_name_ = get_tree_state_ptr()->action_name;
            return true;
        }
        else
        {
            return false;
        }
    }

    void TreeRoot::update_state()
    {
        // ! TODO
    }

} // namespace Insertion

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////