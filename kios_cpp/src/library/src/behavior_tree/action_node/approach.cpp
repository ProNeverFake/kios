#include "behavior_tree/action_node/approach.hpp"

namespace Insertion
{
    Approach::Approach(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : MetaNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    /**
     * @brief Here to apply the success condition check.
     *
     * @return true
     * @return false
     */
    bool Approach::is_success()
    {
        if (get_task_state_ptr()->isActionSuccess)
        {
            get_task_state_ptr()->isActionSuccess = false; //* success flag consumed
            return true;
        }
        else
        {
            return false;
        }
    }

    void Approach::update_tree_state()
    {
        auto tree_state_ptr = get_tree_state_ptr();
        auto node_context = get_node_context_ref();
        tree_state_ptr->action_name = node_context.action_name;
        tree_state_ptr->action_phase = node_context.action_phase;
    }

    void Approach::node_context_initialize()
    {
        auto node_context = get_node_context_ref();
        node_context.node_name = "approach";
        node_context.action_phase = kios::ActionPhase::APPROACH;
        node_context.parameter["skill"]["action_name"] = "approach";
        node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::APPROACH;
    }

    BT::NodeStatus Approach::onStart()
    {
        if (is_success())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus Approach::onRunning()
    {
        if (is_success())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void Approach::onHalted()
    {
        // * interrupted behavior. do nothing.
        std::cout << "Action stoped" << std::endl;
    }

} // namespace Insertion
