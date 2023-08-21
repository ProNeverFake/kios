#include "behavior_tree/action_node/contact.hpp"

namespace Insertion
{
    Contact::Contact(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
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
    bool Contact::is_success()
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

    void Contact::update_tree_state()
    {
        auto tree_state_ptr = get_tree_state_ptr();
        auto node_context = get_node_context_ref();
        tree_state_ptr->action_name = node_context.action_name;
        tree_state_ptr->action_phase = node_context.action_phase;
    }

    void Contact::node_context_initialize()
    {
        auto node_context = get_node_context_ref();
        node_context.node_name = "contact";
        node_context.action_phase = kios::ActionPhase::CONTACT;
        node_context.parameter["skill"]["action_name"] = "contact";
        node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::CONTACT;
    }

    BT::NodeStatus Contact::onStart()
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
    BT::NodeStatus Contact::onRunning()
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

    void Contact::onHalted()
    {
        // * interrupted behavior. do nothing.
        std::cout << "Action stoped" << std::endl;
    }

} // namespace Insertion
