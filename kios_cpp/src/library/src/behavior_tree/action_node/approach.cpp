#include "behavior_tree/action_node/approach.hpp"

namespace Insertion
{
    Approach::Approach(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : HyperMetaNode<BT::StatefulActionNode>(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void Approach::update_tree_state()
    {
        std::cout << "APPROACH UPDATE TREE STATE" << std::endl;
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;
        std::cout << "UPDATED VALUE: " << get_tree_state_ptr()->action_name << std::endl;
    }

    void Approach::node_context_initialize()
    {
        std::cout << "node_context_initialize" << std::endl;
        auto &node_context = get_node_context_ref();
        node_context.node_name = "APPROACH";
        node_context.action_name = "approach";
        node_context.action_phase = kios::ActionPhase::APPROACH;
        node_context.parameter["skill"]["action_name"] = "approach";
        node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::APPROACH;
        std::cout << "NEW: " << node_context.node_name << std::endl;
    }

    bool Approach::is_success()
    {
        return consume_mios_success();
    }

    BT::NodeStatus Approach::onStart()
    {
        if (has_succeeded_once())
        {
            return BT::NodeStatus::SUCCESS;
        }
        if (is_success())
        {
            mark_success();
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
            mark_success();
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
