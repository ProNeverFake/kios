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
    }

    void Approach::node_context_initialize()
    {
        std::cout << "APPROACH INITIALIZE" << std::endl;
        auto &node_context = get_node_context_ref();
        node_context.node_name = "APPROACH";
        node_context.action_name = "approach";
        node_context.action_phase = kios::ActionPhase::APPROACH;
        node_context.parameter["skill"]["action_name"] = "approach";
        node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::APPROACH;
    }

    bool Approach::is_success()
    {
        std::cout << "APPROACH IS_SUCCESS?" << std::endl;

        return consume_mios_success();
    }

    BT::NodeStatus Approach::onStart()
    {
        std::cout << "APPROACH ON START" << std::endl;
        if (has_succeeded_once())
        {
            std::cout << "APPROACH HAS ONCE SUCCEEDED" << std::endl;
            return BT::NodeStatus::SKIPPED;
        }
        if (is_success())
        {
            std::cout << "APPROACH ALREADY SUCCESS" << std::endl;

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "APPROACH GO RUNNING" << std::endl;

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus Approach::onRunning()
    {
        std::cout << "APPROACH ON RUNNING" << std::endl;
        if (has_succeeded_once())
        {
            std::cout << "APPROACH HAS ONCE SUCCEEDED == true" << std::endl;
            return BT::NodeStatus::SKIPPED;
        }
        if (is_success())
        {
            std::cout << "APPROACH SUCCESS" << std::endl;

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "APPROACH RUNNING" << std::endl;
            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void Approach::onHalted()
    {
        // * interrupted behavior. do nothing.
        std::cout << "APPROACH ON HALTED" << std::endl;
    }

    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    // Approach::Approach(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
    //     : HyperMetaNode<BT::SyncActionNode>(name, config, tree_state_ptr, task_state_ptr)
    // {
    //     // initialize local context
    //     node_context_initialize();
    // }

    // void Approach::update_tree_state()
    // {
    //     std::cout << "APPROACH UPDATE TREE STATE" << std::endl;
    //     get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
    //     get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;
    //     std::cout << "UPDATED VALUE: " << get_tree_state_ptr()->action_name << std::endl;
    // }

    // void Approach::node_context_initialize()
    // {
    //     std::cout << "node_context_initialize" << std::endl;
    //     auto &node_context = get_node_context_ref();
    //     node_context.node_name = "APPROACH";
    //     node_context.action_name = "approach";
    //     node_context.action_phase = kios::ActionPhase::APPROACH;
    //     node_context.parameter["skill"]["action_name"] = "approach";
    //     node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::APPROACH;
    //     std::cout << "NEW: " << node_context.node_name << std::endl;
    // }

    // bool Approach::is_success()
    // {
    //     return consume_mios_success();
    // }

    // BT::NodeStatus Approach::tick()
    // {
    //     if (has_succeeded_once())
    //     {
    //         std::cout << "APPROACH HAS ONCE SUCCEEDED" << std::endl;
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     if (is_success())
    //     {
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     else
    //     {
    //         std::cout << "APPROACH GO RUNNNING" << std::endl;
    //         update_tree_state();
    //         return BT::NodeStatus::RUNNING;
    //     }
    // }

} // namespace Insertion
