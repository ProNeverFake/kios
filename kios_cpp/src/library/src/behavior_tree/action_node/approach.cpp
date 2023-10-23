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
        spdlog::trace("Approach::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void Approach::node_context_initialize()
    {
        spdlog::trace("Approach::node_context_initialize()");
        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("Approach");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "APPROACH";
        node_context.action_name = "approach";
        node_context.action_phase = kios::ActionPhase::APPROACH;
    }

    bool Approach::is_success()
    {
        spdlog::trace("Approach::is_success()");
        return consume_mios_success();
    }

    BT::NodeStatus Approach::onStart()
    {
        spdlog::trace("Approach::onStart()");

        if (has_succeeded_once())
        {
            spdlog::debug("APPROACH HAS ONCE SUCCEEDED");
            return BT::NodeStatus::SUCCESS;
        }
        if (is_success())
        {
            spdlog::debug("APPROACH ALREADY SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("APPROACH GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus Approach::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("APPROACH SUCCEEDS");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("APPROACH RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void Approach::onHalted()
    {
        // * interrupted behavior. do nothing.
        spdlog::trace("Approach::onHalted()");
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
