#include "behavior_tree/action_node/wiggle.hpp"

namespace Insertion
{
    Wiggle::Wiggle(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void Wiggle::update_tree_state()
    {
        std::cout << "WIGGLE UPDATE TREE STATE" << std::endl;
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void Wiggle::node_context_initialize()
    {
        std::cout << "WIGGLE INITIALIZE" << std::endl;

        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("Wiggle");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "WIGGLE";
        node_context.action_name = "wiggle";

        node_context.action_phase = kios::ActionPhase::WIGGLE;
    }

    bool Wiggle::is_success()
    {
        std::cout << "WIGGLE IS_SUCCESS?" << std::endl;

        return consume_mios_success();
    }

    BT::NodeStatus Wiggle::onStart()
    {
        std::cout << "WIGGLE ON START" << std::endl;
        // ! normal action
        // if (has_succeeded_once())
        // {
        //     std::cout << "WIGGLE HAS ONCE SUCCEEDED" << std::endl;
        //     return BT::NodeStatus::SKIPPED;
        // }
        if (is_success())
        {
            std::cout << "WIGGLE ALREADY SUCCESS" << std::endl;

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "WIGGLE GO RUNNING" << std::endl;

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus Wiggle::onRunning()
    {
        std::cout << "WIGGLE ON RUNNING" << std::endl;
        if (has_succeeded_once())
        {
            std::cout << "WIGGLE HAS ONCE SUCCEEDED == true" << std::endl;
            return BT::NodeStatus::SKIPPED;
        }
        if (is_success())
        {
            std::cout << "WIGGLE SUCCESS" << std::endl;

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "WIGGLE RUNNING" << std::endl;
            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void Wiggle::onHalted()
    {
        // * interrupted behavior. do nothing.
        std::cout << "WIGGLE ON HALTED" << std::endl;
    }

    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    // Wiggle::Wiggle(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
    //     : HyperMetaNode<BT::SyncActionNode>(name, config, tree_state_ptr, task_state_ptr)
    // {
    //     // initialize local context
    //     node_context_initialize();
    // }

    // void Wiggle::update_tree_state()
    // {
    //     std::cout << "WIGGLE UPDATE TREE STATE" << std::endl;
    //     get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
    //     get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;
    //     std::cout << "UPDATED VALUE: " << get_tree_state_ptr()->action_name << std::endl;
    // }

    // void Wiggle::node_context_initialize()
    // {
    //     std::cout << "node_context_initialize" << std::endl;
    //     auto &node_context = get_node_context_ref();
    //     node_context.node_name = "WIGGLE";
    //     node_context.action_name = "wiggle";
    //     node_context.action_phase = kios::ActionPhase::WIGGLE;
    //     node_context.parameter["skill"]["action_name"] = "wiggle";
    //     node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::WIGGLE;
    //     std::cout << "NEW: " << node_context.node_name << std::endl;
    // }

    // bool Wiggle::is_success()
    // {
    //     return consume_mios_success();
    // }

    // BT::NodeStatus Wiggle::tick()
    // {
    //     if (has_succeeded_once())
    //     {
    //         std::cout << "WIGGLE HAS ONCE SUCCEEDED" << std::endl;
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     if (is_success())
    //     {
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     else
    //     {
    //         std::cout << "WIGGLE GO RUNNNING" << std::endl;
    //         update_tree_state();
    //         return BT::NodeStatus::RUNNING;
    //     }
    // }

} // namespace Insertion
