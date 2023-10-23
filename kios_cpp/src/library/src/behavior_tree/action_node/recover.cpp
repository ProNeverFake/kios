#include "behavior_tree/action_node/recover.hpp"

namespace Insertion
{
    Recover::Recover(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void Recover::update_tree_state()
    {
        spdlog::trace("Recover::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void Recover::node_context_initialize()
    {
        spdlog::trace("Recover::node_context_initialize()");
        // ! add

        auto &node_context = get_node_context_ref();
        node_context.node_name = "RECOVER";
        node_context.action_name = "recover";
        node_context.action_phase = kios::ActionPhase::RECOVER;
    }

    bool Recover::is_success()
    {
        spdlog::trace("Recover::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus Recover::onStart()
    {
        spdlog::trace("Recover::onStart()");

        // if (has_succeeded_once()) // ! should remove this!
        // {
        //     spdlog::debug("CARTESIAN_MOVE HAS ONCE SUCCEEDED");

        //     return BT::NodeStatus::SUCCESS;
        // }
        if (is_success())
        {
            spdlog::debug("RECOVER ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("RECOVER GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus Recover::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("RECOVER SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("RECOVER RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void Recover::onHalted()
    {
        spdlog::trace("Recover::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
