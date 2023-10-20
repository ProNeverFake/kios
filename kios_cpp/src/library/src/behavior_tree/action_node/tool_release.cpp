#include "behavior_tree/action_node/tool_release.hpp"

namespace Insertion
{
    ToolRelease::ToolRelease(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void ToolRelease::update_tree_state()
    {
        spdlog::trace("ToolRelease::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void ToolRelease::node_context_initialize()
    {
        spdlog::trace("ToolRelease::node_context_initialize()");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "ToolRelease";
        node_context.action_name = "toolrelease";
        node_context.action_phase = kios::ActionPhase::TOOL_RELEASE;
    }

    bool ToolRelease::is_success()
    {
        spdlog::trace("ToolRelease::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus ToolRelease::onStart()
    {
        spdlog::trace("ToolRelease::onStart()");

        if (is_success())
        {
            spdlog::debug("TOOLRELEASE ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLRELEASE GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus ToolRelease::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("TOOLRELEASE SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLRELEASE RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void ToolRelease::onHalted()
    {
        spdlog::trace("ToolRelease::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
