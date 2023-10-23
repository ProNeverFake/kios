#include "behavior_tree/compound_action_node/tool_pick.hpp"

namespace Insertion
{
    ToolPick::ToolPick(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void ToolPick::update_tree_state()
    {
        spdlog::trace("ToolPick::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void ToolPick::node_context_initialize()
    {
        spdlog::trace("ToolPick::node_context_initialize()");
        // ! add
        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("Pick");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "Tool_Pick";
        node_context.action_name = "tool_pick";
        node_context.action_phase = kios::ActionPhase::TOOL_PICK;
    }

    bool ToolPick::is_success()
    {
        spdlog::trace("ToolPick::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus ToolPick::onStart()
    {
        spdlog::trace("ToolPick::onStart()");

        if (is_success())
        {
            spdlog::debug("TOOLPICK ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLPICK GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus ToolPick::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("TOOLPICK SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLPICK RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void ToolPick::onHalted()
    {
        spdlog::trace("ToolPick::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
