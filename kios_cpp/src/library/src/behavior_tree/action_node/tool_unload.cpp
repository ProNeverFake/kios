#include "behavior_tree/action_node/tool_unload.hpp"

namespace Insertion
{
    ToolUnload::ToolUnload(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void ToolUnload::update_tree_state()
    {
        spdlog::trace("ToolUnload::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void ToolUnload::node_context_initialize()
    {
        spdlog::trace("ToolUnload::node_context_initialize()");
        // ! add
        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("ToolLoad"); // ! here still use load's object because of the skill obejct definition in mios

        auto &node_context = get_node_context_ref();
        node_context.node_name = "Tool_Unload";
        node_context.action_name = "tool_unload";
        node_context.action_phase = kios::ActionPhase::TOOL_UNLOAD;
    }

    bool ToolUnload::is_success()
    {
        spdlog::trace("ToolUnload::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus ToolUnload::onStart()
    {
        spdlog::trace("ToolUnload::onStart()");

        if (is_success())
        {
            spdlog::debug("TOOLUNLOAD ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLUNLOAD GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus ToolUnload::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("TOOLUNLOAD SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLUNLOAD RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void ToolUnload::onHalted()
    {
        spdlog::trace("ToolUnload::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
