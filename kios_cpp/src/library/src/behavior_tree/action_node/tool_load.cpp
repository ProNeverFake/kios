#include "behavior_tree/action_node/tool_load.hpp"

namespace Insertion
{
    ToolLoad::ToolLoad(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void ToolLoad::update_tree_state()
    {
        spdlog::trace("ToolLoad::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void ToolLoad::node_context_initialize()
    {
        spdlog::trace("ToolLoad::node_context_initialize()");
        // ! add
        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("ToolLoad");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "Tool_Load";
        node_context.action_name = "tool_load";
        node_context.action_phase = kios::ActionPhase::TOOL_LOAD;
    }

    bool ToolLoad::is_success()
    {
        spdlog::trace("ToolLoad::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus ToolLoad::onStart()
    {
        spdlog::trace("ToolLoad::onStart()");

        if (is_success())
        {
            spdlog::debug("TOOLLOAD ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLLOAD GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus ToolLoad::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("TOOLLOAD SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLLOAD RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void ToolLoad::onHalted()
    {
        spdlog::trace("ToolLoad::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
