#include "behavior_tree/action_node/tool_grasp.hpp"

namespace Insertion
{
    ToolGrasp::ToolGrasp(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void ToolGrasp::update_tree_state()
    {
        spdlog::trace("ToolGrasp::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void ToolGrasp::node_context_initialize()
    {
        spdlog::trace("ToolGrasp::node_context_initialize()");
        // ! add
        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("ToolGrasp");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "Tool_Grasp";
        node_context.action_name = "tool_grasp";
        node_context.action_phase = kios::ActionPhase::TOOL_GRASP;
    }

    bool ToolGrasp::is_success()
    {
        spdlog::trace("ToolGrasp::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus ToolGrasp::onStart()
    {
        spdlog::trace("ToolGrasp::onStart()");

        if (is_success())
        {
            spdlog::debug("TOOLGRASP ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLGRASP GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus ToolGrasp::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("TOOLGRASP SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("TOOLGRASP RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void ToolGrasp::onHalted()
    {
        spdlog::trace("ToolGrasp::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
