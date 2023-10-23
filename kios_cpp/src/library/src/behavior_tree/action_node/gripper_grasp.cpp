#include "behavior_tree/action_node/gripper_grasp.hpp"

namespace Insertion
{
    GripperGrasp::GripperGrasp(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void GripperGrasp::update_tree_state()
    {
        spdlog::trace("GripperGrasp::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void GripperGrasp::node_context_initialize()
    {
        spdlog::trace("GripperGrasp::node_context_initialize()");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "GripperGrasp";
        node_context.action_name = "grippergrasp";
        node_context.action_phase = kios::ActionPhase::GRIPPER_GRASP;
    }

    bool GripperGrasp::is_success()
    {
        spdlog::trace("GripperGrasp::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus GripperGrasp::onStart()
    {
        spdlog::trace("GripperGrasp::onStart()");

        if (is_success())
        {
            spdlog::debug("GRIPPERGRASP ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERGRASP GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus GripperGrasp::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("GRIPPERGRASP SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERGRASP RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void GripperGrasp::onHalted()
    {
        spdlog::trace("GripperGrasp::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
