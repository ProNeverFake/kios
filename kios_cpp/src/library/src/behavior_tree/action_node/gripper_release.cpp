#include "behavior_tree/action_node/gripper_release.hpp"

namespace Insertion
{
    GripperRelease::GripperRelease(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void GripperRelease::update_tree_state()
    {
        spdlog::trace("GripperRelease::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void GripperRelease::node_context_initialize()
    {
        spdlog::trace("GripperRelease::node_context_initialize()");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "GripperRelease";
        node_context.action_name = "gripperrelease";
        node_context.action_phase = kios::ActionPhase::GRIPPER_RELEASE;
    }

    bool GripperRelease::is_success()
    {
        spdlog::trace("GripperRelease::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus GripperRelease::onStart()
    {
        spdlog::trace("GripperRelease::onStart()");

        if (is_success())
        {
            spdlog::debug("GRIPPERRELEASE ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERRELEASE GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus GripperRelease::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("GRIPPERRELEASE SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERRELEASE RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void GripperRelease::onHalted()
    {
        spdlog::trace("GripperRelease::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
