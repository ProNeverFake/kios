#include "behavior_tree/action_node/gripper_force.hpp"

namespace Insertion
{
    GripperForce::GripperForce(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void GripperForce::update_tree_state()
    {
        spdlog::trace("GripperForce::update_tree_state()");
        // * here must copy
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void GripperForce::node_context_initialize()
    {
        spdlog::trace("GripperForce::node_context_initialize()");
        auto &node_context = get_node_context_ref();
        node_context.node_name = "GRIPPER_FORCE";
        node_context.action_name = "gripper_force";

        node_context.action_phase = kios::ActionPhase::GRIPPER_FORCE;
    }

    bool GripperForce::is_success()
    {
        spdlog::trace("GripperForce::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus GripperForce::onStart()
    {
        spdlog::trace("GripperForce::onStart()");

        if (has_succeeded_once())
        {
            spdlog::debug("GRIPPER_FORCE HAS ONCE SUCCEEDED");
            return BT::NodeStatus::SUCCESS;
        }
        if (is_success())
        {
            spdlog::debug("GRIPPER_FORCE ALREADY SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPER_FORCE GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus GripperForce::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("GRIPPER_FORCE SUCCEEDS");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPER_FORCE RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void GripperForce::onHalted()
    {
        // * interrupted behavior. do nothing.
        spdlog::trace("GripperForce::onHalted()");
    }

} // namespace Insertion
