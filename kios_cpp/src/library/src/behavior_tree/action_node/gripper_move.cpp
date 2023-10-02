#include "behavior_tree/action_node/gripper_move.hpp"

namespace Insertion
{
    GripperMove::GripperMove(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : HyperMetaNode<BT::StatefulActionNode>(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void GripperMove::update_tree_state()
    {
        spdlog::trace("GripperMove::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void GripperMove::node_context_initialize()
    {
        spdlog::trace("GripperMove::node_context_initialize()");
        auto &node_context = get_node_context_ref();
        node_context.node_name = "GRIPPER_MOVE";
        node_context.action_name = "gripper_move";

        node_context.action_phase = kios::ActionPhase::GRIPPER_MOVE;
    }

    bool GripperMove::is_success()
    {
        spdlog::trace("GripperMove::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus GripperMove::onStart()
    {
        spdlog::trace("GripperMove::onStart()");

        if (has_succeeded_once())
        {
            spdlog::debug("GRIPPERMOVE HAS ONCE SUCCEEDED");
            return BT::NodeStatus::SUCCESS;
        }
        if (is_success())
        {
            spdlog::debug("GRIPPERMOVE ALREADY SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERMOVE GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus GripperMove::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("GRIPPERMOVE SUCCEEDS");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERMOVE RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void GripperMove::onHalted()
    {
        // * interrupted behavior. do nothing.
        spdlog::trace("GripperMove::onHalted()");
    }

} // namespace Insertion
