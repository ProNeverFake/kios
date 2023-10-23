#include "behavior_tree/compound_action_node/gripper_pick.hpp"

namespace Insertion
{
    GripperPick::GripperPick(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void GripperPick::update_tree_state()
    {
        spdlog::trace("GripperPick::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void GripperPick::node_context_initialize()
    {
        spdlog::trace("GripperPick::node_context_initialize()");
        // ! add
        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("Pick");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "GripperPick";
        node_context.action_name = "gripper_pick";
        node_context.action_phase = kios::ActionPhase::GRIPPER_PICK;
    }

    bool GripperPick::is_success()
    {
        spdlog::trace("GripperPick::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus GripperPick::onStart()
    {
        spdlog::trace("GripperPick::onStart()");

        if (is_success())
        {
            spdlog::debug("GRIPPERPICK ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERPICK GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus GripperPick::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("GRIPPERPICK SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERPICK RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void GripperPick::onHalted()
    {
        spdlog::trace("GripperPick::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
