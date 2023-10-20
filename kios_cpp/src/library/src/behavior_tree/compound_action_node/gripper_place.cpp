#include "behavior_tree/compound_action_node/gripper_place.hpp"

namespace Insertion
{
    GripperPlace::GripperPlace(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void GripperPlace::update_tree_state()
    {
        spdlog::trace("GripperPlace::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void GripperPlace::node_context_initialize()
    {
        spdlog::trace("GripperPlace::node_context_initialize()");
        // ! add
        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("Place");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "Gripper_Place";
        node_context.action_name = "gripper_place";
        node_context.action_phase = kios::ActionPhase::GRIPPER_PLACE;
    }

    bool GripperPlace::is_success()
    {
        spdlog::trace("GripperPlace::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus GripperPlace::onStart()
    {
        spdlog::trace("GripperPlace::onStart()");

        if (is_success())
        {
            spdlog::debug("GRIPPERPLACE ALREADY SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERPLACE GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus GripperPlace::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("GRIPPERPLACE SUCCEEDED");
            on_success();
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("GRIPPERPLACE RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void GripperPlace::onHalted()
    {
        spdlog::trace("GripperPlace::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
