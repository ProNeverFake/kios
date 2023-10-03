#include "behavior_tree/action_node/joint_move.hpp"

namespace Insertion
{
    JointMove::JointMove(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : KiosActionNode(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void JointMove::update_tree_state()
    {
        spdlog::trace("JointMove::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;

        get_tree_state_ptr()->object_keys = get_obejct_keys_ref();
        get_tree_state_ptr()->object_names = get_object_names_ref();
        // ! add archive
        get_tree_state_ptr()->node_archive = get_archive_ref();
    }

    void JointMove::node_context_initialize()
    {
        spdlog::trace("JointMove::node_context_initialize()");
        // ! add
        auto &obj_keys = get_obejct_keys_ref();
        obj_keys.push_back("JointMove");

        auto &node_context = get_node_context_ref();
        node_context.node_name = "JOINT_MOVE";
        node_context.action_name = "joint_move";

        node_context.action_phase = kios::ActionPhase::JOINT_MOVE;
    }

    bool JointMove::is_success()
    {
        spdlog::trace("JointMove::is_success()");
        // * THIS SKILL CONSUME SUCCESS FROM MIOS
        return consume_mios_success();
    }

    BT::NodeStatus JointMove::onStart()
    {
        spdlog::trace("JointMove::onStart()");

        if (has_succeeded_once())
        {
            spdlog::debug("CARTESIAN_MOVE HAS ONCE SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        if (is_success())
        {
            spdlog::debug("JOINTMOVE ALREADY SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("JOINTMOVE GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus JointMove::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("JOINTMOVE SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("JOINTMOVE RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void JointMove::onHalted()
    {
        spdlog::trace("JointMove::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
