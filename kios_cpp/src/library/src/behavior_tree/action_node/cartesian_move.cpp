#include "behavior_tree/action_node/cartesian_move.hpp"

namespace Insertion
{
    CartesianMove::CartesianMove(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : HyperMetaNode<BT::StatefulActionNode>(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    void CartesianMove::update_tree_state()
    {
        spdlog::trace("CartesianMove::update_tree_state()");
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;
    }

    void CartesianMove::node_context_initialize()
    {
        spdlog::trace("CartesianMove::node_context_initialize()");
        auto &node_context = get_node_context_ref();
        node_context.node_name = "CARTESIAN_MOVE";
        node_context.action_name = "cartesian_move";
        node_context.action_phase = kios::ActionPhase::CARTESIAN_MOVE;
        node_context.parameter["skill"]["action_name"] = "cartesian_ move";
        node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::CARTESIAN_MOVE;
    }

    bool CartesianMove::is_success()
    {
        spdlog::trace("CartesianMove::is_success()");

        return consume_mios_success();
    }

    BT::NodeStatus CartesianMove::onStart()
    {
        spdlog::trace("CartesianMove::onStart()");

        if (has_succeeded_once())
        {
            spdlog::debug("CARTESIAN_MOVE HAS ONCE SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        if (is_success())
        {
            spdlog::debug("CARTESIANMOVE ALREADY SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("CARTESIANMOVE GO RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus CartesianMove::onRunning()
    {
        if (is_success())
        {
            spdlog::debug("CARTESIANMOVE SUCCEEDED");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            spdlog::debug("CARTESIANMOVE RUNNING");

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void CartesianMove::onHalted()
    {
        spdlog::trace("CartesianMove::onHalted()");

        // * interrupted behavior. do nothing.
    }

} // namespace Insertion
