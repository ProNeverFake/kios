#include "behavior_tree/action_node/wiggle.hpp"

namespace Insertion
{
    Wiggle::Wiggle(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::ActionPhaseContext> context_ptr, std::shared_ptr<kios::RobotState> state_ptr)
        : MetaNode(name, config)
    {
        m_node_context_ptr = context_ptr;
        m_robot_state_ptr = state_ptr;
    }

    /**
     * @brief Here to apply the success condition check.
     *
     * @return true
     * @return false
     */
    bool Wiggle::is_success()
    {
        // TODO
        return false;
    }
    /**
     * @brief temporarily just set action_name
     *
     */
    void Wiggle::set_action_context()
    {
        m_node_context_ptr->parameter["skill"]["action_name"] = "wiggle";
        m_node_context_ptr->action_phase = kios::ActionPhase::WIGGLE;
        m_node_context_ptr->parameter["skill"]["action_phase"] = kios::ActionPhase::WIGGLE;
    }
    void Wiggle::node_context_initialize()
    {
        std::shared_ptr<kios::ActionPhaseContext> context_ptr = get_context_ptr();
        context_ptr->node_name = "wiggle";
        // todo add more command context here.
    }

    BT::NodeStatus Wiggle::onStart()
    {
        // getInput("target_position", target_position);
        // * get the current state from data_pool class
        // * check the state
        if (is_success())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
        // if (msec <= 0)
        // {
        //     // no need to go into the RUNNING state
        //     return BT::NodeStatus::SUCCESS;
        // }
        // else
        // {
        //     using namespace std::chrono;
        //     // once the deadline is reached, we will return SUCCESS.
        //     deadline_ = system_clock::now() + milliseconds(msec);
        //     return BT::NodeStatus::RUNNING;
        // }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus Wiggle::onRunning()
    {
        if (is_success())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    void Wiggle::onHalted()
    {
        // nothing to do here...
        std::cout << "Action stoped" << std::endl;
    }

} // namespace Insertion
