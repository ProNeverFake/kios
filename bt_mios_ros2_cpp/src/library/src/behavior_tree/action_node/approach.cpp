#include "behavior_tree/action_node/approach.hpp"

namespace Insertion
{
    Approach::Approach(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<ActionNodeContext> context_ptr, std::shared_ptr<RobotState> state_ptr)
        : MetaNode(name, config)
    {
        m_node_context_ptr = context_ptr;
        m_robot_state_ptr = state_ptr;
    }

    BT::PortsList Approach::providedPorts()
    {
        // amount of milliseconds that we want to sleep

        return {BT::InputPort<std::vector<double>>("target_position")};
    }
    /**
     * @brief Here to apply the success condition check.
     *
     * @return true
     * @return false
     */
    bool Approach::is_success()
    {
        return true;
    };
    void Approach::node_context_initialize()
    {
        std::shared_ptr<ActionNodeContext> context_ptr = get_context_ptr();
        context_ptr->node_name = "approach";
        // todo add more command context here.
    };

    BT::NodeStatus Approach::onStart()
    {
        // getInput("target_position", target_position);
        // * get the current state from data_pool class
        // * check the state
        if (is_success())
        {
            return BT::NodeStatus::RUNNING;
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
    BT::NodeStatus Approach::onRunning()
    {
        if (is_success())
        {
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    void Approach::onHalted()
    {
        // nothing to do here...
        std::cout << "Action stoped" << std::endl;
    }

} // namespace Insertion
