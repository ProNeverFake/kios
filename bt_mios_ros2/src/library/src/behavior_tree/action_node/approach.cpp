#include "behavior_tree/action_node/approach.hpp"

namespace Insertion
{
    Approach::Approach(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config)
    {
    }

    BT::PortsList Approach::providedPorts()
    {
        // amount of milliseconds that we want to sleep

        // return {BT::InputPort<std::vector<double>>("target_position")};
    }

    BT::NodeStatus Approach::onStart()
    {
        // getInput("target_position", target_position);

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
        if (std::chrono::system_clock::now() >= deadline_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    void Approach::onHalted()
    {
        // nothing to do here...
        std::cout << "SleepNode interrupted" << std::endl;
    }

} // namespace Insertion
