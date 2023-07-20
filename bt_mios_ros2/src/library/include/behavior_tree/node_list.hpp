#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "behavior_tree/action_node/approach.hpp"
#include "behavior_tree/action_node/fit.hpp"
#include "behavior_tree/action_node/align.hpp"
#include "behavior_tree/action_node/push.hpp"
#include "behavior_tree/action_node/reach.hpp"

namespace DummyNodes
{

    using BT::NodeStatus;

    NodeStatus CheckBattery();

    NodeStatus CheckTemperature();
    NodeStatus SayHello();

    class GripperInterface
    {
    public:
        GripperInterface()
            : _opened(true)
        {
        }

        NodeStatus open();

        NodeStatus close();

    private:
        bool _opened;
    };

    // Example os Asynchronous node that use StatefulActionNode as base class
    class SleepNode : public BT::StatefulActionNode
    {
    public:
        SleepNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            // amount of milliseconds that we want to sleep
            return {BT::InputPort<int>("msec")};
        }

        NodeStatus onStart() override
        {
            int msec = 0;
            getInput("msec", msec);
            if (msec <= 0)
            {
                // no need to go into the RUNNING state
                return NodeStatus::SUCCESS;
            }
            else
            {
                using namespace std::chrono;
                // once the deadline is reached, we will return SUCCESS.
                deadline_ = system_clock::now() + milliseconds(msec);
                return NodeStatus::RUNNING;
            }
        }

        /// method invoked by an action in the RUNNING state.
        NodeStatus onRunning() override
        {
            if (std::chrono::system_clock::now() >= deadline_)
            {
                return NodeStatus::SUCCESS;
            }
            else
            {
                return NodeStatus::RUNNING;
            }
        }

        void onHalted() override
        {
            // nothing to do here...
            std::cout << "SleepNode interrupted" << std::endl;
        }

    private:
        std::chrono::system_clock::time_point deadline_;
    };

} // namespace DummyNodes

// BB CODE
namespace Insertion
{
    /**
     * @brief a meta node class for insertion behavior tree node
     *
     */
    // class MetaNode : public BT::StatefulActionNode
    // {
    // public:
    //     MetaNode(const std::string &name, const BT::NodeConfig &config)
    //         : BT::StatefulActionNode(name, config)
    //     {
    //     }
    //     auto chose_action(); // todo
    // };

    inline void RegisterNodes(BT::BehaviorTreeFactory &factory)
    {
        // static GripperInterface grip_singleton;
        // factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
        // factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &grip_singleton));
        // factory.registerNodeType<Approach>("Approach");
        // factory.registerNodeType<Reach>("Reach");
    }

} // namespace Insertion

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
