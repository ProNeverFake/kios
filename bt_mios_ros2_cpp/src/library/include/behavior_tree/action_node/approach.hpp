#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "behavior_tree/action_node/meta_node.hpp"
// #include "behavior_tree/action_node/action_context.hpp"

namespace Insertion

{
    class Approach : public MetaNode
    {
    public:
        Approach(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<ActionNodeContext> context_ptr, std::shared_ptr<RobotState> state_ptr);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;

        /// method invoked by an action in the RUNNING state.
        BT::NodeStatus onRunning() override;
        // Method invoked when interrupted
        void onHalted() override;

    private:
        int number;
        std::shared_ptr<ActionNodeContext> m_node_context_ptr;
        std::shared_ptr<RobotState> m_robot_state_ptr;
        void node_context_initialize();
        bool is_success();
        std::chrono::system_clock::time_point deadline_;
    };

} // namespace Insertion
