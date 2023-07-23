#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "behavior_tree/action_node/meta_node.hpp"
#include "behavior_tree/action_node/action_context.hpp"

namespace Insertion

{
    class Approach : public MetaNode
    {
    public:
        Approach(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;

        /// method invoked by an action in the RUNNING state.
        BT::NodeStatus onRunning() override;
        // Method invoked when interrupted
        void onHalted() override;

    private:
        std::shared_ptr<ActionNodeContext> node_context_ptr;
        void node_context_initialize() override;
        bool is_success() override;
        std::vector<double> target_position;
        std::chrono::system_clock::time_point deadline_;
    };

} // namespace Insertion
