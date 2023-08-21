#include "behavior_tree/action_node/meta_node.hpp"

namespace Insertion

{
    class Contact : public MetaNode
    {
    public:
        Contact(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);

        BT::NodeStatus onStart() override;

        /// method invoked by an action in the RUNNING state.
        BT::NodeStatus onRunning() override;
        // Method invoked when interrupted
        void onHalted() override;
        void node_context_initialize() override; //
        void update_tree_state() override;

    private:
        bool is_success() override;

        std::chrono::system_clock::time_point deadline_;
    };

} // namespace Insertion
