#include "behavior_tree/meta_node/meta_node.hpp"

namespace Insertion

{
    class Wiggle : public HyperMetaNode<BT::StatefulActionNode>
    {
    public:
        Wiggle(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);

        BT::NodeStatus onStart() override;
        /// method invoked by an action in the RUNNING state.
        BT::NodeStatus onRunning() override;

        // Method invoked when interrupted
        void onHalted() override;
        void node_context_initialize() override; //
        void update_tree_state() override;

        bool is_success() override;

    private:
        std::chrono::system_clock::time_point deadline_;
    };

    // class Approach : public HyperMetaNode<BT::StatefulActionNode>
    // {
    // public:
    //     Approach(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);

    //     BT::NodeStatus onStart() override;

    //     /// method invoked by an action in the RUNNING state.
    //     BT::NodeStatus onRunning() override;
    //     // Method invoked when interrupted
    //     void onHalted() override;
    //     void node_context_initialize() override; //
    //     void update_tree_state() override;
    //     bool is_success() override;

    // private:
    //     std::chrono::system_clock::time_point deadline_;
    // };

    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    // class Approach : public HyperMetaNode<BT::SyncActionNode>
    // {
    // public:
    //     Approach(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);

    //     BT::NodeStatus tick() override;

    //     void node_context_initialize() override; //
    //     void update_tree_state() override;

    //     bool is_success() override;

    // private:
    //     std::chrono::system_clock::time_point deadline_;
    // };

} // namespace Insertion
