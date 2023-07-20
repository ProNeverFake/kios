// #include <behaviortree_cpp/behavior_tree.h>
// #include <behaviortree_cpp/bt_factory.h>

// namespace Insertion

// {
//     class Fit : public BT::StatefulActionNode
//     {
//     public:
//         Fit(const std::string &name, const BT::NodeConfig &config);

//         static BT::PortsList providedPorts();

//         BT::NodeStatus onStart() override;

//         /// method invoked by an action in the RUNNING state.
//         BT::NodeStatus onRunning() override;
//         // Method invoked when interrupted
//         void onHalted() override;

//     private:
//         std::vector<double> target_position;
//         std::chrono::system_clock::time_point deadline_;
//     };

// } // namespace Insertion
