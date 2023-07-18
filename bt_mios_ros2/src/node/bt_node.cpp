// /**
//  * @file bt_node.cpp
//  * @author your name (you@domain.com)
//  * @brief the base node of behavior tree
//  * @version 0.1
//  * @date 2023-07-16
//  * 
//  * @copyright Copyright (c) 2023
//  * 
//  */

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <sstream>
// #include <string>
// #include <vector>

// #include <cinttypes>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/publisher_options.hpp"
// #include "std_msgs/msg/string.hpp"

// #include "example_interfaces/action/fibonacci.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"


// #include "bt_node.hpp"

// using namespace std::chrono_literals;

// // the behavior_tree_node
// rclcpp::Node::SharedPtr bt_node = nullptr;



// BehaviorTreeNode::BehaviorTreeNode() : Node("behavior_tree_node"), count_1_(0), count_2_(0)
// {
//   // Create publisher with unique network flow endpoints
//   // Enable unique network flow endpoints via options
//   auto options_1 = rclcpp::PublisherOptions();
//   options_1.require_unique_network_flow_endpoints =
//       RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED;
//   publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10, options_1);
//   timer_1_ = this->create_wall_timer(
//       500ms, std::bind(&BehaviorTreeNode::timer_1_callback, this));

//   // Create publisher without unique network flow endpoints
//   // Unique network flow endpoints are disabled in default options
//   publisher_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
//   timer_2_ = this->create_wall_timer(
//       1000ms, std::bind(&BehaviorTreeNode::timer_2_callback, this));

//   // Catch an exception if implementation does not support get_network_flow_endpoints.
//   try
//   {
//     // Get network flow endpoints
//     auto network_flow_endpoints_1 = publisher_1_->get_network_flow_endpoints();
//     auto network_flow_endpoints_2 = publisher_2_->get_network_flow_endpoints();

//     // Print network flow endpoints
//     print_network_flow_endpoints(network_flow_endpoints_1);
//     print_network_flow_endpoints(network_flow_endpoints_2);
//   }
//   catch (const rclcpp::exceptions::RCLError &e)
//   {
//     RCLCPP_INFO(
//         this->get_logger(), "%s", e.what());
//   }
// }

// void BehaviorTreeNode::timer_1_callback()
// {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, world! " + std::to_string(count_1_++);

//   RCLCPP_INFO(
//       this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   publisher_1_->publish(message);
// }
// void BehaviorTreeNode::timer_2_callback()
// {
//   auto message = std_msgs::msg::String();
//   message.data = "Hej, världen! " + std::to_string(count_2_++);

//   RCLCPP_INFO(
//       this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   publisher_2_->publish(message);
// }
// /// Print network flow endpoints in JSON-like format
// void BehaviorTreeNode::print_network_flow_endpoints(
//     const std::vector<rclcpp::NetworkFlowEndpoint> &network_flow_endpoints) const
// {
//   std::ostringstream stream;
//   stream << "{\"networkFlowEndpoints\": [";
//   bool comma_skip = true;
//   for (auto network_flow_endpoint : network_flow_endpoints)
//   {
//     if (comma_skip)
//     {
//       comma_skip = false;
//     }
//     else
//     {
//       stream << ",";
//     }
//     stream << network_flow_endpoint;
//   }
//   stream << "]}";
//   RCLCPP_INFO(
//       this->get_logger(), "%s",
//       stream.str().c_str());
// }

// using Fibonacci = example_interfaces::action::Fibonacci;
// // todo
// void feedback_callback(
//   rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
//   const std::shared_ptr<const Fibonacci::Feedback> feedback)
// {
//   RCLCPP_INFO(
//     bt_node->get_logger(),
//     "Next number in sequence received: %" PRId32,
//     feedback->sequence.back());
// }



// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);

//   bt_node = rclcpp::Node::make_shared("behavior_tree_node");

//   auto action_client = rclcpp_action::create_client<Fibonacci>(bt_node, "bt_action");

//   // find action server
//   if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
//     RCLCPP_ERROR(bt_node->get_logger(), "Action server not available after waiting");
//     return 1;
//   }

//   // generate a goal msg
//   auto goal_msg = Fibonacci::Goal();
//   goal_msg.order = 10;
  
//   RCLCPP_INFO(bt_node->get_logger(), "Sending goal");

//   // set options for sending the goal
//   auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
//   send_goal_options.feedback_callback = feedback_callback;
//   // generate the future handle used to spin
//   auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

//   // spin the node with the goal handle and get the completed future
//   if (rclcpp::spin_until_future_complete(bt_node, goal_handle_future) !=
//     rclcpp::FutureReturnCode::SUCCESS)
//   {
//     RCLCPP_ERROR(bt_node->get_logger(), "send goal call failed :(");
//     return 1;
//   }

//   // get the handle in spin
//   rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
  
//   if (!goal_handle) { //* if the shareptr is null or empty
//     RCLCPP_ERROR(bt_node->get_logger(), "Goal was rejected by server");
//     return 1;
//   }

//   // get the result handle w.r.t the goal handle
//   auto result_future = action_client->async_get_result(goal_handle);

//   RCLCPP_INFO(bt_node->get_logger(), "Waiting for result");

//   // spin to ask for result with result handle
//   if (rclcpp::spin_until_future_complete(bt_node, result_future) !=
//     rclcpp::FutureReturnCode::SUCCESS)
//   {
//     RCLCPP_ERROR(bt_node->get_logger(), "get result call failed :(");
//     return 1;
//   }

//   // get the result
//   rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrapped_result = result_future.get();

//   // check the result code
//   switch (wrapped_result.code) {
//     case rclcpp_action::ResultCode::SUCCEEDED:
//       break;
//     case rclcpp_action::ResultCode::ABORTED:
//       RCLCPP_ERROR(bt_node->get_logger(), "Goal was aborted");
//       return 1;
//     case rclcpp_action::ResultCode::CANCELED:
//       RCLCPP_ERROR(bt_node->get_logger(), "Goal was canceled");
//       return 1;
//     default:
//       RCLCPP_ERROR(bt_node->get_logger(), "Unknown result code");
//       return 1;
//   }

//   RCLCPP_INFO(bt_node->get_logger(), "result received");

//   // todo
//   for (auto number : wrapped_result.result->sequence) {
//     RCLCPP_INFO(bt_node->get_logger(), "%" PRId32, number);
//   }

//   // reset the client and the node
//   action_client.reset();
//   bt_node.reset();

//   // rclcpp::spin(std::make_shared<BehaviorTreeNode>());
//   rclcpp::shutdown();
//   return 0;
// }












