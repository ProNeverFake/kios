// /**
//  * @file mios_node.cpp
//  * @author your name (you@domain.com)
//  * @brief the mios communication node 
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

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/publisher_options.hpp"
// #include "std_msgs/msg/string.hpp"

// #include "mios_node.hpp"

// using namespace std::chrono_literals;

// class MiosNode : public rclcpp::Node
// {
// public:
//   MiosNode()
//   : Node("mios_node"), count_1_(0), count_2_(0)
//   {
//     // Create publisher with unique network flow endpoints
//     // Enable unique network flow endpoints via options
//     auto options_1 = rclcpp::PublisherOptions();
//     options_1.require_unique_network_flow_endpoints =
//       RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED;
//     publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10, options_1);
//     timer_1_ = this->create_wall_timer(
//       500ms, std::bind(&MiosNode::timer_1_callback, this));

//     // Create publisher without unique network flow endpoints
//     // Unique network flow endpoints are disabled in default options
//     publisher_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
//     timer_2_ = this->create_wall_timer(
//       1000ms, std::bind(&MiosNode::timer_2_callback, this));

//     // Catch an exception if implementation does not support get_network_flow_endpoints.
//     try {
//       // Get network flow endpoints
//       auto network_flow_endpoints_1 = publisher_1_->get_network_flow_endpoints();
//       auto network_flow_endpoints_2 = publisher_2_->get_network_flow_endpoints();

//       // Print network flow endpoints
//       print_network_flow_endpoints(network_flow_endpoints_1);
//       print_network_flow_endpoints(network_flow_endpoints_2);
//     } catch (const rclcpp::exceptions::RCLError & e) {
//       RCLCPP_INFO(
//         this->get_logger(), "%s", e.what());
//     }
//   }

// private:
//   void timer_1_callback()
//   {
//     auto message = std_msgs::msg::String();
//     message.data = "Hello, world! " + std::to_string(count_1_++);

//     RCLCPP_INFO(
//       this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_1_->publish(message);
//   }
//   void timer_2_callback()
//   {
//     auto message = std_msgs::msg::String();
//     message.data = "Hej, världen! " + std::to_string(count_2_++);

//     RCLCPP_INFO(
//       this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_2_->publish(message);
//   }
//   /// Print network flow endpoints in JSON-like format
//   void print_network_flow_endpoints(
//     const std::vector<rclcpp::NetworkFlowEndpoint> & network_flow_endpoints) const
//   {
//     std::ostringstream stream;
//     stream << "{\"networkFlowEndpoints\": [";
//     bool comma_skip = true;
//     for (auto network_flow_endpoint : network_flow_endpoints) {
//       if (comma_skip) {
//         comma_skip = false;
//       } else {
//         stream << ",";
//       }
//       stream << network_flow_endpoint;
//     }
//     stream << "]}";
//     RCLCPP_INFO(
//       this->get_logger(), "%s",
//       stream.str().c_str());
//   }
//   rclcpp::TimerBase::SharedPtr timer_1_;
//   rclcpp::TimerBase::SharedPtr timer_2_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
//   size_t count_1_;
//   size_t count_2_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MiosNode>());
//   rclcpp::shutdown();
//   return 0;
// }
