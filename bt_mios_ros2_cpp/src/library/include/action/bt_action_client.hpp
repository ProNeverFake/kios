/**
 * @file bt_action_client.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "bt_mios_ros2_interface/action/command_test.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"


class BTActionClient : public rclcpp::Node
{
public:
  using BTCommand = bt_mios_ros2_interface::action::CommandTest;
  using GoalHandleBTCommand = rclcpp_action::ClientGoalHandle<BTCommand>;

  explicit BTActionClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
    : Node("behavior_tree_action_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<BTCommand>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "test_action");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&BTActionClient::send_goal, this));
  }

  bool is_goal_done() const;

  void send_goal();

private:
  rclcpp_action::Client<BTCommand>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(GoalHandleBTCommand::SharedPtr goal_handle);

  void feedback_callback(
    GoalHandleBTCommand::SharedPtr,
    const std::shared_ptr<const BTCommand::Feedback> feedback);

  void result_callback(const GoalHandleBTCommand::WrappedResult & result);
};  

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto action_client = std::make_shared<BTActionClient>();

//   while (!action_client->is_goal_done()) {
//     rclcpp::spin_some(action_client);
//   }

//   rclcpp::shutdown();
//   return 0;
// }
