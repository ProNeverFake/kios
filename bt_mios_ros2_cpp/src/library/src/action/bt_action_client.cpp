/**
 * @file bt_action_client.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-17
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

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action/bt_action_client.hpp"

bool BTActionClient::is_goal_done() const
{
  return this->goal_done_;
}

void BTActionClient::send_goal()
{
  using namespace std::placeholders;

  this->timer_->cancel();

  this->goal_done_ = false;

  if (!this->client_ptr_)
  {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->goal_done_ = true;
    return;
  }

  auto goal_msg = BTCommand::Goal();
  goal_msg.command = "test_goal";

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<BTCommand>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&BTActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&BTActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&BTActionClient::result_callback, this, _1);
  auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void BTActionClient::goal_response_callback(GoalHandleBTCommand::SharedPtr goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void BTActionClient::feedback_callback(
    GoalHandleBTCommand::SharedPtr,
    const std::shared_ptr<const BTCommand::Feedback> feedback)
{
  RCLCPP_INFO(
      this->get_logger(),
      "The feedback received: %" PRId32,
      feedback->state_number.back());
}

void BTActionClient::result_callback(const GoalHandleBTCommand::WrappedResult &result)
{
  this->goal_done_ = true;
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received");
  RCLCPP_INFO(this->get_logger(), "%" PRId32, result.result->result_number);
  // for (auto number : result.result->result_number)
  // {
  //   RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
  // }
}

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
