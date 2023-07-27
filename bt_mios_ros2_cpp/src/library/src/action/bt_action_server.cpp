/**
 * @file bt_action_server.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <functional>
#include <memory>
#include <thread>

#include "bt_mios_ros2_interface/action/command_test.hpp"
#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

#include "action/bt_action_server.hpp"

rclcpp_action::GoalResponse BTActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const BTCommand::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->command.c_str());
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->command == "test_reject") {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

rclcpp_action::CancelResponse BTActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleBTCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

void BTActionServer::execute(const std::shared_ptr<GoalHandleBTCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    //* create feedback ptr
    auto feedback = std::make_shared<BTCommand::Feedback>();
    auto & state_number = feedback->state_number;
    state_number.push_back(0);
    //* create result ptr
    auto result = std::make_shared<BTCommand::Result>();
    auto & result_number = result->result_number;
    result_number = goal->command_number;

    int i = 0;
    while(i<100 && rclcpp::ok()){

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->result = "canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }

      // update the result
      result_number++;
      // update the feedback
      feedback->state_number.push_back(result_number);
      
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");
      i++;
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) { //* this only check the state of rclcpp!!!
      result->result = "success";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

void BTActionServer::handle_accepted(const std::shared_ptr<GoalHandleBTCommand> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&BTActionServer::execute, this, _1), goal_handle}.detach();
  }


// TODO multiple def of main function
// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);

//   auto action_server = std::make_shared<BTActionServer>();

//   rclcpp::spin(action_server);

//   rclcpp::shutdown();
//   return 0;
// }
