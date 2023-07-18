// /**
//  * @file bt_action_server.cpp
//  * @author your name (you@domain.com)
//  * @brief action server for responsing the state of the tasks
//  * @version 0.1
//  * @date 2023-07-16
//  * 
//  * @copyright Copyright (c) 2023
//  * 
//  */

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "action/bt_action_server.hpp"

// TODO multiple def of main function
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<BTActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}


// class BTActionServer : public rclcpp::Node
// {
// public:
//   using Fibonacci = example_interfaces::action::Fibonacci;
//   using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

//   explicit BTActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
//   : Node("mios_action_node", options)
//   {
//     using namespace std::placeholders;

//     this->action_server_ = rclcpp_action::create_server<Fibonacci>(
//       this->get_node_base_interface(),
//       this->get_node_clock_interface(),
//       this->get_node_logging_interface(),
//       this->get_node_waitables_interface(),
//       "fibonacci",
//       std::bind(&BTActionServer::handle_goal, this, _1, _2),
//       std::bind(&BTActionServer::handle_cancel, this, _1),
//       std::bind(&BTActionServer::handle_accepted, this, _1));
//   }

// private:
//   rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

//   rclcpp_action::GoalResponse handle_goal(
//     const rclcpp_action::GoalUUID & uuid,
//     std::shared_ptr<const Fibonacci::Goal> goal)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
//     (void)uuid;
//     // Let's reject sequences that are over 9000
//     if (goal->order > 9000) {
//       return rclcpp_action::GoalResponse::REJECT;
//     }
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse handle_cancel(
//     const std::shared_ptr<GoalHandleFibonacci> goal_handle)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
//     (void)goal_handle;
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }

//   void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
//   {
//     RCLCPP_INFO(this->get_logger(), "Executing goal");
//     rclcpp::Rate loop_rate(1);
//     const auto goal = goal_handle->get_goal();
//     auto feedback = std::make_shared<Fibonacci::Feedback>();
//     auto & sequence = feedback->sequence;
//     sequence.push_back(0);
//     sequence.push_back(1);
//     auto result = std::make_shared<Fibonacci::Result>();

//     for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
//       // Check if there is a cancel request
//       if (goal_handle->is_canceling()) {
//         result->sequence = sequence;
//         goal_handle->canceled(result);
//         RCLCPP_INFO(this->get_logger(), "Goal Canceled");
//         return;
//       }
//       // Update sequence
//       sequence.push_back(sequence[i] + sequence[i - 1]);
//       // Publish feedback
//       goal_handle->publish_feedback(feedback);
//       RCLCPP_INFO(this->get_logger(), "Publish Feedback");

//       loop_rate.sleep();
//     }

//     // Check if goal is done
//     if (rclcpp::ok()) {
//       result->sequence = sequence;
//       goal_handle->succeed(result);
//       RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
//     }
//   }

//   void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
//   {
//     using namespace std::placeholders;
//     // this needs to return quickly to avoid blocking the executor, so spin up a new thread
//     std::thread{std::bind(&BTActionServer::execute, this, _1), goal_handle}.detach();
//   }
// };  // class BTActionServer

// // TODO multiple def of main function
// // int main(int argc, char ** argv)
// // {
// //   rclcpp::init(argc, argv);

// //   auto action_server = std::make_shared<BTActionServer>();

// //   rclcpp::spin(action_server);

// //   rclcpp::shutdown();
// //   return 0;
// // }
