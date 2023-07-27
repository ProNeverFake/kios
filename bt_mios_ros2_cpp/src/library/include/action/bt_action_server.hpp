// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <thread>

#include "bt_mios_ros2_interface/action/command_test.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class BTActionServer : public rclcpp::Node
{
public:
  using BTCommand = bt_mios_ros2_interface::action::CommandTest;
  using GoalHandleBTCommand = rclcpp_action::ServerGoalHandle<BTCommand>;
  
  explicit BTActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("behavior_tree_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<BTCommand>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "test_action",
      std::bind(&BTActionServer::handle_goal, this, _1, _2),
      std::bind(&BTActionServer::handle_cancel, this, _1),
      std::bind(&BTActionServer::handle_accepted, this, _1));
  };

private:

  rclcpp_action::Server<BTCommand>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const BTCommand::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleBTCommand> goal_handle);

  void mios_communication();

  

  void execute(const std::shared_ptr<GoalHandleBTCommand> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleBTCommand> goal_handle);

};
// TODO multiple def of main function
// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);

//   auto action_server = std::make_shared<BTActionServer>();

//   rclcpp::spin(action_server);

//   rclcpp::shutdown();
//   return 0;
// }
