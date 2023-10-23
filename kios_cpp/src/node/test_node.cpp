#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "kios_interface/msg/mios_state.hpp"
#include "kios_interface/msg/task_state.hpp"
#include "kios_interface/msg/sensor_state.hpp"

#include "kios_utils/kios_utils.hpp"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <optional>

using std::placeholders::_1;

class TestNode : public rclcpp::Node
{
public:
    TestNode()
        : Node("test_node"),
          context_clerk_()

    {
        this->declare_parameter("power", true);

        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        //* initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TestNode::timer_callback, this),
            timer_callback_group_);
    }

    bool check_power()
    {
        return this->get_parameter("power").as_bool();
    }

private:
    // callback group
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    kios::DefaultActionContext memory_test;
    kios::ContextClerk context_clerk_;

    // callbacks
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Test Loop.");
        auto str = kios::action_phase_to_str(kios::ActionPhase::INITIALIZATION);
        RCLCPP_INFO_STREAM(this->get_logger(), "the string is " << str.value() << " ! ");
        RCLCPP_INFO_STREAM(this->get_logger(), "the value is " << memory_test.get_default_context(kios::ActionPhase::CARTESIAN_MOVE).value() << " ! ");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto test_node = std::make_shared<TestNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}