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

#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <optional>

struct TaskState
{
    std::vector<double> tf_f_ext_k;
};

// //! UNFINISHED
template <typename T>
class ThreadSafeData
{
private:
    T data_;
    std::mutex mtx;

public:
    void write_data(T const &new_data)
    {
        std::lock_guard<std::mutex> lock(mtx);
        data_ = new_data;
    }

    T read_data()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return data_;
    }
};

using std::placeholders::_1;

class Messenger : public rclcpp::Node
{
public:
    Messenger()
        : Node("messenger"),
          is_running(true),
          mios_state({0, 0, 0, 0, 0, 0})

    {
        //* initialize the callback groups
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        publisher_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        timer_callback_group_ = publisher_callback_group_;
        //* initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Messenger::timer_callback, this),
            timer_callback_group_);
        //* initialize ros pub sub options
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = subscription_callback_group_;
        rclcpp::PublisherOptions publisher_options;
        publisher_options.callback_group = publisher_callback_group_;
        //* initialize the pub sub callbacks
        subscription_ = this->create_subscription<kios_interface::msg::MiosState>(
            "mios_state_topic",
            qos,
            std::bind(&Messenger::subscription_callback, this, _1),
            subscription_options);
        publisher_ = this->create_publisher<kios_interface::msg::TaskState>(
            "task_state_topic",
            qos,
            publisher_options);
    }

private:
    //! TEMP_STATE
    std::vector<double> mios_state;
    // flag
    bool is_running;
    ThreadSafeData<TaskState> ts_task_state_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr publisher_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    // callbacks
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<kios_interface::msg::TaskState>::SharedPtr publisher_;
    rclcpp::Subscription<kios_interface::msg::MiosState>::SharedPtr subscription_;

    void subscription_callback(const kios_interface::msg::MiosState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "subscription hit.");
        //// TEST
        RCLCPP_INFO(this->get_logger(), "subscription listened: %f.", msg->tf_f_ext_k[2]);
        // ! TEMP READ
        TaskState task_state;
        task_state.tf_f_ext_k = msg->tf_f_ext_k;
        ts_task_state_.write_data(task_state);
        // mios_state = msg->tf_f_ext_k;
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "timer hit.\n");
        auto message = kios_interface::msg::TaskState();
        message.tf_f_ext_k = ts_task_state_.read_data().tf_f_ext_k;
        RCLCPP_INFO(this->get_logger(), "Publishing: task_state\n");
        publisher_->publish(message);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto messenger = std::make_shared<Messenger>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(messenger);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}