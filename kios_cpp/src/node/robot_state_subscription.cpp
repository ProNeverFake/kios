#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "ws_client/ws_client.hpp"

#include "kios_interface/msg/robot_state.hpp"

using std::placeholders::_1;

class BTRos2Node : public rclcpp::Node
{
public:
    BTRos2Node()
        : Node("bt_ros2_node"),
          ws_url("ws://localhost:12000/mios/core"),
          udp_ip("127.0.0.1"),
          udp_port(12346),
          is_update(false)
    {
        //* initialize the callback groups
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        is_update_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // * initialize the websocket messenger
        m_messenger = std::make_shared<BTMessenger>(ws_url);
        // websocket connection
        m_messenger->connect();

        // register the udp subscriber
        mios_register_udp();

        // * set the grasped object
        m_messenger->send_grasped_object();

        // * initilize the client
        m_is_update_client_ptr = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(
            "/bt_udp_node/set_parameters_atomically",
            rmw_qos_profile_services_default,
            is_update_callback_group_);

        // * initialize the subscription
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

        subscription_ = this->create_subscription<kios_interface::msg::RobotState>(
            "mios_state_topic",
            qos,
            std::bind(&BTRos2Node::subscription_callback, this, _1));
    }

    void mios_register_udp()
    {
        m_messenger->register_udp(udp_port);
    }

    void mios_unregister_udp()
    {
        m_messenger->unregister_udp();
    }

private:
    // callback group
    rclcpp::CallbackGroup::SharedPtr is_update_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

    // callbacks
    rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr m_is_update_client_ptr;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<kios_interface::msg::RobotState>::SharedPtr subscription_;

    // ws_client rel
    std::shared_ptr<BTMessenger> m_messenger;
    std::string ws_url;

    // udp subscriber ip
    std::string udp_ip;
    int udp_port;
    bool is_update;

    void subscription_callback(const kios_interface::msg::RobotState::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "subscription hit.");
        RCLCPP_INFO(this->get_logger(), "subscription listened: %f.", msg->tf_f_ext_k[2]);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto bt_ros2_node = std::make_shared<BTRos2Node>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(bt_ros2_node);

    executor.spin();

    // * unregister the udp before shutdown.
    bt_ros2_node->mios_unregister_udp();
    rclcpp::shutdown();
    return 0;
}