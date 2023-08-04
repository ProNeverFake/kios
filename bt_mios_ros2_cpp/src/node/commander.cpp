
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "ws_client/ws_client.hpp"

using std::placeholders::_1;

class Commander : public rclcpp::Node
{
public:
    Commander()
        : Node("commander"),
          ws_url("ws://localhost:12000/mios/core"),
          udp_ip("localhost"),
          udp_port(12346)
    {
        // * initialize the websocket messenger
        m_messenger = std::make_shared<BTMessenger>(ws_url);
        // websocket connection
        m_messenger->connect();
        // register the udp subscriber
        mios_register_udp();
        // * set the grasped object
        m_messenger->send_grasped_object();
        // subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
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
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr is_update_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

    // callbacks
    rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr m_is_update_client_ptr;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<bt_mios_ros2_interface::msg::RobotState>::SharedPtr subscription_;

    // ws_client rel
    std::shared_ptr<BTMessenger> m_messenger;
    std::string ws_url;

    // udp subscriber ip
    std::string udp_ip;
    int udp_port;
    bool is_update;

    void topic_callback(const std_msgs::msg::String &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    /**
     * @brief set the param "is_update" in node bt_udp_node as true
     * TODO make a generic set param method
     *
     */
    void start_update_state()
    {
        if (is_update == false)
        {
            auto parameter = rcl_interfaces::msg::Parameter();
            auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

            parameter.name = "is_update";
            parameter.value.type = 1;          //  bool = 1,    int = 2,        float = 3,     string = 4
            parameter.value.bool_value = true; // .bool_value, .integer_value, .double_value, .string_value

            request->parameters.push_back(parameter);

            while (!m_is_update_client_ptr->wait_for_service(std::chrono::milliseconds(500)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "service is_update not available, waiting again...");
            }
            // !!!! CHECK
            auto result_future = m_is_update_client_ptr->async_send_request(request);
            std::future_status status = result_future.wait_for(
                std::chrono::milliseconds(50));
            if (status == std::future_status::ready)
            {
                auto result = result_future.get();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "is update: response timed out!");
            }
            is_update = true;
        }
        else
        {
            // is update == true, do nothing
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto commander = std::make_shared<Commander>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(commander);

    executor.spin();

    // * unregister the udp before shutdown.
    commander->mios_unregister_udp();
    rclcpp::shutdown();
    return 0;
}
