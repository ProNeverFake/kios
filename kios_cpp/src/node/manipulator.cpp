#include <map>
#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "kios_communication/ws_client.hpp"

using std::placeholders::_1;

class Manipulator : public rclcpp::Node
{
public:
    Manipulator()
        : Node("manipulator")
    {
        client_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        // * initialize the clients
        // m_is_update_client_ptr = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(
        //     "/bt_udp_node/set_parameters_atomically",
        //     rmw_qos_profile_services_default,
        //     client_callback_group_);
        // * initialize the timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Manipulator::timer_callback, this),
            timer_callback_group_);

        // * initialize the server
        // subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr server_callback_group_;

    // callbacks
    std::map<std::string, rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr> client_map;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "time hit.\n");
    }

    /**
     * @brief set the
     *
     */
    void node_on(std::string &node_name)
    {
        if (client_map.find(node_name) != client_map.end())
        {
            auto client_ptr = client_map[node_name];
            auto parameter = rcl_interfaces::msg::Parameter();
            auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

            parameter.name = "on";
            parameter.value.type = 1;          //  bool = 1,    int = 2,        float = 3,     string = 4
            parameter.value.bool_value = true; // .bool_value, .integer_value, .double_value, .string_value

            request->parameters.push_back(parameter);

            if (!client_ptr->wait_for_service(std::chrono::milliseconds(500)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "node_on: set parameter service is_update not available!");
            }
            else
            {
                auto result_future = client_ptr->async_send_request(request);
                std::future_status status = result_future.wait_for(std::chrono::milliseconds(500));
                if (status == std::future_status::ready)
                {
                    auto result = result_future.get();
                    // * handle the result.
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "node_on: response timed out!");
                }
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "node_on: client for node %s not found!\n", node_name.c_str());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto manipulator = std::make_shared<Manipulator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(manipulator);

    executor.spin();

    // * unregister the udp before shutdown.
    rclcpp::shutdown();
    return 0;
}
