#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cinttypes>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/node_list.hpp"
#include "ws_client/ws_client.hpp"

#include "bt_mios_ros2_interface/srv/request_state.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class BTRos2Node : public rclcpp::Node
{
public:
    BTRos2Node()
        : Node("bt_ros2_node")
    {
        //* initialize the callback groups
        client_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&BTRos2Node::timer_callback, this), timer_callback_group_);
        // * initilize the client
        // m_client_ptr = this->create_client<bt_mios_ros2_interface::srv::RequestState>(
        //     "request_state", rmw_qos_profile_services_default, client_callback_group_);
        // TODO the web_socket receiver of the message from the server:
        m_add_two_ints_ptr = this->create_client<example_interfaces::srv::AddTwoInts>(
            "add_two_ints", rmw_qos_profile_services_default, client_callback_group_);
    }

private:
    // callback group
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    // srv client
    // rclcpp::Client<bt_mios_ros2_interface::srv::RequestState>::SharedPtr m_client_ptr;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr m_add_two_ints_ptr;
    rclcpp::TimerBase::SharedPtr timer_;

    void add_two_ints_request()
    {
        while (!m_add_two_ints_ptr->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting.");
                break;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 1;
        request->b = 2;
        auto result_future = m_add_two_ints_ptr->async_send_request(request);
        result_future.wait();
        RCLCPP_INFO(this->get_logger(), "the result is: %ld.\n", result_future.get()->sum);

        // std::future_status status = result_future.wait_for(std::chrono::milliseconds(1000));
        // if (status == std::future_status::ready)
        // {
        //     RCLCPP_INFO(this->get_logger(), "the result is: %ld.\n", result_future.get()->sum);
        // }
        // else if (status == std::future_status::deferred)
        // {
        //     RCLCPP_INFO(this->get_logger(), "deferred.");
        // }
        // else if (status == std::future_status::timeout)
        // {
        //     RCLCPP_INFO(this->get_logger(), "PASSSSSSSSSSSSSS.");
        // }

        // if (result_future.valid())
        // {
        //     RCLCPP_INFO(this->get_logger(), "test: valid.");
        //     auto result = result_future.get();
        //     RCLCPP_INFO(this->get_logger(), "test: get finished.");
        //     RCLCPP_INFO(this->get_logger(), "result handling: the result = %i\n.", result->sum);

        // }
        // RCLCPP_INFO(this->get_logger(), "test: not valid.");
        // result_future.wait_for(std::chrono::milliseconds(500));
        // if (result_future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
        // {
        //     handle_service_result(result_future);
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "Service call timed out!");
        // }
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Sending request");
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        auto result_future = m_add_two_ints_ptr->async_send_request(request);
        std::future_status status = result_future.wait_for(10s); // timeout to guarantee a graceful finish
        if (status == std::future_status::ready)
        {
            RCLCPP_INFO(this->get_logger(), "Received response: %ld\n", result_future.get()->sum);
        }
        // add_two_ints_request();
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
    rclcpp::shutdown();
    return 0;
}

// using AddTwoInts = example_interfaces::srv::AddTwoInts;

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("minimal_client");
//     auto client = node->create_client<AddTwoInts>("add_two_ints");
//     while (!client->wait_for_service(std::chrono::seconds(1)))
//     {
//         if (!rclcpp::ok())
//         {
//             RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
//             return 1;
//         }
//         RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
//     }
//     auto request = std::make_shared<AddTwoInts::Request>();
//     request->a = 41;
//     request->b = 1;
//     auto result_future = client->async_send_request(request);
//     if (rclcpp::spin_until_future_complete(node, result_future) !=
//         rclcpp::FutureReturnCode::SUCCESS)
//     {
//         RCLCPP_ERROR(node->get_logger(), "service call failed :(");
//         client->remove_pending_request(result_future);
//         return 1;
//     }
//     auto result = result_future.get();
//     RCLCPP_INFO(
//         node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
//         request->a, request->b, result->sum);
//     rclcpp::shutdown();
//     return 0;
// }