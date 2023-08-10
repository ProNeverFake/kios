#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
// #include "std_msgs/msg/string.hpp"

#include "behavior_tree/node_list.hpp"
#include "ws_client/ws_client.hpp"

// #include "kios_interface/srv/request_state.hpp"
#include "kios_interface/msg/robot_state.hpp"

class BTRos2Node : public rclcpp::Node
{
public:
    BTRos2Node()
        : Node("bt_ros2_node"),
          ws_url("ws://localhost:12000/mios/core"),
          udp_ip("127.0.0.1"),
          is_update(false)
    {
        //* initialize the callback groups
        client_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        is_update_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        // * initialize the tree_root
        m_tree_root = std::make_shared<Insertion::TreeRoot>();
        // * initialize the websocket messenger
        m_messenger = std::make_shared<BTMessenger>(ws_url);
        // websocket connection
        m_messenger->connect();

        // register the udp subscriber
        mios_register_udp();

        // * set the grasped object
        m_messenger->send_grasped_object();

        // the ros spin method:
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&BTRos2Node::timer_callback, this), timer_callback_group_);
        // * state the update of the udp
        // * initilize the client
        m_client_ptr = this->create_client<kios_interface::srv::RequestState>(
            "request_state",
            rmw_qos_profile_services_default,
            client_callback_group_);
        m_is_update_client_ptr = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(
            "/bt_udp_node/set_parameters_atomically",
            rmw_qos_profile_services_default,
            is_update_callback_group_);

        // TODO the web_socket receiver of the message from the server:
    }

private:
    // callback group
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr is_update_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscriber_callback_group_;

    // srv client
    // ! discarded service!!
    // rclcpp::Client<kios_interface::srv::RequestState>::SharedPtr m_client_ptr;
    rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr m_is_update_client_ptr;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<kios_interface::msg::RobotState>::SharedPtr subscription_;

    // ws_client rel
    std::shared_ptr<BTMessenger> m_messenger;
    std::string ws_url;

    // udp subscriber ip
    std::string udp_ip;

    bool is_update;

    // behavior tree rel
    std::shared_ptr<Insertion::TreeRoot> m_tree_root;
    BT::NodeStatus tick_result;
    // TODO service not stable. change it to action
    void update_state_context()
    {
        while (!m_client_ptr->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting.");
                break;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        }
        auto request = std::make_shared<kios_interface::srv::RequestState::Request>();
        request->object = "state";
        auto result_future = m_client_ptr->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "start to wait for the service.");

        std::future_status status = result_future.wait_for(
            std::chrono::milliseconds(20));
        if (status == std::future_status::ready)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "test: get finished.");
            m_tree_root->get_state_ptr()->TF_F_ext_K = result->tf_f_ext_k;
            RCLCPP_INFO(this->get_logger(), "result handling: the result = %f\n.", result->tf_f_ext_k[2]);
            // handle_service_result(result_future);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Service call timed out!");
        }
    }

    // void handle_service_result(rclcpp::Client<kios_interface::srv::RequestState>::SharedFuture result_future)
    // {
    //     auto result = result_future.get();
    //     RCLCPP_INFO(this->get_logger(), "test: get finished.");
    //     m_tree_root->get_state_ptr()->TF_F_ext_K = result->tf_f_ext_k;
    //     RCLCPP_INFO(this->get_logger(), "result handling: the result = %d\n.", result->tf_f_ext_k[2]);
    // }

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

    void mios_register_udp()
    {
        m_messenger->register_udp();
    }

    void mios_unregister_udp()
    {
        m_messenger->unregister_udp();
    }
    /**
     * @brief check the tree state
     *
     * @return true
     * @return false
     */
    bool check_tick_result()
    {
        switch (tick_result)
        {
        case BT::NodeStatus::RUNNING: {
            return true;
        };
        case BT::NodeStatus::SUCCESS: {
            return false;
        };
        default: {
            return false;
        }
        }
    }
    /**
     * @brief
     *
     */
    void timer_callback()
    {
        // * make the udp node start to get pkg
        start_update_state();
        // * update the state with client
        update_state_context();
        // * get command context from the tree by tick it
        tick_result = m_tree_root->tick_once();
        RCLCPP_INFO(this->get_logger(), "Tick once.\n");
        // * check tick_result
        if (check_tick_result())
        {
            // * go ahead
            RCLCPP_INFO(this->get_logger(), "RUNNING.\n");

            // * check if action changed
            if (m_tree_root->is_action_switch())
            {
                // * stop the current task
                m_messenger->stop_task();
                // * use wait request
                // * send new context
                m_messenger->start_task(m_tree_root->get_context_ptr()->parameter);
                // * use wait request
                RCLCPP_INFO(this->get_logger(), "Action Switched.\n");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Action Running.\n");
                // do nothing
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Action succeeds.\n");
            // * stop
        }
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