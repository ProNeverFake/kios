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

#include "bt_mios_ros2_interface/srv/request_state.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class BTRos2Node : public rclcpp::Node
{
public:
    BTRos2Node()
        : Node("bt_ros2_node"),
          //   m_tree_root(tree_root),
          // ? localhost invalid?
          ws_url("ws://localhost:12000/mios/core"),
          udp_ip("127.0.0.1"),
          m_client_ptr(nullptr)
    {
        // * initialize the tree_root
        m_tree_root = std::make_shared<Insertion::TreeRoot>();
        // * initialize the websocket messenger
        m_messenger = std::make_shared<BTMessenger>(ws_url);
        // websocket connection
        m_messenger->connect();
        // register the udp subscriber
        mios_register_udp();
        // the ros spin method:
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BTRos2Node::timer_callback, this));
        // * set the grasped object
        m_messenger->send_grasped_object();
        // set parameter of bt_udp_node to start context and state update
        start_update_state();
        // * initilize the client
        m_client_ptr = this->create_client<bt_mios_ros2_interface::srv::RequestState>("request_state");
        // TODO the web_socket receiver of the message from the server:
        // m_messenger->
    }

private:
    // srv client
    rclcpp::Client<bt_mios_ros2_interface::srv::RequestState>::SharedPtr m_client_ptr;

    rclcpp::TimerBase::SharedPtr timer_;

    // ws_client rel
    std::shared_ptr<BTMessenger> m_messenger;
    std::string ws_url;

    // udp subscriber ip
    std::string udp_ip;

    // behavior tree rel
    std::shared_ptr<Insertion::TreeRoot> m_tree_root;
    BT::NodeStatus tick_result;
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
        auto request = std::make_shared<bt_mios_ros2_interface::srv::RequestState::Request>();
        request->object = "state";
        auto result_future = m_client_ptr->async_send_request(request);
        if (result_future.wait_for(std::chrono::milliseconds(5)) == std::future_status::ready) // wait for the result for 5 seconds
        {
            handle_service_result(result_future);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Service call timed out!");
        }
    }

    void handle_service_result(rclcpp::Client<bt_mios_ros2_interface::srv::RequestState>::SharedFuture result_future)
    {
        auto result = result_future.get();
        m_tree_root->get_state_ptr()->TF_F_ext_K = result->tf_f_ext_k;
    }

    /**
     * @brief set the param "is_update" in node bt_udp_node as true
     * TODO make a generic set param method
     *
     */
    void start_update_state()
    {
        auto parameter = rcl_interfaces::msg::Parameter();
        auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

        std::string service_name = "bt_udp_node/set_parameters_atomically";
        auto client = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(service_name);

        parameter.name = "is_update";
        parameter.value.type = 1;          //  bool = 1,    int = 2,        float = 3,     string = 4
        parameter.value.bool_value = true; // .bool_value, .integer_value, .double_value, .string_value

        request->parameters.push_back(parameter);

        while (!client->wait_for_service(std::chrono::milliseconds(10)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service %s not available, waiting again...", service_name.c_str());
        }
        auto result = client->async_send_request(request);
    }

    void mios_register_udp()
    {
        m_messenger->register_udp();
    }

    void mios_interrupt()
    // ! MOVE TO MESSENGER
    {
        // nlohmann::json payload;
        // payload["ip"] = udp_ip;
        // payload["port"] = udp;
        // payload["subscribe"] = {"tau_ext", "q", "TF_F_ext_K"};
        // if (m_messenger.check_connection())
        // {
        //     m_messenger.send("subscribe_telemetry", payload);
        // }
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
        // * update the state
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
                // ! m_messenger->stop_task();
                // * use wait request
                // * send new context
                m_messenger->start_task(m_tree_root->get_context_ptr()->parameter);
                // * use wait request
            }
            else
            {
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

    rclcpp::spin(bt_ros2_node);
    rclcpp::shutdown();
    return 0;
}