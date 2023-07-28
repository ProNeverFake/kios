#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

#include "behavior_tree/node_list.hpp"
#include "ws_client/ws_client.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class BTRos2Node : public rclcpp::Node
{
public:
    BTRos2Node(std::shared_ptr<Insertion::TreeRoot> tree_root)
        : Node("bt_ros2_node"), m_tree_root(tree_root),
          // ! COMPLETE
          ws_url("ws://localhost:12000/mios/core"),
          udp_ip(""),
          m_messenger(ws_url)
    {
        // websocket connection
        m_messenger.connect();
        // waiting time for the connection
        std::this_thread::sleep_for(std::chrono::seconds(5));
        // register the udp subscriber
        mios_register_udp();
        // the ros spin method:
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BTRos2Node::timer_callback, this));
        // TODO the web_socket receiver of the message from the server:
    }

private:
    void update_context()
    {
        // ! set the flag parameter ON
    }
    void mios_register_udp()
    {
        m_messenger.register_udp();
    }
    void mios_interrupt()
    // ! COMPLETE
    {
        nlohmann::json payload;
        payload["ip"] = "";
        payload["port"] = 0;
        payload["subscribe"] = {"tau_ext", "q", "TF_F_ext_K"};
        if (m_messenger.check_connection())
        {
            m_messenger.send("subscribe_telemetry", payload);
        }
    }
    void mios_unregister_udp()
    {
        m_messenger.unregister_udp();
        // nlohmann::json payload;
        // payload["ip"] = "localhost";
        // if (m_messenger.check_connection())
        // {
        //     m_messenger.send("unsubscribe_telemetry", payload);
        // }
    }
    bool check_tick_result()
    {
        if (tick_result == BT::NodeStatus::RUNNING)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool check_action_context()
    {
        return true;
    }
    // nlohmann::json context_transform();
    /**
     * @brief
     *
     */
    void timer_callback()
    {
        // * get command context from the tree by tick it
        tick_result = m_tree_root->tick_once();
        RCLCPP_INFO(this->get_logger(), "Tick once.\n");
        // // * check tick_result
        if (check_tick_result())
        {
            RCLCPP_INFO(this->get_logger(), "RUNNING.\n");
            // * go ahead
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "???? NOT RUNNING.\n");
            // * stop
        }
        // * check
        // * action_context --- json context
        // m_context = context_transform();
        // * send json context
        // m_messenger.send("dummy_skill", m_context);
        // *
        // auto message = std_msgs::msg::String();
        // message.data = "test_message";
        //  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // ws_client rel
    BTMessenger m_messenger;
    std::string ws_url;
    // udp subscriber ip
    std::string udp_ip;
    // nlohmann::json m_context;

    // behavior tree rel
    std::shared_ptr<Insertion::TreeRoot> m_tree_root;
    BT::NodeStatus tick_result;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto tree_root_ptr = std::make_shared<Insertion::TreeRoot>();
    auto bt_ros2_node = std::make_shared<BTRos2Node>(tree_root_ptr);

    rclcpp::spin(bt_ros2_node);
    rclcpp::shutdown();
    return 0;
}