

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "behavior_tree/node_list.hpp"
#include "ws_client/ws_client.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class BTRos2Node : public rclcpp::Node
{
public:
    BTRos2Node(Insertion::TreeRoot &tree_root)
        : Node("bt_ros2_node"), m_tree_root(tree_root),
          ws_url("ws://localhost:12000/mios/core"),
          m_messenger(ws_url)
    {
        // websocket connection
        m_messenger.connect();
        // waiting time for the connection
        std::this_thread::sleep_for(std::chrono::seconds(5));
        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        //* the ros spin method:
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&BTRos2Node::timer_callback, this));
        // * the receiver of the message from the server:
    }

private:
    bool check_command_state();
    bool check_tick_result();
    bool check_action_context();
    nlohmann::json context_transform();
    void timer_callback()
    {
        // * get command context from the tree by tick it
        tick_result = m_tree_root.tick_once();
        RCLCPP_INFO(this->get_logger(), "Tick once.\n");
        // * check tick_result
        if (check_tick_result())
        {
            // * go ahead
        }
        else
        {
            // * stop
        }
        // * check action_context
        // * action_context --- json context
        m_context = context_transform();
        // * send json context
        m_messenger.send("dummy_skill", m_context);
        // *
        // auto message = std_msgs::msg::String();
        // message.data = "test_message";
        //  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // ws_client rel
    BTMessenger m_messenger;
    std::string ws_url;
    nlohmann::json m_context;

    // behavior tree rel
    Insertion::TreeRoot &m_tree_root;
    BT::NodeStatus tick_result;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    std::shared_ptr<BTRos2Node> bt_ros2_node = std::make_shared<BTRos2Node>(Insertion::tree_root);

    rclcpp::spin(bt_ros2_node);
    rclcpp::shutdown();
    return 0;
}