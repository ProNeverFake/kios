#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/node_list.hpp"
#include "ws_client/ws_client.hpp"

#include "bt_mios_ros2_interface/msg/robot_state.hpp"

using std::placeholders::_1;

class TreeNode : public rclcpp::Node
{
public:
    TreeNode()
        : Node("tree_node")
    {
        //* initialize the callback groups
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        is_update_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // * initialize the tree_root
        m_tree_root = std::make_shared<Insertion::TreeRoot>();

        // the ros spin method:
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(50),
        //     std::bind(&TreeNode::timer_callback, this),
        //     timer_callback_group_);

        // * initilize the client
        m_is_update_client_ptr = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(
            "/bt_udp_node/set_parameters_atomically",
            rmw_qos_profile_services_default,
            is_update_callback_group_);
        // * initialize the subscription
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        // ! applied new QoS

        subscription_ = this->create_subscription<bt_mios_ros2_interface::msg::RobotState>(
            "mios_state_topic",
            qos,
            std::bind(&TreeNode::subscription_callback, this, _1));
    }

private:
    // callback group
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr is_update_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

    // callbacks
    rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr m_is_update_client_ptr;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<bt_mios_ros2_interface::msg::RobotState>::SharedPtr subscription_;
    rclcpp::Publisher<bt_mios_ros2_interface::msg::SkillContext>::SharedPtr publisher_;

    // behavior tree rel
    std::shared_ptr<Insertion::TreeRoot>
        m_tree_root;
    BT::NodeStatus tick_result;

    void subscription_callback(const bt_mios_ros2_interface::msg::RobotState::SharedPtr msg) const
    {
        m_tree_root->get_state_ptr()->TF_F_ext_K = msg->tf_f_ext_k;
        RCLCPP_INFO(this->get_logger(), "subscription listened: %f.", msg->tf_f_ext_k[2]);
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

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "time hit.\n");
        // * make the udp node start to get pkg
        start_update_state();
        // // * get command context from the tree by tick it
        // tick_result = m_tree_root->tick_once();
        // RCLCPP_INFO(this->get_logger(), "Tick once.\n");
        // // * check tick_result
        // if (check_tick_result())
        // {
        //     // * go ahead
        //     RCLCPP_INFO(this->get_logger(), "RUNNING.\n");

        //     // * check if action changed
        //     if (m_tree_root->is_action_switch())
        //     {
        //         // * stop the current task
        //         m_messenger->stop_task();
        //         // * use wait request
        //         // * send new context
        //         m_messenger->start_task(m_tree_root->get_context_ptr()->parameter);
        //         // * use wait request
        //         RCLCPP_INFO(this->get_logger(), "Action Switched.\n");
        //     }
        //     else
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Action Running.\n");
        //         // do nothing
        //     }
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "Action succeeds.\n");
        //     // * stop
        // }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto bt_ros2_node = std::make_shared<TreeNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(bt_ros2_node);

    executor.spin();

    // * unregister the udp before shutdown.
    rclcpp::shutdown();
    return 0;
}