#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// #include "rcl_interfaces/srv/set_parameters_atomically.hpp"
// #include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/node_list.hpp"

#include "kios_interface/msg/tree_state.hpp"
#include "kios_interface/msg/task_state.hpp"
#include "kios_interface/msg/skill_context.hpp"

using std::placeholders::_1;

class TreeNode : public rclcpp::Node
{
public:
    TreeNode()
        : Node("tree_node"),
          is_running(true)
    {
        // declare mission parameter
        this->declare_parameter("is_mission_success", false);
        this->declare_parameter("power_on", true);
        //* initialize the callback groups
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        publisher_callback_group_ = timer_callback_group_;

        // * initialize the tree_root
        m_tree_root = std::make_shared<Insertion::TreeRoot>();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TreeNode::timer_callback, this),
            timer_callback_group_);

        // ! QoS not verified
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = subscription_callback_group_;
        rclcpp::PublisherOptions publisher_options;
        publisher_options.callback_group = publisher_callback_group_;
        // * initialize the callbacks
        subscription_ = this->create_subscription<kios_interface::msg::TaskState>(
            "mios_state_topic",
            qos,
            std::bind(&TreeNode::subscription_callback, this, _1),
            subscription_options);
        publisher_ = this->create_publisher<kios_interface::msg::TreeState>(
            "tree_state_topic",
            qos,
            publisher_options);
    }

private:
    // flags
    bool is_running;

    // callback group
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr publisher_callback_group_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<kios_interface::msg::TaskState>::SharedPtr subscription_;
    rclcpp::Publisher<kios_interface::msg::TreeState>::SharedPtr publisher_;

    // behavior tree rel
    std::shared_ptr<Insertion::TreeRoot> m_tree_root;
    BT::NodeStatus tick_result;

    void subscription_callback(const kios_interface::msg::TaskState::SharedPtr msg) const
    {
        m_tree_root->get_state_ptr()->TF_F_ext_K = msg->tf_f_ext_k;
        RCLCPP_INFO(this->get_logger(), "subscription listened: %f.", msg->tf_f_ext_k[2]);
    }

    /**
     * @brief tick the tree and publish the context
     *
     */
    void timer_callback()
    {
        if (is_running)
        {
            // * tick the tree
            tick_result = m_tree_root->tick_once();
            RCLCPP_INFO(this->get_logger(), "Tick tree once.\n");
            // * check tick_result
            if (is_tree_running())
            {
                // * go ahead
                RCLCPP_INFO(this->get_logger(), "Tree state: RUNNING.\n");

                // * publish tree state
                kios_interface::msg::TreeState msg;
                msg.action_phase = static_cast<int32_t>(m_tree_root->get_context_ptr()->action_phase);
                msg.is_runnning = true;
                publisher_->publish(msg);
            }
            else
            {
                // RCLCPP_INFO(this->get_logger(), "Tree .\n");
                // * stop
                is_running = false;
            }
        }
    }

    /**
     * @brief check the tree state
     *
     * @return true
     * @return false
     */
    bool is_tree_running()
    {
        switch (tick_result)
        {
        case BT::NodeStatus::RUNNING: {
            return true;
        };
        case BT::NodeStatus::SUCCESS: {
            RCLCPP_INFO(this->get_logger(), "MISSION SUCCEEDS.");
            std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("is_mission_success", true)};
            this->set_parameters(all_new_parameters);
            return false;
        };
        default: {
            RCLCPP_ERROR(this->get_logger(), "UNDEFINED BEHAVIOR!\n");
            return false;
        }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto tree_node = std::make_shared<TreeNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(tree_node);

    executor.spin();

    // * unregister the udp before shutdown.
    rclcpp::shutdown();
    return 0;
}