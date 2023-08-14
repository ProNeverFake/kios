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

#include "kios_interface/srv/get_object_request.hpp"

using std::placeholders::_1;

class TreeNode : public rclcpp::Node
{
public:
    TreeNode()
        : Node("tree_node"),
          is_running(true)
    {
        // initialize object list
        object_list_.push_back("contact");
        object_list_.push_back("approach");

        // declare mission parameter
        this->declare_parameter("is_update_object", true);
        this->declare_parameter("is_mission_success", false);
        this->declare_parameter("power_on", true);
        //* initialize the callback groups
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        publisher_callback_group_ = timer_callback_group_;
        client_callback_group_ = timer_callback_group_;

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
        get_object_client_ = this->create_client<kios_interface::srv::GetObjectRequest>(
            "get_object_service",
            rmw_qos_profile_services_default,
            client_callback_group_);
    }

private:
    // flags
    bool is_running;

    // object list
    std::vector<std::string> object_list_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr publisher_callback_group_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<kios_interface::msg::TaskState>::SharedPtr subscription_;
    rclcpp::Publisher<kios_interface::msg::TreeState>::SharedPtr publisher_;
    rclcpp::Client<kios_interface::srv::GetObjectRequest>::SharedPtr get_object_client_;

    // behavior tree rel
    std::shared_ptr<Insertion::TreeRoot>
        m_tree_root;
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
        // * get_object_service: update the objects
        if (this->get_parameter("is_update_object").as_bool() == true)
        {
            // * send request to update the object
            auto request = std::make_shared<kios_interface::srv::GetObjectRequest::Request>();
            request->object_list = object_list_;
            while (!get_object_client_->wait_for_service(std::chrono::milliseconds(50)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service get_object_service not available, waiting ...");
            }
            auto result_future = get_object_client_->async_send_request(request);
            std::future_status status = result_future.wait_until(
                std::chrono::steady_clock::now() + std::chrono::seconds(1));
            if (status == std::future_status::ready)
            {
                auto result = result_future.get();
                if (result->is_success == true)
                {
                    RCLCPP_INFO(this->get_logger(), "get_object_service: Service call succeeded.");
                    try
                    {
                        nlohmann::json object_data = nlohmann::json::parse(result->object_data);
                    }
                    catch (...)
                    {
                        RCLCPP_FATAL(this->get_logger(), "get_object_service: ERROR IN JSON FILE PARSING!");
                        // * BBDEBUG: SHUTDOWN
                        rclcpp::shutdown();
                    }
                    // TODO handle the object_data
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "get_object_service: Service call failed! Error message: %s", result->error_message);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "get_object_service: Service call timed out!");
            }

            // * reset the flag in node param
            std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("is_update_object", false)};
            this->set_parameters(all_new_parameters);
        }
        // * tick the tree
        if (is_running)
        {
            tick_result = m_tree_root->tick_once();
            RCLCPP_INFO(this->get_logger(), "Tick tree once.\n");
            // * check tick_result
            if (is_tree_running())
            {
                // * go ahead
                RCLCPP_INFO(this->get_logger(), "Tree state: RUNNING.\n");

                // * publish tree state
                kios_interface::msg::TreeState msg;
                msg.action_name = m_tree_root->get_context_ptr()->action_name;
                msg.action_phase = static_cast<int32_t>(m_tree_root->get_context_ptr()->action_phase);
                msg.is_runnning = true;
                RCLCPP_INFO(this->get_logger(), "Tree action node name: %s.\n", msg.action_name.c_str());
                publisher_->publish(msg);
            }
            else
            {
                // RCLCPP_INFO(this->get_logger(), "Tree .\n");
                // * stop
                is_running = false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "the tree has been finished!");
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