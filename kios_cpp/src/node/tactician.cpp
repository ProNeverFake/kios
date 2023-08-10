#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nlohmann/json.hpp"

#include "behavior_tree/node_list.hpp"

#include "kios_interface/msg/task_state.hpp"
#include "kios_interface/msg/skill_context.hpp"
#include "kios_interface/msg/tree_state.hpp"

#include "kios_interface/srv/command_request.hpp"

#include "kios_utils/kios_utils.hpp"

using std::placeholders::_1;

class Tactician : public rclcpp::Node
{
public:
    Tactician()
        : Node("tactician"),
          is_running(true),
          is_switch_action_phase(false),
          is_busy(false),
          power_on(true)
    {
        // declare mission parameter
        this->declare_parameter("power_on", true);

        //* initialize the callback groups
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        client_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // * initialize the objects
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Tactician::timer_callback, this),
            timer_callback_group_);

        // * initilize the client
        client_ = this->create_client<kios_interface::srv::CommandRequest>(
            "command_request_service",
            rmw_qos_profile_services_default,
            client_callback_group_);
        // * initialize the subscription
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = subscription_callback_group_;

        // * initialize the callbacks
        task_state_subscription_ = this->create_subscription<kios_interface::msg::TaskState>(
            "task_state_topic",
            qos,
            std::bind(&Tactician::task_subscription_callback, this, _1),
            subscription_options);

        tree_state_subscription_ = this->create_subscription<kios_interface::msg::TreeState>(
            "tree_state_topic",
            qos,
            std::bind(&Tactician::tree_subscription_callback, this, _1),
            subscription_options);
    }

private:
    // flags
    bool power_on;
    bool is_running;
    bool is_switch_action_phase;
    bool is_busy;

    // ! TEMP STATE
    kios::TaskState task_state;
    kios::ThreadSafeData<kios::TreeState> ts_tree_state_;
    // Insertion::ActionNodeContext action_node_context_;

    kios::ActionPhaseContext action_phase_context_;
    kios::CommandRequest command_request_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    rclcpp::Client<kios_interface::srv::CommandRequest>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<kios_interface::msg::TreeState>::SharedPtr tree_state_subscription_;
    rclcpp::Subscription<kios_interface::msg::TaskState>::SharedPtr task_state_subscription_;

    void task_subscription_callback(const kios_interface::msg::TaskState::SharedPtr msg)
    {
        task_state.tf_f_ext_k = msg->tf_f_ext_k;
        RCLCPP_INFO(this->get_logger(), "task subscription listened: %f.", msg->tf_f_ext_k);
    }

    void tree_subscription_callback(const kios_interface::msg::TreeState::SharedPtr msg)
    {
        if (is_busy == false)
        {
            RCLCPP_INFO(this->get_logger(), "tree_subscription hit.");

            kios::TreeState tree_state_temp_ = ts_tree_state_.read_data();
            tree_state_temp_.action_phase = static_cast<kios::ActionPhase>(msg->action_phase);
            tree_state_temp_.is_running = msg->is_runnning;

            // if action phase switched
            if (tree_state_temp_.action_phase != tree_state_temp_.last_action_phase)
            {
                // update the last action phase
                tree_state_temp_.last_action_phase = tree_state_temp_.action_phase;
                // flag to start a command request
                is_switch_action_phase = true;
            }
            else
            {
                // pass
            }
            // * update tree state in node
            ts_tree_state_.write_data(tree_state_temp_);
        }
        else
        {
            // pass
        }
        // update tree state
    }

    void update_command_request()
    {
        kios::TreeState tree_state_temp_ = ts_tree_state_.read_data();

        command_request_.command_type = kios::CommandType::STOP_OLD_START_NEW;
        // * handle the command context here
        command_request_.command_context["skill"]["action_context"]["action_name"] = tree_state_temp_.action_name;
        command_request_.command_context["skill"]["action_context"]["action_phase"] = tree_state_temp_.action_phase;
    }

    /**
     * @brief tick the tree and publish the context
     *
     */
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "The value of is_running as an integer is: %d", is_running);
        // std::cout << "DEBUG: IS_RUNNING: " << is_running << std::endl;
        if (is_running == true)
        {
            if (is_switch_action_phase == true)
            {
                is_busy = true;
                // update context
                update_command_request();
                auto request = std::make_shared<kios_interface::srv::CommandRequest::Request>();
                request->command_type = static_cast<int32_t>(command_request_.command_type);
                request->command_context = command_request_.command_context.dump();
                // client send request
                while (!client_->wait_for_service(std::chrono::seconds(1)))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                        rclcpp::shutdown();
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                }

                auto result_future = client_->async_send_request(request);
                std::future_status status = result_future.wait_until(
                    std::chrono::steady_clock::now() + std::chrono::seconds(5));
                if (status == std::future_status::ready)
                {
                    auto result = result_future.get();
                    if (result->is_accepted == true)
                    {
                        RCLCPP_INFO(this->get_logger(), "Command accepted.\n");
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Command refused!\n");
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call timed out!\n");
                }

                is_busy = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Timer: continue the last action phase.");
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Timer: not running.\n");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto tactician = std::make_shared<Tactician>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(tactician);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}