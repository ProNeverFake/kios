#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "nlohmann/json.hpp"

#include "behavior_tree/tree_root.hpp"

#include "kios_interface/msg/task_state.hpp"
#include "kios_interface/msg/tree_state.hpp"

#include "kios_interface/srv/command_request.hpp"
#include "kios_interface/srv/switch_action_request.hpp"

#include "kios_utils/kios_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Tactician : public rclcpp::Node
{
public:
    Tactician()
        : Node("tactician"),
          isSwitchAction(false),
          isBusy(false),
          command_context_(),
          tree_state_(),
          task_state_()
    {
        std::cout << "start initialization" << std::endl;
        // declare mission parameter
        this->declare_parameter("power", true);

        //* initialize the callback groups
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        client_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        server_callback_group_ = client_callback_group_;

        // * initilize the client
        command_client_ = this->create_client<kios_interface::srv::CommandRequest>(
            "command_request_service",
            rmw_qos_profile_services_default,
            client_callback_group_);

        //*
        switch_action_server_ = this->create_service<kios_interface::srv::SwitchActionRequest>(
            "switch_action_service",
            std::bind(&Tactician::switch_action_server_callback, this, _1, _2),
            rmw_qos_profile_services_default,
            server_callback_group_);

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

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Tactician::timer_callback, this),
            timer_callback_group_);

        std::cout << "finish initialization" << std::endl;
    }

    bool check_power()
    {
        return this->get_parameter("power").as_bool();
    }

    void switch_power(bool turn_on)
    {
        std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("power", turn_on)};
        this->set_parameters(all_new_parameters);
        if (turn_on)
        {
            RCLCPP_INFO(this->get_logger(), "switch_power: turn on.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "switch_power: turn off.");
        }
    }

private:
    // thread safe rel
    std::mutex tree_state_mtx_;
    std::mutex task_state_mtx_;

    // flags
    std::atomic_bool isSwitchAction;
    std::atomic_bool isBusy; // this is necessary because of the timer callback group setting

    // state of tree and task
    kios::TaskState task_state_;
    kios::TreeState tree_state_;

    kios::CommandContext command_context_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr server_callback_group_;

    rclcpp::Service<kios_interface::srv::SwitchActionRequest>::SharedPtr switch_action_server_;
    rclcpp::Client<kios_interface::srv::CommandRequest>::SharedPtr command_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<kios_interface::msg::TaskState>::SharedPtr task_state_subscription_;

    /**
     * @brief subcriber callback. update member variable task_state (percept).
     *
     * @param msg
     */
    void task_subscription_callback(kios_interface::msg::TaskState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> task_state_guard(task_state_mtx_);
        RCLCPP_INFO(this->get_logger(), "SUB HIT, try to move");
        if (msg->tf_f_ext_k.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "WHY IS THE MSG EMPTY???");
        }
        // std::stringstream ss;

        // for (size_t i = 0; i < msg->tf_f_ext_k.size(); ++i)
        // {
        //     ss << msg->tf_f_ext_k[i];
        //     if (i != msg->tf_f_ext_k.size() - 1)
        //     { // if not the last element
        //         ss << ", ";
        //     }
        // }
        // std::string str = ss.str();
        // std::cout << str << std::endl;
        // std::cout << "PRINT TEST: " << msg->tf_f_ext_k << std::endl;
        // double test_number = msg->tf_f_ext_k[2];
        // RCLCPP_INFO_STREAM(this->get_logger(), "task subscription listened: " << test_number);
        // RCLCPP_INFO(this->get_logger(), "task subscription listened: %f", msg->tf_f_ext_k[2]);

        task_state_.tf_f_ext_k = std::move(msg->tf_f_ext_k);
    }

    /**
     * @brief switch action server callback. Response to the request and set flag for timer callback.
     *
     * @param request
     * @param response
     */
    void switch_action_server_callback(
        const std::shared_ptr<kios_interface::srv::SwitchActionRequest::Request> request,
        const std::shared_ptr<kios_interface::srv::SwitchActionRequest::Response> response)
    {
        if (check_power() == true)
        {
            if (isBusy.load())
            {
                RCLCPP_ERROR(this->get_logger(), "Node is busy, request refused!");
                response->is_accepted = false;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "SWITCH ACTION HIT!");

                // * update tree state
                std::lock_guard<std::mutex> lock(tree_state_mtx_);
                tree_state_.action_name = std::move(request->action_name);
                tree_state_.action_phase = std::move(static_cast<kios::ActionPhase>(request->action_phase));
                // * set flag for timer
                isSwitchAction.store(true);
            }
            RCLCPP_ERROR(this->get_logger(), "switch_action request accepted.");
            response->is_accepted = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "POWER OFF, request refused!");
            response->is_accepted = false;
        }
    }

    /**
     * @brief prereserved inline method to implement skill parameter generating algo.
     *
     */
    void generate_command_context()
    {
        // * lock task state (perception) and tree state
        std::lock_guard<std::mutex> task_state_guard(task_state_mtx_);
        std::lock_guard<std::mutex> tree_state_guard(tree_state_mtx_);

        /////////////////////////////////////////////
        // * HERE THE PART TO GENERATE SKILL PARAMETER AND UPDATE THE COMMAND CONTEXT
        // * now just copy.
        // ! only use stop old start new
        command_context_.command_type = kios::CommandType::STOP_OLD_START_NEW;
        // * handle the command context here
        command_context_.command_context["skill"]["action_context"]["action_name"] = tree_state_.action_name;
        command_context_.command_context["skill"]["action_context"]["action_phase"] = tree_state_.action_phase;

        /////////////////////////////////////////////
    }

    /**
     * @brief inline function for send the command request to commander
     *
     * @param ready_deadline max time to wait until server is ready
     * @param response_deadline max time to wait until response is ready
     * @return true
     * @return false
     */
    bool send_command_request(int ready_deadline = 50, int response_deadline = 50)
    {
        auto request = std::make_shared<kios_interface::srv::CommandRequest::Request>();
        request->command_type = static_cast<int32_t>(command_context_.command_type);
        request->command_context = command_context_.command_context.dump();

        // client send request
        while (!command_client_->wait_for_service(std::chrono::milliseconds(ready_deadline)))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Service " << command_client_->get_service_name() << " is not available after waiting for " << ready_deadline << " miliseconds!");
            return false;
        }
        auto result_future = command_client_->async_send_request(request);
        std::future_status status = result_future.wait_until(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(response_deadline));
        if (status == std::future_status::ready)
        {
            auto result = result_future.get();
            if (result->is_accepted == true)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Service " << command_client_->get_service_name() << " request accepted.");
                return true;
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Service " << command_client_->get_service_name() << " request refused!");
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "UNKNOWN ERROR: Service " << command_client_->get_service_name() << " is available but response is not ready after waiting for " << response_deadline << "miliseconds!");
            return false;
        }
    }

    /**
     * @brief timer callback. handle the switch action request and send the command request to commander.
     *
     */
    void timer_callback()
    {
        if (check_power() == true)
        {
            if (isSwitchAction.load() == true)
            {
                if (!isBusy.load())
                {
                    // * set busy
                    isBusy.store(true);
                    generate_command_context();
                    if (!send_command_request(1000, 1000))
                    {
                        //* error in command request service. turn off for debug.
                        switch_power(false);
                    }
                    // * command_request finished. reset flags.
                    isSwitchAction.store(false);
                    isBusy.store(false);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Timer: Node is busy now.");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Timer: Continue the last action phase.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "POWER OFF, Timer pass ...");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto tactician = std::make_shared<Tactician>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(tactician);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}