
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nlohmann/json.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/tree_root.hpp"

#include "kios_communication/ws_client.hpp"

#include "kios_interface/srv/command_request.hpp"
#include "kios_interface/srv/teach_object_service.hpp"
#include "kios_interface/action/execute_skill.hpp"

#include "kios_utils/kios_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using ExecuteSkillAction = kios_interface::action::ExecuteSkill;
using GoalHandleExecuteSkill = rclcpp_action::ServerGoalHandle<ExecuteSkillAction>;

class Commander : public rclcpp::Node
{
public:
    explicit Commander(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("commander", options),
          ws_url("ws://localhost:12000/mios/core"),
          udp_ip("127.0.0.1"), // not used
          udp_port_(12346),
          isBusy(false),
          subscription_list_{"tau_ext", "q", "TF_F_ext_K", "system_time", "T_T_EE"}
    {
        // declare power parameter
        this->declare_parameter("power", true);

        // callback group
        service_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // * initialize the websocket messenger

        messenger_ = std::make_shared<BTMessenger>(ws_url);
        // websocket connection
        messenger_->special_connect();
        while (!messenger_->wait_for_open_connection(3))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_WARN(this->get_logger(), "USER SHUTDOWN DETECTED.");
                std::exit(EXIT_FAILURE); // exit immediately.
            }
            RCLCPP_INFO(this->get_logger(), "websocket connection not ready. Waiting for an open connection.");
        }

        // udp register
        mios_register_udp(udp_port_, subscription_list_);

        // * initialize service
        command_service_ = this->create_service<kios_interface::srv::CommandRequest>(
            "command_request_service",
            std::bind(&Commander::command_service_callback, this, _1, _2),
            rmw_qos_profile_services_default,
            service_callback_group_);

        // * CLI service
        teach_object_service_ = this->create_service<kios_interface::srv::TeachObjectService>(
            "teach_object_cli",
            std::bind(&Commander::teach_object_service_callback, this, _1, _2),
            rmw_qos_profile_services_default,
            service_callback_group_);

        // * action server
        this->execute_skill_action_server_ = rclcpp_action::create_server<ExecuteSkillAction>(
            this,
            "execute_skill_action",
            std::bind(&Commander::execute_skill_handle_goal, this, _1, _2),
            std::bind(&Commander::execute_skill_handle_cancel, this, _1),
            std::bind(&Commander::execute_skill_handle_accepted, this, _1));
    }

    // connection rel
    void mios_register_udp(int &udp_port, nlohmann::json sub_list)
    {
        messenger_->register_udp(udp_port, sub_list);
    }
    void mios_unregister_udp()
    {
        messenger_->unregister_udp();
    }

    void shut_down_connection()
    {
        messenger_->unregister_udp();
        messenger_->close();
    }

    bool check_power()
    {
        return this->get_parameter("power").as_bool();
    }

private:
    // flags
    bool isBusy;

    //! TEMP STATE
    kios::ActionPhaseContext action_phase_context_;
    kios::CommandRequest command_request_;

    nlohmann::json task_response_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    // callbacks
    rclcpp::Service<kios_interface::srv::CommandRequest>::SharedPtr command_service_;
    rclcpp::Service<kios_interface::srv::TeachObjectService>::SharedPtr teach_object_service_;

    rclcpp_action::Server<ExecuteSkillAction>::SharedPtr execute_skill_action_server_;

    // ws_client rel
    std::shared_ptr<BTMessenger> messenger_;
    std::string ws_url;

    // udp subscriber ip
    std::string udp_ip; // not used
    int udp_port_;
    nlohmann::json subscription_list_;

    ///////////////////////////////////////////////// ! ACTION SERVER   /////////////////////////////////////////////////
    rclcpp_action::GoalResponse execute_skill_handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ExecuteSkillAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request.");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse execute_skill_handle_cancel(
        const std::shared_ptr<GoalHandleExecuteSkill> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute_skill_handle_accepted(const std::shared_ptr<GoalHandleExecuteSkill> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&Commander::execute_skill_execute, this, _1), goal_handle}.detach();
    }

    // handle the goal here: execute the action and return executing
    void execute_skill_execute(const std::shared_ptr<GoalHandleExecuteSkill> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<ExecuteSkillAction::Feedback>();
        auto result = std::make_shared<ExecuteSkillAction::Result>();
        result->result = false;

        // * get command context
        try
        {
            command_request_.command_context = nlohmann::json::parse(goal->command_context);
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "SOMETHING WRONG WITH THE JSON PARSE!");
            result->error_code = kios_interface::action::ExecuteSkill::Result::NODE_ERROR;
            result->message = "SOMETHING WRONG WITH THE JSON PARSE!";
            goal_handle->abort(result);
            return;
        }
        command_request_.command_type = static_cast<kios::CommandType>(goal->command_type);
        command_request_.skill_type = goal->skill_type;

        // * issue command
        switch (command_request_.command_type)
        {
        case kios::CommandType::STOP_OLD_START_NEW: {
            RCLCPP_INFO(this->get_logger(), "Issuing command: stop old start new...");
            // * future for trace
            std::promise<nlohmann::json> result_promise;
            std::future<nlohmann::json> result_future = result_promise.get_future();
            std::atomic_bool isInterrupted(false);
            // * stop the old task first
            if (!stop_task_request())
            {
                // ! failed
                RCLCPP_ERROR(this->get_logger(), "Issuing command: Stop task failed!!!");
                result->error_code = kios_interface::action::ExecuteSkill::Result::MIOS_ERROR;
                goal_handle->abort(result);
                return;
            }

            if (start_task_request(command_request_) == false)
            {
                // ! failed
                RCLCPP_ERROR(this->get_logger(), "Issuing command: Start task failed!!!");
                result->error_code = kios_interface::action::ExecuteSkill::Result::MIOS_ERROR;
                goal_handle->abort(result);
                return;
            }

            // * start a new thread to tun wait for task
            // ? where is the task uuid?
            auto wait_thread = std::thread(&Commander::wait_for_task_result, this, task_response_["result"]["task_uuid"], std::ref(result_promise));
            wait_thread.detach();

            // * wait for result in this thread
            while (result_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout) //websocket result
            {
                // * Check if there is a cancel request
                if (goal_handle->is_canceling())
                {
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    // notify the thread to stop
                    isInterrupted.store(true);
                    // * stop the action
                    result->result = true;
                    result->error_code = kios_interface::action::ExecuteSkill::Result::CANCELLED;
                    goal_handle->canceled(result);
                    return;
                }

                // * Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback");
            }

            // Check if goal is done
            if (rclcpp::ok())
            {
                result->result = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
            return;
        }
        case kios::CommandType::STOP_OLD_TASK: {
            RCLCPP_INFO(this->get_logger(), "Issuing command: stop old command...");
            stop_task_command(); // * don't care about the result
            result->result = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            return;
        }
        default:
            RCLCPP_ERROR(this->get_logger(), "ISSUING COMMAND: UNDEFINED COMMANDTYPE!");
            result->error_code = kios_interface::action::ExecuteSkill::Result::NODE_ERROR;
            result->message = "UNDEFINED COMMANDTYPE!";
            goal_handle->abort(result);
            return;
        }
    }

    ///////////////////////////////////////////////// ! ACTION SERVER   /////////////////////////////////////////////////

    void command_service_callback(
        const std::shared_ptr<kios_interface::srv::CommandRequest::Request> request,
        const std::shared_ptr<kios_interface::srv::CommandRequest::Response> response)
    {
        if (check_power() == true)
        {
            // * read the command request
            command_request_.command_type = static_cast<kios::CommandType>(request->command_type);
            try
            {
                command_request_.command_context = nlohmann::json::parse(request->command_context);
            }
            catch (...)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "SOMETHING WRONG WITH THE JSON PARSE!");
                response->is_accepted = false;
                return;
            }
            command_request_.skill_type = request->skill_type;
            issue_command(command_request_);
            response->is_accepted = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "POWER OFF, request refused!");
            response->is_accepted = false;
        }
    }

    void issue_command(const kios::CommandRequest &command_request)
    {
        switch (command_request.command_type)
        {
        case kios::CommandType::INITIALIZATION: {
            RCLCPP_INFO(this->get_logger(), "Issuing command: initialization...");
            break;
        }
        case kios::CommandType::STOP_OLD_START_NEW: {
            RCLCPP_INFO(this->get_logger(), "Issuing command: stop old start new...");
            if (stop_task_request() == true)
            {
                start_task_request(command_request);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Issuing command: BAD NEWS FROM RESPONSE!");
            }
            ////////////////////////////////
            // !!!!! TEST remove stop_task
            ////////////////////////////////
            // start_task_command(command_request.command_context);
            break;
        }
        case kios::CommandType::STOP_OLD_TASK: {
            RCLCPP_INFO(this->get_logger(), "Issuing command: stop old command...");
            stop_task_command();
            break;
        }
        default:
            RCLCPP_ERROR(this->get_logger(), "ISSUING COMMAND: UNDEFINED COMMANDTYPE!");
            break;
        }
    };

    // ! test
    bool stop_task_request()
    {
        auto result_opt = messenger_->stop_task_request();
        if (result_opt.has_value())
        {
            auto result = result_opt.value();
            spdlog::info("Stop task request get response if_success: {}", result["result"]["result"].dump());
            if (static_cast<bool>(result["result"]["result"]) == true)
            {
                return true;
            }
            else if (static_cast<bool>(result["result"]["result"]) == false)
            {
                spdlog::error("Error message: {}", result["result"]["error"].dump());
                return false;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    bool start_task_request(const kios::CommandRequest request)
    {
        auto result_opt = messenger_->start_task_request(request.command_context, request.skill_type);
        if (result_opt.has_value())
        {
            // ! dangerous
            task_response_ = std::move(result_opt.value());
            spdlog::info("start task request get response if_success: {}", task_response_["result"]["result"].dump());
            if (static_cast<bool>(task_response_["result"]["result"]) == true)
            {
                return true;
            }
            else if (static_cast<bool>(task_response_["result"]["result"]) == false)
            {
                spdlog::error("Error message: {}", task_response_["result"]["error"].dump());
                return false;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    // * run this in a new thread
    void wait_for_task_result(int task_uuid, std::promise<std::optional<nlohmann::json>> &task_promise, std::atomic_bool &isInterrupted)
    {
        messenger_->wait_for_task_result(task_uuid, task_promise, isInterrupted);
    }


    void stop_task_command()
    {
        messenger_->stop_task_command();
    }

    void start_task_command(const nlohmann::json &skill_context)
    {
        messenger_->start_task_command(skill_context);
    }

    void teach_object(const nlohmann::json &object_context)
    {
        messenger_->send_and_wait("teach_object", object_context);
    }

    // ! TODO
    void get_object(const nlohmann::json &object_context)
    {
        messenger_->send_and_wait("get_object", object_context);
    }

    /**
     * @brief teach object service callback. see teach_location in mios python module.
     *
     * @param request
     * @param response
     */
    void teach_object_service_callback(
        const std::shared_ptr<kios_interface::srv::TeachObjectService::Request> request,
        const std::shared_ptr<kios_interface::srv::TeachObjectService::Response> response)
    {
        if (check_power() == true)
        {
            // * read the command request
            std::string object_name = request->object_name;
            nlohmann::json object_context = {{"object", object_name}};
            teach_object(object_context);
            response->is_success = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "commander not running, request refused!");
            response->is_success = false;
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(Commander)

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto commander = std::make_shared<Commander>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(commander);

    executor.spin();

    // * unregister the udp before shutdown.
    commander->shut_down_connection();
    rclcpp::shutdown();
    return 0;
}
