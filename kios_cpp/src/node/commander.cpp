
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/tree_root.hpp"

#include "kios_communication/ws_client.hpp"

#include "kios_interface/srv/command_request.hpp"
#include "kios_interface/srv/teach_object_service.hpp"

#include "kios_utils/kios_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Commander : public rclcpp::Node
{
public:
    Commander()
        : Node("commander"),
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

        // ! HERE REMOVED. THE PRECONDITION IN BBGENERALSKILL IS ALSO REMOVED.
        // messenger_->send_grasped_object();

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

    // callback group
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    // callbacks
    rclcpp::Service<kios_interface::srv::CommandRequest>::SharedPtr command_service_;
    rclcpp::Service<kios_interface::srv::TeachObjectService>::SharedPtr teach_object_service_;

    // ws_client rel
    std::shared_ptr<BTMessenger> messenger_;
    std::string ws_url;

    // udp subscriber ip
    std::string udp_ip; // not used
    int udp_port_;
    nlohmann::json subscription_list_;
    // std::vector<std::string> subscription_list;

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

    bool stop_task_request()
    {
        return messenger_->stop_task_request();
    }

    bool start_task_request(const kios::CommandRequest request)
    {
        return messenger_->start_task_request(request.command_context, request.skill_type);
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
