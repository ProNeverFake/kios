
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
          udp_ip("127.0.0.1"),
          udp_port(12346),
          is_busy(false)
    {
        // declare power parameter
        this->declare_parameter("power", true);

        // callback group
        service_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // * initialize the websocket messenger
        m_messenger = std::make_shared<BTMessenger>(ws_url);
        // websocket connection
        m_messenger->special_connect();
        while (!m_messenger->wait_for_open_connection(3))
        {
            RCLCPP_INFO(this->get_logger(), "websocket connection not ready. Waiting for an open connection.");
        }

        // udp register
        mios_register_udp();

        // object announcement
        m_messenger->send_grasped_object();

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
    void mios_register_udp()
    {
        m_messenger->register_udp(udp_port);
    }
    void mios_unregister_udp()
    {
        m_messenger->unregister_udp();
    }
    void shut_down_connection()
    {
        m_messenger->unregister_udp();
        m_messenger->close();
    }
    bool check_power()
    {
        if (this->get_parameter("power").as_bool() == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

private:
    // flags
    bool is_busy;

    //! TEMP STATE
    kios::ActionPhaseContext action_phase_context_;
    kios::CommandRequest command_request_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    // callbacks
    rclcpp::Service<kios_interface::srv::CommandRequest>::SharedPtr command_service_;
    rclcpp::Service<kios_interface::srv::TeachObjectService>::SharedPtr teach_object_service_;

    // ws_client rel
    std::shared_ptr<BTMessenger> m_messenger;
    std::string ws_url;

    // udp subscriber ip
    std::string udp_ip;
    int udp_port;

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
            stop_task();
            start_task(command_request.command_context);
            break;
        }
        default:
            RCLCPP_ERROR(this->get_logger(), "ISSUING COMMAND: UNDEFINED COMMANDTYPE!");
            break;
        }
    };

    void stop_task()
    {
        m_messenger->stop_task();
    }

    void start_task(const nlohmann::json &skill_context)
    {
        m_messenger->start_task(skill_context);
    }

    void teach_object(const nlohmann::json &object_context)
    {
        m_messenger->send_and_wait("teach_object", object_context);
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
