
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/node_list.hpp"

#include "ws_client/ws_client.hpp"

#include "kios_interface/srv/command_request.hpp"

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
          is_runnning(true),
          is_busy(false)
    {
        // initialize the spdlog for ws_client
        std::string verbosity = "trace";
        spdlog::level::level_enum info_level;
        if (verbosity == "trace")
        {
            info_level = spdlog::level::trace;
        }
        else if (verbosity == "debug")
        {
            info_level = spdlog::level::debug;
        }
        else if (verbosity == "info")
        {
            info_level = spdlog::level::info;
        }
        else
        {
            info_level = spdlog::level::info;
        }

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(info_level);
        console_sink->set_pattern("[kios][ws_client][%^%l%$] %v");

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/kios_ws_client.txt", true);
        file_sink->set_level(spdlog::level::debug);

        auto logger = std::shared_ptr<spdlog::logger>(new spdlog::logger("mios", {console_sink, file_sink}));
        logger->set_level(info_level);
        spdlog::set_default_logger(logger);
        spdlog::info("spdlog: initialized.");

        // declare mission parameter
        this->declare_parameter("power_on", true);
        // callback group
        service_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

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
        service_ = this->create_service<kios_interface::srv::CommandRequest>(
            "command_request_service",
            std::bind(&Commander::service_callback, this, _1, _2),
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

private:
    // flags
    bool is_runnning;
    bool is_busy;

    //! TEMP STATE
    kios::ActionPhaseContext action_phase_context_;
    kios::CommandRequest command_request_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    // callbacks
    rclcpp::Service<kios_interface::srv::CommandRequest>::SharedPtr service_;

    // ws_client rel
    std::shared_ptr<BTMessenger> m_messenger;
    std::string ws_url;

    // udp subscriber ip
    std::string udp_ip;
    int udp_port;

    void service_callback(
        const std::shared_ptr<kios_interface::srv::CommandRequest::Request> request,
        const std::shared_ptr<kios_interface::srv::CommandRequest::Response> response)
    {
        if (is_runnning)
        {
            // * read the command request
            command_request_.command_type = static_cast<kios::CommandType>(request->command_type);
            try
            {
                command_request_.command_context = nlohmann::json::parse(request->command_context);
            }
            catch (...)
            {
                std::cerr << "SOMETHING WRONG WITH THE JSON PARSE!" << '\n';
            }
            issue_command(command_request_);
            response->is_accepted = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Not running, request refused!");
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
        // passing
    };

    void stop_task()
    {
        m_messenger->stop_task();
    }

    void start_task(const nlohmann::json &skill_context)
    {
        m_messenger->start_task(skill_context);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // register the nodes
    auto commander = std::make_shared<Commander>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(commander);

    executor.spin();

    // * unregister the udp before shutdown.
    commander->shut_down_connection();
    rclcpp::shutdown();
    return 0;
}
