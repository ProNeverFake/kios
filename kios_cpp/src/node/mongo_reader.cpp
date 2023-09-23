
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"

#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/tree_root.hpp"

#include "kios_communication/object_master.hpp"

#include "kios_interface/srv/command_request.hpp"
#include "kios_interface/srv/get_object_request.hpp"
#include "kios_utils/kios_utils.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

using std::placeholders::_1;
using std::placeholders::_2;

class MongoReader : public rclcpp::Node
{
public:
    MongoReader()
        : Node("mongo_reader"),
          mongo_port(27017),
          // ! THIS UNSIGNED VARIABLE IS PASSED AS ZERO IN MONGODB_CLIENT. NOT FIXED YET.
          object_master_ptr_(std::make_shared<kios::ObjectMaster>("left"))
    {
        //*  initialize the spdlog for ws_client
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
        console_sink->set_pattern("[kios][mongo_reader][%^%l%$] %v");

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/kios_mongoreader.txt", true);
        file_sink->set_level(spdlog::level::debug);

        auto logger = std::shared_ptr<spdlog::logger>(new spdlog::logger("mongoreader", {console_sink, file_sink}));
        logger->set_level(info_level);
        spdlog::set_default_logger(logger);
        spdlog::info("spdlog: initialized.");

        // declare power parameter
        this->declare_parameter("power", true);

        // callback group
        service_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        // timer_callback_group_ = this->create_callback_group(
        //     rclcpp::CallbackGroupType::MutuallyExclusive);

        get_object_serer_ = this->create_service<kios_interface::srv::GetObjectRequest>(
            "get_object_service",
            std::bind(&MongoReader::get_object_service_callback, this, _1, _2),
            rmw_qos_profile_services_default,
            service_callback_group_);

        // test_timer_ = this->create_wall_timer(
        //     std::chrono::seconds(5),
        //     std::bind(&MongoReader::timer_callback, this),
        //     timer_callback_group_);
        // * initialize object master
        object_master_ptr_->initialize(0);
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
    std::shared_ptr<kios::ObjectMaster> object_master_ptr_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    // rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    // callbacks
    rclcpp::Service<kios_interface::srv::GetObjectRequest>::SharedPtr get_object_serer_;
    // rclcpp::TimerBase::SharedPtr test_timer_;

    unsigned int mongo_port; // not used now

    std::unordered_map<std::string, kios::Object> object_dictionary_;

    bool update_object_dictionary()
    {
        if (object_master_ptr_->load_environment())
        {
            auto dict = object_master_ptr_->get_object_dictionary();
            object_dictionary_.swap(dict);
            return true;
        }
        else
        {
            return false;
        }
    }

    void get_object_service_callback(
        const std::shared_ptr<kios_interface::srv::GetObjectRequest::Request> request,
        const std::shared_ptr<kios_interface::srv::GetObjectRequest::Response> response)
    {
        if (check_power() == true)
        {
            if (update_object_dictionary())
            {
                // * request context is the name list of objects that need to be fetched. but now just fetch all.
                // * assemble the response
                try
                {
                    for (const auto &entity : object_dictionary_)
                    {
                        response->object_name.push_back(entity.first);
                        response->object_data.push_back(entity.second.to_json().dump());
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), "Service call accepted.");
                    response->is_accepted = true;
                }
                catch (...)
                {
                    RCLCPP_ERROR(this->get_logger(), "FAILED WHEN ASSEMBLING GET OBJECT RESPONSE!");
                    response->is_accepted = false;
                    switch_power(false);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "FAILED WHEN UPDATING OBJECT DICTIONARY!");
                response->is_accepted = false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Mongo reader not running, request refused!");
            response->is_accepted = false;
        }
    }

    // // ! TEST
    // void timer_callback()
    // {
    //     if (update_object_dictionary())
    //     {

    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "ERROR!");
    //     }
    // }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto mongo_reader = std::make_shared<MongoReader>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mongo_reader);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
