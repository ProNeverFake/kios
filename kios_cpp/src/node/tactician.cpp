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

#include "kios_interface/srv/archive_action_request.hpp"
#include "kios_interface/srv/fetch_skill_parameter_request.hpp"

#include "kios_utils/kios_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Tactician : public rclcpp::Node
{
public:
    Tactician()
        : Node("tactician"),
          command_context_(),
          tree_state_(),
          task_state_(),
          context_clerk_()
    {
        std::cout << "start initialization" << std::endl;

        //* initialize the callback groups
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        client_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        server_callback_group_ = client_callback_group_;

        // * initialize archive action server
        archive_action_server_ = this->create_service<kios_interface::srv::ArchiveActionRequest>(
            "archive_action_service",
            std::bind(&Tactician::archive_action_server_callback, this, _1, _2),
            rmw_qos_profile_services_default,
            server_callback_group_);

        // * initialize fetch skill parameter server
        fetch_skill_parameter_server_ = this->create_service<kios_interface::srv::FetchSkillParameterRequest>(
            "fetch_skill_parameter_service",
            std::bind(&Tactician::fetch_skill_parameter_server_callback, this, _1, _2),
            rmw_qos_profile_services_default,
            server_callback_group_);

        // * initialize the subscription
        // ! the subscription is supressed for now.
        // rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        // rclcpp::SubscriptionOptions subscription_options;
        // subscription_options.callback_group = subscription_callback_group_;

        // task_state_subscription_ = this->create_subscription<kios_interface::msg::TaskState>(
        //     "task_state_topic",
        //     qos,
        //     std::bind(&Tactician::task_subscription_callback, this, _1),
        //     subscription_options);

        // * initialize context clerk
        context_clerk_.read_archive(); // bool value return is not useful here.

        std::cout << "finish initialization" << std::endl;

        rclcpp::sleep_for(std::chrono::seconds(3));
    }

private:
    // action parameter manager
    kios::ContextClerk context_clerk_;

    // thread safe rel
    std::mutex tree_state_mtx_;
    std::mutex task_state_mtx_;

    // state of tree and task
    kios::TaskState task_state_;
    kios::TreeState tree_state_;

    kios::CommandContext command_context_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr server_callback_group_;

    rclcpp::Service<kios_interface::srv::ArchiveActionRequest>::SharedPtr archive_action_server_;

    // rclcpp::Subscription<kios_interface::msg::TaskState>::SharedPtr task_state_subscription_;

    rclcpp::Service<kios_interface::srv::FetchSkillParameterRequest>::SharedPtr fetch_skill_parameter_server_;

    // /**
    //  * @brief subcriber callback. update member variable task_state (percept).
    //  ! suppress this for now.
    //  * @param msg
    //  */
    // void task_subscription_callback(kios_interface::msg::TaskState::SharedPtr msg)
    // {
    //     std::lock_guard<std::mutex> task_state_guard(task_state_mtx_);
    //     // RCLCPP_INFO(this->get_logger(), "SUB HIT, try to move");
    //     // * update task state
    //     task_state_.from_ros2_msg(*msg);
    // }

    void fetch_skill_parameter_server_callback(
        const std::shared_ptr<kios_interface::srv::FetchSkillParameterRequest::Request> request,
        const std::shared_ptr<kios_interface::srv::FetchSkillParameterRequest::Response> response)
    {
        // * update tree state
        std::lock_guard<std::mutex> lock(tree_state_mtx_);
        tree_state_.tree_phase = static_cast<kios::TreePhase>(request->tree_phase);
        tree_state_.node_archive = kios::NodeArchive::from_ros2_msg(request->node_archive);
        tree_state_.object_keys = std::move(request->object_keys);
        tree_state_.object_names = std::move(request->object_names);

        // context = skill parameter
        nlohmann::json context = context_clerk_.get_context(tree_state_.node_archive);
        // * important: here remove the action_context to prevent context inconsistency in mios
        if (context["skill"].contains("action_context"))
        {
            context["skill"].erase("action_context");
        }
        // ground the objects
        const auto &obj_keys = tree_state_.object_keys;
        const auto &obj_names = tree_state_.object_names;
        for (int i = 0; i < obj_keys.size(); i++)
        {
            context["skill"]["objects"][obj_keys[i]] = obj_names[i];
        }

        // load skill parameters into response
        response->skill_parameters_json = context.dump();
        RCLCPP_INFO(this->get_logger(), "fetch skill parameter request accepted.");
        response->is_accepted = true;
    }

    /**
     * @brief the method to archive all the nodes.
     *
     * @param request
     * @param response
     */
    void archive_action_server_callback(
        const std::shared_ptr<kios_interface::srv::ArchiveActionRequest::Request> request,
        const std::shared_ptr<kios_interface::srv::ArchiveActionRequest::Response> response)
    {
        std::string err_msg = "";
        bool isAccepted = true;

        RCLCPP_INFO(this->get_logger(), "Archiving the actions...");
        for (auto &archive : request->archive_list)
        {
            // decode the archive
            kios::NodeArchive arch{archive.action_group, archive.action_id, archive.description, static_cast<kios::ActionPhase>(archive.action_phase)};
            // try to archive the node.
            if (!context_clerk_.archive_action(arch))
            {
                err_msg = "ERROR when archiving the action in group " + std::to_string(archive.action_group) + " with id " + std::to_string(archive.action_id);
                isAccepted = false;
                RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
                break;
            }
        }

        response->is_accepted = isAccepted;
        response->error_message = err_msg;
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