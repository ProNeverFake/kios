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
          isSwitchAction(false),
          isBusy(false),
          command_context_(),
          tree_state_(),
          task_state_(),
          context_clerk_()
    {
        std::cout << "start initialization" << std::endl;

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

        // * initialize switch action server
        switch_action_server_ = this->create_service<kios_interface::srv::SwitchActionRequest>(
            "switch_action_service",
            std::bind(&Tactician::switch_action_server_callback, this, _1, _2),
            rmw_qos_profile_services_default,
            server_callback_group_);

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
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = subscription_callback_group_;

        // * initialize the callbacks
        task_state_subscription_ = this->create_subscription<kios_interface::msg::TaskState>(
            "task_state_topic",
            qos,
            std::bind(&Tactician::task_subscription_callback, this, _1),
            subscription_options);

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100),
        //     std::bind(&Tactician::timer_callback, this),
        //     timer_callback_group_);

        // * initialize context clerk
        // context_clerk_.initialize(); // ! YOU SHOULD NOT USE THIS.
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

    // flags
    std::atomic_bool isSwitchAction;
    std::atomic_bool isBusy; // necessary because of the timer callback group setting

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
    rclcpp::Service<kios_interface::srv::ArchiveActionRequest>::SharedPtr archive_action_server_;
    rclcpp::Client<kios_interface::srv::CommandRequest>::SharedPtr command_client_;
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<kios_interface::msg::TaskState>::SharedPtr task_state_subscription_;

    rclcpp::Service<kios_interface::srv::FetchSkillParameterRequest>::SharedPtr fetch_skill_parameter_server_;

    /**
     * @brief subcriber callback. update member variable task_state (percept).
     *
     * @param msg
     */
    void task_subscription_callback(kios_interface::msg::TaskState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> task_state_guard(task_state_mtx_);
        // RCLCPP_INFO(this->get_logger(), "SUB HIT, try to move");
        // * update task state
        task_state_.from_ros2_msg(*msg);
    }

    /**
     * @brief switch action server callback. Response to the request and set flag for timer callback.
     * ! will be deprecated
     * @param request
     * @param response
     */
    void switch_action_server_callback(
        const std::shared_ptr<kios_interface::srv::SwitchActionRequest::Request> request,
        const std::shared_ptr<kios_interface::srv::SwitchActionRequest::Response> response)
    {
        if (isBusy.load())
        {
            RCLCPP_ERROR(this->get_logger(), "Node is busy, request refused!");
            response->is_accepted = false;
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "SWITCH ACTION HIT!");

            // * update tree state
            std::lock_guard<std::mutex> lock(tree_state_mtx_);
            // tree_state_.action_name = std::move(request->action_name);
            // tree_state_.action_phase = static_cast<kios::ActionPhase>(request->action_phase);
            tree_state_.tree_phase = static_cast<kios::TreePhase>(request->tree_phase);

            tree_state_.object_keys = std::move(request->object_keys);
            tree_state_.object_names = std::move(request->object_names);

            tree_state_.node_archive = kios::NodeArchive::from_ros2_msg(request->node_archive);

            // * set flag for timer
            isSwitchAction.store(true);

            RCLCPP_INFO(this->get_logger(), "switch_action request accepted.");
            response->is_accepted = true;
        }
    }

    /**
     * @brief switch action server callback. Response to the request and set flag for timer callback.
     *
     * @param request
     * @param response
     */
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
        // ! important: here remove the action_context to prevent context inconsistency in mios
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

        if (isBusy.load())
        {
            isAccepted = false;
            err_msg = "Node is busy, request refused !";
            RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Archiving the actions...");
            for (auto &archive : request->archive_list)
            {
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
        }

        response->is_accepted = isAccepted;
        response->error_message = err_msg;
    }

    /**
     * @brief prereserved inline method to ground the skill
     *
     */
    void generate_command_context()
    {
        /////////////////////////////////////////////
        // * now just use default
        // fetch context from clerk
        nlohmann::json context = context_clerk_.get_context(tree_state_.node_archive);
        // ! important: remove the action_context. this is not necessary.
        if (context["skill"].contains("action_context"))
        {
            context["skill"].erase("action_context");
        }
        // * the objects to be grounded from mongoDB can be changed here according to the object_keys.
        // * use those from the request.
        const auto &obj_keys = tree_state_.object_keys;
        const auto &obj_names = tree_state_.object_names;
        for (int i = 0; i < obj_keys.size(); i++)
        {
            context["skill"]["objects"][obj_keys[i]] = obj_names[i];
        }
        // * only use stop old start new now
        command_context_.command_type = kios::CommandType::STOP_OLD_START_NEW;

        // * USE THIS INSTEAD
        command_context_.command_context = context;
        command_context_.skill_type = kios::ap_to_mios_skill(tree_state_.node_archive.action_phase);

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
        request->skill_type = command_context_.skill_type;

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
     * @brief handle the switch action request. generate the command context and send it to commander.
     *
     */
    void handle_request()
    {
        RCLCPP_INFO(this->get_logger(), "HANDLE REQUEST");

        std::lock_guard<std::mutex> task_state_guard(task_state_mtx_);
        // handle the request according to tree phase;
        switch (tree_state_.tree_phase)
        {
        case kios::TreePhase::PAUSE: {
            RCLCPP_ERROR(this->get_logger(), "TREE PHASE: PAUSE");

            // * tree is waiting for tactician to generate parameter and request for command.
            generate_command_context();
            if (!send_command_request(1000, 1000))
            {
                //* error in command request service.
                // * invoke error in tree

                // * turn off for check
            }
            break;
        }
        case kios::TreePhase::ERROR: {
            // * error in tree. stop and check.
            break;
        }
        case kios::TreePhase::FAILURE: {
            // * this won't arrive at tactician in principle. but just stop.
            break;
        }
        case kios::TreePhase::FINISH: {
            RCLCPP_ERROR(this->get_logger(), "TREE PHASE: FINISH");

            // * the tree is finished. send stop old task command and stop
            command_context_.command_type = kios::CommandType::STOP_OLD_TASK;

            if (!send_command_request(1000, 1000))
            {
                //* error in command request service.
                // * invoke error in tree

                // * turn off for check
            }
            // * archive the nodes contexts into json file.
            context_clerk_.store_archive();
            break;
        }
        case kios::TreePhase::IDLE: {
            RCLCPP_ERROR(this->get_logger(), "TREE PHASE: IDLE");
            // * this won't arrive at tactician in principle. there must be an error somewhere.
            break;
        }
        case kios::TreePhase::SUCCESS: {
            RCLCPP_ERROR(this->get_logger(), "TREE PHASE: SUCCESS");

            // * mios success ---> tree success ---> tree start new action ---> tree pause and send switch action phase request
            // * ............ ---> ............ ---> BT return success and FINISH ---> tree send switch action phase request with FINISH
            // * so this won't arrive at tactician in principle.

            break;
        }
        case kios::TreePhase::RESUME: {
            RCLCPP_ERROR(this->get_logger(), "TREE PHASE: RESUME");

            // * this won't ....
            break;
        }

        default: {
            RCLCPP_ERROR(this->get_logger(), "HANDLING UNDEFINED TREEPHASE!");
            break;
        }
        }
    }

    /**
     * @brief timer callback. check the need of request handling periodically.
     * ! will be deprecated
     */
    void timer_callback()
    {
        if (isSwitchAction.load() == true)
        {
            if (!isBusy.load())
            {
                // set busy
                isBusy.store(true);

                handle_request();

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
            RCLCPP_INFO_ONCE(this->get_logger(), "Timer: Continue the last action phase.");
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