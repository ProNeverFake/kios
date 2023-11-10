#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_server.hpp>
#include <string>
#include <optional>
#include <mutex>
#include <thread>

#include "kios_utils/data_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/tree_root.hpp"
#include "kios_utils/kios_utils.hpp"
#include "kios_communication/boost_udp.hpp"

#include "kios_interface/msg/tree_state.hpp"
#include "kios_interface/msg/task_state.hpp"

#include "kios_interface/srv/get_object_request.hpp"
#include "kios_interface/srv/switch_tree_phase_request.hpp"
#include "kios_interface/srv/get_object_request.hpp"
#include "kios_interface/srv/archive_action_request.hpp"
#include "kios_interface/srv/command_request.hpp"
#include "kios_interface/srv/execute_tree_request.hpp"

#include "kios_interface/srv/fetch_skill_parameter_request.hpp"
#include "kios_interface/action/execute_tree.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using ExecuteTree = kios_interface::action::ExecuteTree;
using GoalHandleExecuteTree = rclcpp_action::ServerGoalHandle<ExecuteTree>;

class TreeNode : public rclcpp::Node
{
public:
    explicit TreeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("tree_node", options),
          isActionSuccess_(false),
          tree_state_ptr_(std::make_shared<kios::TreeState>()),
          task_state_ptr_(std::make_shared<kios::TaskState>()),
          hasUpdatedObjects_(false),
          hasLoadedArchive_(false),
          node_archive_list_()
    {
        // * set ros2 logger severity level
        auto logger = this->get_logger();
        rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_INFO);

        //* declare mission parameter
        this->declare_parameter("power", true);

        //* initialize the callback groups
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        subscription_callback_group_ = timer_callback_group_;

        client_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // ! for test
        // m_tree_root = std::make_shared<Insertion::TreeRoot>(tree_state_ptr_, task_state_ptr_);

        // ! for TEST initialize the tree with test_tree
        // tree_initialize(); // bool return is not handled.

        // * Set qos and options
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = subscription_callback_group_;

        // * initialize the callbacks
        subscription_ = this->create_subscription<kios_interface::msg::TaskState>(
            "task_state_topic",
            qos,
            std::bind(&TreeNode::subscription_callback, this, _1),
            subscription_options);

        archive_action_client_ = this->create_client<kios_interface::srv::ArchiveActionRequest>(
            "archive_action_service",
            rmw_qos_profile_services_default,
            client_callback_group_);

        get_object_client_ = this->create_client<kios_interface::srv::GetObjectRequest>(
            "get_object_service",
            rmw_qos_profile_services_default,
            client_callback_group_);
        // switch_tree_phase_server_ = this->create_service<kios_interface::srv::SwitchTreePhaseRequest>(
        //     "switch_tree_phase_service",
        //     std::bind(&TreeNode::switch_tree_phase_server_callback, this, _1, _2),
        //     rmw_qos_profile_services_default,
        //     client_callback_group_);
        fetch_skill_parameter_client_ = this->create_client<kios_interface::srv::FetchSkillParameterRequest>(
            "fetch_skill_parameter_service",
            rmw_qos_profile_services_default,
            client_callback_group_);
        command_client_ = this->create_client<kios_interface::srv::CommandRequest>(
            "command_request_service",
            rmw_qos_profile_services_default,
            client_callback_group_);

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100),
        //     std::bind(&TreeNode::timer_callback, this),
        //     timer_callback_group_);

        this->execute_tree_action_server_ = rclcpp_action::create_server<ExecuteTree>(
            this,
            "execute_tree_action",
            std::bind(&TreeNode::execute_tree_handle_goal, this, _1, _2),
            std::bind(&TreeNode::execute_tree_handle_cancel, this, _1),
            std::bind(&TreeNode::execute_tree_handle_accepted, this, _1));

        udp_socket_ = std::make_shared<kios::BTReceiver>("127.0.0.1", 8888);

        // * set tree phase to resume to let tree tick
        tree_phase_ = kios::TreePhase::RESUME;

        rclcpp::sleep_for(std::chrono::seconds(4));
    }

    bool check_power()
    {
        // lack the check of the parameter existence
        return this->get_parameter("power").as_bool();
    }

    void switch_power(bool turn_on)
    {
        std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("power", turn_on)};
        this->set_parameters(all_new_parameters);
        if (turn_on)
        {
            RCLCPP_WARN(this->get_logger(), "switch_power: turn on.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "switch_power: turn off.");
        }
    }

private:
    std::vector<kios_interface::msg::NodeArchive> node_archive_list_;
    bool hasLoadedArchive_;

    // flag
    bool isActionSuccess_;
    bool hasUpdatedObjects_;

    // thread data rel
    std::mutex tree_mtx_;
    std::mutex tree_phase_mtx_;

    nlohmann::json skill_parameter_;

    // * UDP socket rel
    std::shared_ptr<kios::BTReceiver> udp_socket_;

    // tree rel
    kios::TreePhase tree_phase_;
    std::shared_ptr<kios::TreeState> tree_state_ptr_;
    std::shared_ptr<kios::TaskState> task_state_ptr_;

    // object dictionary
    std::shared_ptr<std::unordered_map<std::string, kios::Object>> object_dictionary_ptr_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<kios_interface::msg::TaskState>::SharedPtr subscription_;
    rclcpp::Client<kios_interface::srv::ArchiveActionRequest>::SharedPtr archive_action_client_;
    // rclcpp::Service<kios_interface::srv::SwitchTreePhaseRequest>::SharedPtr switch_tree_phase_server_;
    rclcpp::Client<kios_interface::srv::GetObjectRequest>::SharedPtr get_object_client_;
    rclcpp::Client<kios_interface::srv::FetchSkillParameterRequest>::SharedPtr fetch_skill_parameter_client_;
    rclcpp::Client<kios_interface::srv::CommandRequest>::SharedPtr command_client_;

    rclcpp_action::Server<ExecuteTree>::SharedPtr execute_tree_action_server_;

    // behavior tree rel
    std::shared_ptr<Insertion::TreeRoot> m_tree_root;
    BT::NodeStatus tick_result;

    ////////////////////////////////// ACTION SERVER ///////////////////////////////////////

    rclcpp_action::GoalResponse execute_tree_handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ExecuteTree::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received execute_tree goal request!");
        (void)uuid;
        (void)goal;
        // ! must accept.
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse execute_tree_handle_cancel(
        const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        // ! should not be canceled by the client.
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute_tree_handle_accepted(const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
    {
        using namespace std::placeholders;
        // * start the thread to execute the tree
        std::thread{std::bind(&TreeNode::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteTree::Feedback>();

        auto result = std::make_shared<ExecuteTree::Result>();

        // * initialize the tree here:

        // ////////////////////////////////////////////////////////////////
        // rclcpp::Rate loop_rate(1);
        // while (rclcpp::ok())
        // {
        //     // Check if there is a cancel request
        //     if (goal_handle->is_canceling())
        //     {
        //         //! this part should not be executed.
        //         goal_handle->canceled(result);
        //         RCLCPP_INFO(this->get_logger(), "Goal canceled");
        //         return;
        //     }
        //     // * execute the tree, tick

        //     // * feedback (you don't need feedback)
        //     goal_handle->publish_feedback(feedback);
        //     RCLCPP_INFO(this->get_logger(), "Publish feedback");

        //     loop_rate.sleep();
        // }

        // // * lock tree phase first
        // std::lock_guard<std::mutex> lock_tree_phase(tree_phase_mtx_);
        // // * check the necessity of updating objects
        // if (hasUpdatedObjects_ == false)
        // {
        //     RCLCPP_INFO(this->get_logger(), "update the object...");
        //     if (update_object(1000, 1000) == false)
        //     {
        //         switch_tree_phase("ERROR", tree_phase_);
        //     }
        //     else
        //     {
        //         hasUpdatedObjects_ = true;
        //     }
        // }

        // // * ask tactician to load the actions' parameters according to the archives.
        // if (hasLoadedArchive_ == false)
        // {
        //     // !! BB: THE CORRECT ORDER SHOULD BE FETCH THE OBJECT, GENERATE THE TREE, THEN CHECK THE OBJECTS WHEN GENERATING THE TREE.
        //     // ! NOW JUST CHECK THE OBJECT HERE.
        //     if (!m_tree_root->check_grounded_objects())
        //     {
        //         switch_tree_phase("ERROR", tree_phase_);
        //     }
        //     // !!

        //     RCLCPP_INFO_STREAM(this->get_logger(), "Now ask the tactician to Load the archives...");
        //     if (load_node_archive(1000, 1000) == false)
        //     {
        //         switch_tree_phase("ERROR", tree_phase_);
        //     }
        //     else
        //     {
        //         hasLoadedArchive_ = true;
        //     }
        // }

        // // * get mios skill execution state (only if there is a msg in the msg queue of udp receiver)
        // std::string message;
        // if (udp_socket_->get_message(message) == true)
        // {
        //     if (!switch_tree_phase(message, tree_phase_))
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "switch_tree_phase: TREE PHASE UNDEFINED!");
        //         // * turn off the tree node for debug.
        //         switch_tree_phase("ERROR", tree_phase_);
        //     }
        // }

        // // * lock tree first
        // std::lock_guard<std::mutex> lock_tree(tree_mtx_);
        // // * update tree phase in tree state for the need in BT
        // tree_state_ptr_->tree_phase = tree_phase_;
        // // * do tree cycle
        // tree_cycle();

        // ////////////////////////////////////////////////////////////////

        // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
        // {
        //     // Check if there is a cancel request
        //     if (goal_handle->is_canceling())
        //     {
        //         result->sequence = sequence;
        //         goal_handle->canceled(result);
        //         RCLCPP_INFO(this->get_logger(), "Goal canceled");
        //         return;
        //     }
        //     // Update sequence
        //     sequence.push_back(sequence[i] + sequence[i - 1]);
        //     // Publish feedback
        //     goal_handle->publish_feedback(feedback);
        //     RCLCPP_INFO(this->get_logger(), "Publish feedback");

        //     loop_rate.sleep();
        // }

        feedback->update = "Executing";
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Check if goal is done
        if (rclcpp::ok())
        {
            result->result_code = 0;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////

    bool tree_initialize()
    {
        // generate the tree in tree root
        if (!m_tree_root->initialize_tree())
        {
            return false;
        }
        // load the archives of the tree's action nodes into nodearchivelist
        node_archive_list_.clear();
        auto archive_list = m_tree_root->archive_nodes();
        if (archive_list.has_value())
        {
            for (auto &archive : archive_list.value())
            {
                node_archive_list_.push_back(archive.to_ros2_msg());
            }
        }
        else
        {
            return false;
        }
        return true;
    }

    /**
     * @brief update the task_state. here the lock priority is lower than timer.
     *
     * @param msg
     */
    void subscription_callback(kios_interface::msg::TaskState::SharedPtr msg)
    {
        std::unique_lock<std::mutex> lock(tree_mtx_, std::try_to_lock);
        if (lock.owns_lock())
        {
            // * update task state
            task_state_ptr_->from_ros2_msg(*msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "SUBSCRIPTION: LOCK FAILED. PASS.");
        }
    }

    // /**
    //  * @brief handle the switch tree phase request.
    //  * @param request
    //  * @param response
    //  */
    // void switch_tree_phase_server_callback(
    //     const std::shared_ptr<kios_interface::srv::SwitchTreePhaseRequest::Request> request,
    //     const std::shared_ptr<kios_interface::srv::SwitchTreePhaseRequest::Response> response)
    // {
    //     if (check_power() == true)
    //     {
    //         std::string tree_phase_str = kios::tree_phase_to_str(static_cast<kios::TreePhase>(request->tree_phase));
    //         RCLCPP_INFO_STREAM(
    //             this->get_logger(),
    //             "Switch tree phase to " << tree_phase_str
    //                                     << " on the request from node " << request->client_name
    //                                     << " for reason: " << request->reason);
    //         {
    //             std::lock_guard<std::mutex> lock(tree_phase_mtx_);
    //             tree_phase_ = static_cast<kios::TreePhase>(request->tree_phase);
    //         }
    //         response->is_accepted = true;
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR_STREAM(
    //             this->get_logger(),
    //             "POWER OFF, request from " << request->client_name << " refused!");
    //         response->is_accepted = false;
    //     }
    // }

    /**
     * @brief inline function for send the command request to commander
     *
     * @param ready_deadline max time to wait until server is ready
     * @param response_deadline max time to wait until response is ready
     * @return true
     * @return false
     */
    bool send_command_request(kios::CommandType cmd_type, int ready_deadline = 50, int response_deadline = 50)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "send_command_request: " << int(cmd_type));
        auto request = std::make_shared<kios_interface::srv::CommandRequest::Request>();
        request->command_type = static_cast<int32_t>(cmd_type);
        request->command_context = skill_parameter_.dump();
        request->skill_type = kios::ap_to_mios_skill(tree_state_ptr_->node_archive.action_phase);

        // client send request
        while (!command_client_->wait_for_service(std::chrono::milliseconds(ready_deadline)))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Service " << command_client_->get_service_name() << " is not available after waiting for " << ready_deadline << " miliseconds!");
            // can try to reconnect here
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

    bool send_fetch_skill_parameter_request(int ready_deadline = 50, int response_deadline = 50)
    {
        // * send request to update the object
        auto request = std::make_shared<kios_interface::srv::FetchSkillParameterRequest::Request>();

        // skill identifier
        request->node_archive = tree_state_ptr_->node_archive.to_ros2_msg();

        // tree state
        request->tree_phase = static_cast<int32_t>(tree_state_ptr_->tree_phase);

        // objects
        request->object_keys = tree_state_ptr_->object_keys;
        request->object_names = tree_state_ptr_->object_names;

        int try_times = 5;
        while (!fetch_skill_parameter_client_->wait_for_service(std::chrono::milliseconds(ready_deadline)))
        {
            try_times--;
            if (try_times == 0)
            {
                RCLCPP_ERROR(this->get_logger(), "service %s not available after retries.", fetch_skill_parameter_client_->get_service_name());
                return false;
            }
            continue;
        }
        auto result_future = fetch_skill_parameter_client_->async_send_request(request);
        std::future_status status = result_future.wait_until(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(response_deadline));
        if (status == std::future_status::ready)
        {
            auto result = result_future.get();
            if (result->is_accepted == true)
            {
                RCLCPP_INFO(this->get_logger(), "Service %s response: request accepted.", fetch_skill_parameter_client_->get_service_name());
                // ! hier ist try/catch noetig
                skill_parameter_ = nlohmann::json::parse(result->skill_parameters_json);
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Service %s response: request refused!", fetch_skill_parameter_client_->get_service_name());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "UNKNOWN ERROR: service %s future is available but not ready.", fetch_skill_parameter_client_->get_service_name());
            return false;
        }
    }

    /**
     * @brief THE MAINLINE OF THE NODE.
     * * timer is used for testing the executing part of kios.
     */
    void timer_callback()
    {
        if (check_power())
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Timer works...");
            // * lock tree phase first
            std::lock_guard<std::mutex> lock_tree_phase(tree_phase_mtx_);
            // * check the necessity of updating objects
            if (hasUpdatedObjects_ == false)
            {
                RCLCPP_INFO(this->get_logger(), "update the object...");
                if (update_object(1000, 1000) == false)
                {
                    switch_tree_phase("ERROR", tree_phase_);
                }
                else
                {
                    hasUpdatedObjects_ = true;
                }
            }

            // * ask tactician to load the actions' parameters according to the archives.
            if (hasLoadedArchive_ == false)
            {
                // !! BB: THE CORRECT ORDER SHOULD BE FETCH THE OBJECT, GENERATE THE TREE, THEN CHECK THE OBJECTS WHEN GENERATING THE TREE.
                // ! NOW JUST CHECK THE OBJECT HERE.
                if (!m_tree_root->check_grounded_objects())
                {
                    switch_tree_phase("ERROR", tree_phase_);
                }
                // !!

                RCLCPP_INFO_STREAM(this->get_logger(), "Now ask the tactician to Load the archives...");
                if (load_node_archive(1000, 1000) == false)
                {
                    switch_tree_phase("ERROR", tree_phase_);
                }
                else
                {
                    hasLoadedArchive_ = true;
                }
            }

            // * get mios skill execution state (only if there is a msg in the msg queue of udp receiver)
            std::string message;
            if (udp_socket_->get_message(message) == true)
            {
                if (!switch_tree_phase(message, tree_phase_))
                {
                    RCLCPP_ERROR(this->get_logger(), "switch_tree_phase: TREE PHASE UNDEFINED!");
                    // * turn off the tree node for debug.
                    switch_tree_phase("ERROR", tree_phase_);
                }
            }

            // * lock tree first
            std::lock_guard<std::mutex> lock_tree(tree_mtx_);
            // * update tree phase in tree state for the need in BT
            tree_state_ptr_->tree_phase = tree_phase_;
            // * do tree cycle
            tree_cycle();
        }
        else
        {
            RCLCPP_ERROR_ONCE(this->get_logger(), "POWER OFF, TIMER PASS...");
        }
    }

    /**
     * @brief check the tree state. set the tree phase if tree is not running.
     *
     * @return true tree still running
     * @return false tree is finished or in error. set tree phase
     */
    bool is_tree_running()
    {
        // * check tree state first for detect possibly invoked error in tree node.
        if (tree_state_ptr_->tree_phase == kios::TreePhase::ERROR)
        {
            RCLCPP_FATAL(this->get_logger(), "DETECT INVOKED INNER ERROR FROM TREE NODE!");
            switch_power(false);
            return false;
        }

        // * check tick_result
        switch (tick_result)
        {
        case BT::NodeStatus::RUNNING: {
            return true;
            break;
        };
        case BT::NodeStatus::SUCCESS: {
            RCLCPP_INFO(this->get_logger(), "IS_TREE_RUNNING: MISSION SUCCEEDS.");
            switch_tree_phase("FINISH", tree_phase_);
            return false;
            break;
        };
        case BT::NodeStatus::FAILURE: {
            RCLCPP_ERROR(this->get_logger(), "IS_TREE_RUNNING: TREE IN FAILURE STATUS, MISSION FAILS.");
            switch_tree_phase("FAILURE", tree_phase_);
            return false;
            break;
        }
        default: {
            RCLCPP_ERROR(this->get_logger(), "IS_TREE_RUNNNING: HANDLER FOR RETURNED BT::NODESTATUS IS NOT DEFINED!");
            switch_tree_phase("ERROR", tree_phase_);
            return false;
        }
        }
    }

    /**
     * @brief inline method for tree life cycle. execute the corresponsing method according to the
     * current tree phase.
     *
     */
    inline void tree_cycle()
    {
        switch (tree_phase_)
        {
        case kios::TreePhase::PAUSE: {
            RCLCPP_INFO(this->get_logger(), "tree_cycle: PAUSE.");
            // * tree is waiting for resume signal from mios. reset action success flag. skip tree tick.
            isActionSuccess_ = false;
            break;
        }
        case kios::TreePhase::RESUME: {
            RCLCPP_INFO(this->get_logger(), "tree_cycle: RESUME.");
            // * normal phase.
            execute_tree();
            break;
        }
        case kios::TreePhase::SUCCESS: {
            RCLCPP_INFO(this->get_logger(), "tree_cycle: SUCCESS!");
            // * execute the tree with mios success flag.
            isActionSuccess_ = true;
            execute_tree();
            break;
        }
        case kios::TreePhase::ERROR: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: ERROR!");
            // * error at tree side. turn off for debug.
            // ! should handle the error
            switch_power(false);
            break;
        }
        case kios::TreePhase::FAILURE: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: FAILURE!");
            // * failure at mios side. turn off for debug.
            // TODO update this in action node.
            switch_power(false);
            break;
        }
        case kios::TreePhase::IDLE: {
            RCLCPP_INFO(this->get_logger(), "tree_cycle: IDLE.");
            // initial phase. do nothing.
            break;
        }
        case kios::TreePhase::FINISH: {
            RCLCPP_INFO(this->get_logger(), "tree_cycle: FINISH.");
            tree_state_ptr_->action_name = "finish";
            tree_state_ptr_->action_phase = kios::ActionPhase::FINISH;
            skill_parameter_ = {};
            // * all tasks in tree finished. first send request to finish all actions at mios side.
            // * stop the tasks on mios side.
            if (!send_command_request(kios::CommandType::STOP_OLD_TASK, 1000, 1000))
            {
                RCLCPP_ERROR(this->get_logger(), "tree_cycle at FINISH: failed when sending stop request.");
                switch_tree_phase("ERROR", tree_phase_);
            }
            else
            {
                switch_power(false);
            }
            break;
        }

        default: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: UNDEFINED TREE PHASE!");
            // * default undefined phase handler.
            switch_power(false);
            break;
        }
        }
    }

    /**
     * @brief tree's execution method, called when RESUME and SUCCESS
     *
     */
    void execute_tree()
    {
        // *update the isMiosSuccess flag in context.
        task_state_ptr_->isActionSuccess = isActionSuccess_;

        // *tick the tree first
        RCLCPP_INFO(this->get_logger(), "execute_tree: tick once.");
        tick_result = m_tree_root->tick_once();

        // *check result
        if (is_tree_running())
        {
            // * tree is running. update action phase and check.
            // print check
            RCLCPP_WARN_STREAM(
                this->get_logger(),
                "CHECK ACTION CURRENT - " << tree_state_ptr_->action_name << " VS. LAST - " << tree_state_ptr_->last_action_name);

            if (check_action_switch())
            {
                // pause to send request
                switch_tree_phase("PAUSE", tree_phase_);
                // * update the tree_phase in BT. (TRY REMOVE THIS.)
                tree_state_ptr_->tree_phase = tree_phase_;

                // * get the parameter of the acion node (skill)
                RCLCPP_INFO_STREAM(this->get_logger(), "fetch skill parameter.");
                if (!send_fetch_skill_parameter_request(1000, 1000))
                {
                    switch_tree_phase("ERROR", tree_phase_);
                    return;
                }
                std::cout << "skill parameter: " << skill_parameter_.dump() << std::endl;
                if (!send_command_request(kios::CommandType::STOP_OLD_START_NEW, 1000, 1000))
                {
                    switch_tree_phase("ERROR", tree_phase_);
                }
            }
            else
            {
            }
        }
        else
        {
            // tree phase switch is already handled in is_tree_running(). here pass...
        }
    }

    /**
     * @brief check if an action switch should be done.
     *
     * @return true
     * @return false
     */
    bool check_action_switch()
    {
        // for the situation that the next action node's AP is the same as the last one:
        if (tree_state_ptr_->isSucceeded)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "check_action_swtich: consume isSucceeded.");

            // consume this flag
            tree_state_ptr_->isSucceeded = false;

            // * action switch
            RCLCPP_INFO_STREAM(this->get_logger(), "execute_tree: " + tree_state_ptr_->last_action_name + " succeeds. Swtich to " + tree_state_ptr_->action_name);
            // update the last action properties
            tree_state_ptr_->last_action_name = tree_state_ptr_->action_name;
            tree_state_ptr_->last_action_phase = tree_state_ptr_->action_phase;
            tree_state_ptr_->last_node_archive = tree_state_ptr_->node_archive;

            return true;
        }
        // for the situation that they are different.
        if (tree_state_ptr_->action_phase != tree_state_ptr_->last_action_phase)
        {
            // * action switch
            RCLCPP_INFO_STREAM(this->get_logger(), "execute_tree: Swtich normally to " + tree_state_ptr_->action_name);

            // update the last action properties
            tree_state_ptr_->last_action_name = tree_state_ptr_->action_name;
            tree_state_ptr_->last_action_phase = tree_state_ptr_->action_phase;
            tree_state_ptr_->last_node_archive = tree_state_ptr_->node_archive;

            return true;
        }
        return false;
    }

    /**
     * @brief update the object with GetObjectRequest client
     * @return true
     * @return false
     */
    bool update_object(int ready_deadline = 100, int response_deadline = 1000)
    {
        // * send request to update the object
        auto request = std::make_shared<kios_interface::srv::GetObjectRequest::Request>();
        while (!get_object_client_->wait_for_service(std::chrono::milliseconds(ready_deadline)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                switch_power(false);
                return false;
            }
            RCLCPP_ERROR(this->get_logger(), "service get_object_service is timeout for getting ready!");
            switch_power(false);
            return false;
        }
        auto result_future = get_object_client_->async_send_request(request);
        std::future_status status = result_future.wait_until(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(response_deadline));
        if (status == std::future_status::ready)
        {
            auto result = result_future.get();
            if (result->is_accepted == true)
            {
                RCLCPP_INFO(this->get_logger(), "get_object_service: Service call succeeded.");
                std::unordered_map<std::string, kios::Object> object_dict_;
                try
                {
                    // std::cout << "READ START" << std::endl;
                    for (int i = 0; i < result->object_name.size(); i++)
                    {
                        auto p = std::make_pair(result->object_name[i], kios::Object::from_json(nlohmann::json::parse(result->object_data[i])));
                        object_dict_.emplace(p);
                    }
                    // std::cout << "SWAP" << std::endl;
                    // * update the object dictionary in task state
                    task_state_ptr_->object_dictionary.swap(object_dict_);
                    // std::cout << "READ FINISH" << std::endl;
                    return true;
                }
                catch (...)
                {
                    RCLCPP_FATAL(this->get_logger(), "get_object_service: ERROR IN JSON FILE PARSING!");
                    return false;
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "get_object_service: Service call failed! Error message:" << result->error_message);
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "get_object_service: Service call timed out!");
            return false;
        }
    }

    /**
     * @brief send a request to tactician for archiving the nodes in the current BT.
     *
     * @param ready_deadline
     * @param response_deadline
     * @return true
     * @return false
     */
    bool load_node_archive(int ready_deadline = 100, int response_deadline = 1000)
    {
        auto service_name = archive_action_client_->get_service_name();
        // * send request to update the object
        auto request = std::make_shared<kios_interface::srv::ArchiveActionRequest::Request>();
        // update the archive node list
        request->archive_list = node_archive_list_;
        while (!archive_action_client_->wait_for_service(std::chrono::milliseconds(ready_deadline)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                switch_power(false);
                return false;
            }
            RCLCPP_ERROR_STREAM(this->get_logger(), "service " << service_name << "is timeout for getting ready!");
            switch_power(false);
            return false;
        }
        auto result_future = archive_action_client_->async_send_request(request);
        std::future_status status = result_future.wait_until(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(response_deadline));
        if (status == std::future_status::ready)
        {
            auto result = result_future.get();
            if (result->is_accepted == true)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Service " << service_name << ": Service call succeeded.");
                return true;
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Service " << service_name << ": Service call failed! Error message: " << result->error_message);
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "get_object_service: Service call timed out!");
            return false;
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