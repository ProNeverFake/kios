#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <optional>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "behavior_tree/tree_root.hpp"
#include "kios_utils/kios_utils.hpp"
#include "kios_communication/udp.hpp"

#include "kios_interface/msg/tree_state.hpp"
#include "kios_interface/msg/task_state.hpp"
#include "kios_interface/msg/skill_context.hpp"
#include "kios_interface/srv/get_object_request.hpp"
#include "kios_interface/srv/switch_action_request.hpp"

using std::placeholders::_1;

class TreeNode : public rclcpp::Node
{
public:
    TreeNode()
        : Node("tree_node"),
          tree_state_ptr_(),
          task_state_ptr_(),
          tree_phase_(),
          isActionSuccess_(false)
    {
        // initialize object list
        object_list_.push_back("contact");
        object_list_.push_back("approach");

        //* declare mission parameter
        this->declare_parameter("is_update_object", false);
        this->declare_parameter("is_mission_success", false);
        this->declare_parameter("power", true);
        this->declare_parameter("pub_power", true);
        this->declare_parameter("client_power", true);

        //* initialize the callback groups
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        subscription_callback_group_ = timer_callback_group_;

        publisher_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        client_callback_group_ = publisher_callback_group_;

        // * initialize the tree_root
        m_tree_root = std::make_shared<Insertion::TreeRoot>(tree_state_ptr_, task_state_ptr_);

        // * Set qos and options
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = subscription_callback_group_;
        rclcpp::PublisherOptions publisher_options;
        publisher_options.callback_group = publisher_callback_group_;

        // * initialize the callbacks
        subscription_ = this->create_subscription<kios_interface::msg::TaskState>(
            "mios_state_topic",
            qos,
            std::bind(&TreeNode::subscription_callback, this, _1),
            subscription_options);
        publisher_ = this->create_publisher<kios_interface::msg::TreeState>(
            "tree_state_topic",
            qos,
            publisher_options);
        get_object_client_ = this->create_client<kios_interface::srv::GetObjectRequest>(
            "get_object_service",
            rmw_qos_profile_services_default,
            client_callback_group_);
        switch_action_client_ = this->create_client<kios_interface::srv::SwitchActionRequest>(
            "switch_action_service",
            rmw_qos_profile_services_default,
            client_callback_group_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TreeNode::timer_callback, this),
            timer_callback_group_);
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
    // flag
    bool isActionSuccess_;

    // thread rel
    std::mutex tree_mtx_;

    // * UDP socket rel
    std::shared_ptr<kios::BTReceiver> udp_socket_;
    bool isUdpReady;

    // tree rel
    kios::TreePhase tree_phase_;
    std::shared_ptr<kios::TreeState> tree_state_ptr_;
    std::shared_ptr<kios::TaskState> task_state_ptr_;

    // object list
    std::vector<std::string> object_list_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr publisher_callback_group_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<kios_interface::msg::TaskState>::SharedPtr subscription_;
    rclcpp::Publisher<kios_interface::msg::TreeState>::SharedPtr publisher_;
    rclcpp::Client<kios_interface::srv::GetObjectRequest>::SharedPtr get_object_client_;
    rclcpp::Client<kios_interface::srv::SwitchActionRequest>::SharedPtr switch_action_client_;

    // behavior tree rel
    std::shared_ptr<Insertion::TreeRoot>
        m_tree_root;
    BT::NodeStatus tick_result;

    /**
     * @brief update the task_state. here the lock priority is lower than timer.
     *
     * @param msg
     */
    void subscription_callback(const kios_interface::msg::TaskState::SharedPtr msg)
    {
        std::unique_lock<std::mutex> lock(tree_mtx_, std::try_to_lock);
        if (lock.owns_lock())
        {
            // ! BBDEBUG maybe lock will fail for a long time.
            // ! check the execution speed of the tree.
            RCLCPP_INFO_STREAM(this->get_logger(), "subscription listened: " << msg->tf_f_ext_k[2]);
            m_tree_root->get_task_state_ptr()->tf_f_ext_k = std::move(msg->tf_f_ext_k);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "SUBSCRIPTION: LOCK FAILED. PASS.");
        }
    }

    /**
     * @brief THE MAINLINE OF THE NODE.
     *
     */
    void timer_callback()
    {
        if (check_power())
        {
            // * check the necessity of updating objects
            if (this->get_parameter("is_update_object").as_bool() == true)
            {
                RCLCPP_INFO(this->get_logger(), "update the object...");
                if (update_object() == false)
                {
                    switch_tree_phase("ERROR");
                }
            }

            // * get mios skill execution state
            std::string message;
            if (udp_socket_->get_message(message) == true)
            {
                if (!switch_tree_phase(message))
                {
                    RCLCPP_ERROR(this->get_logger(), "switch_tree_phase: TREE PHASE UNDEFINED!");
                    // * turn off the tree node for debug.
                    switch_tree_phase("ERROR");
                }
            }

            // * lock tree first
            std::lock_guard<std::mutex> lock(tree_mtx_);
            // * do tree cycle
            tree_cycle();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "POWER OFF, TIMER PASS...");
        }
    }

    /**
     * @brief publisher inline method. currently off.
     * ! OUT OF DATE
     */
    void publish_tree_state()
    {
        if (this->get_parameter("pub_power").as_bool() == true)
        {
            kios_interface::msg::TreeState msg;
            msg.action_name = tree_state_ptr_->action_name;
            msg.action_phase = static_cast<int32_t>(tree_state_ptr_->action_phase);
            msg.is_runnning = true;
            RCLCPP_INFO(this->get_logger(), "Tree action node name: %s.", msg.action_name.c_str());
            publisher_->publish(msg);
        }
        else
        {
            // publisher off, do nothing.
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
        switch (tick_result)
        {
        case BT::NodeStatus::RUNNING: {
            return true;
        };
        case BT::NodeStatus::SUCCESS: {
            RCLCPP_INFO(this->get_logger(), "MISSION SUCCEEDS.");
            std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("is_mission_success", true)};
            this->set_parameters(all_new_parameters);
            switch_tree_phase("FINISH");
            return false;
        };
        default: {
            RCLCPP_ERROR(this->get_logger(), "UNDEFINED BEHAVIOR!");
            switch_tree_phase("ERROR");
            return false;
        }
        }
    }

    /**
     * @brief method to switch tree state.
     *
     * @param phase
     * @return true
     * @return false if asked to switch to an undefined phase.
     */
    bool switch_tree_phase(const std::string &phase)
    {
        if (phase == "RESUME")
        {
            tree_phase_ = kios::TreePhase::RESUME;
            return true;
        }
        if (phase == "PAUSE")
        {
            tree_phase_ = kios::TreePhase::PAUSE;
            return true;
        }
        if (phase == "FAILURE")
        {
            tree_phase_ = kios::TreePhase::FAILURE;
            return true;
        }
        if (phase == "SUCCESS")
        {
            tree_phase_ = kios::TreePhase::SUCCESS;
            return true;
        }
        if (phase == "IDLE")
        {
            tree_phase_ = kios::TreePhase::IDLE;
            return true;
        }
        if (phase == "FINISH")
        {
            tree_phase_ = kios::TreePhase::FINISH;
        }
        if (phase == "ERROR")
        {
            tree_phase_ = kios::TreePhase::ERROR;
        }
        return false;
    }

    inline void tree_cycle()
    {
        switch (tree_phase_)
        {
        case kios::TreePhase::PAUSE: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: PAUSE.");
            // * tree is waiting for resume signal from mios. reset action success flag. skip tree tick.
            isActionSuccess_ = false;
            break;
        }
        case kios::TreePhase::RESUME: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: RESUME.");
            // * normal phase. execute the tree.isActionSuccess_ = true;
            execute_tree();
            break;
        }
        case kios::TreePhase::SUCCESS: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: SUCCESS!");
            // * execute the tree with mios success flag.
            isActionSuccess_ = true;
            execute_tree();
            break;
        }
        case kios::TreePhase::ERROR: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: ERROR!");
            // * error at tree side. turn off for debug.
            switch_power(false);
            break;
        }
        case kios::TreePhase::FAILURE: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: FAILURE!");
            // * failure at mios side. turn off for debug.
            // TODO handle the result from commander.
            switch_power(false);
            break;
        }
        case kios::TreePhase::IDLE: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: IDLE.");
            // initial phase. do nothing.
            break;
        }
        case kios::TreePhase::FINISH: {
            RCLCPP_ERROR(this->get_logger(), "tree_cycle: FINISH.");
            // * all tasks in tree finished. turn off for check.
            switch_power(false);
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
     * @brief tree's execution method, called when RESUME
     *
     */
    inline void execute_tree()
    {
        // *update the isMiosSuccess flag in context.
        task_state_ptr_->isActionSuccess = isActionSuccess_;

        // *tick the tree first
        tick_result = m_tree_root->tick_once();
        RCLCPP_INFO(this->get_logger(), "execute_tree: tick once.");
        // *check result
        if (is_tree_running())
        {
            // * publish the tree state
            publish_tree_state(); // ! currently off
            // * tree is running. update action phase and check.

            if (tree_state_ptr_->action_phase != tree_state_ptr_->last_action_phase)
            {
                // * action switch
                RCLCPP_INFO(this->get_logger(), "execute_tre: AP switch hit.");
                // update the last action phase
                tree_state_ptr_->last_action_phase = tree_state_ptr_->action_phase;
                switch_tree_phase("PAUSE");
                // * call service
                if (!send_switch_action_request())
                {
                    switch_tree_phase("ERROR");
                }
            }
        }
        else
        {
            // switch tree phase already called in is_tree_running
        }
    }

    /**
     * @brief send switch_action_request to tactician for generating the action parameter.
     *
     * @return true
     * @return false if send failed
     */
    bool send_switch_action_request()
    {
        // * send request to update the object
        auto request = std::make_shared<kios_interface::srv::SwitchActionRequest::Request>();
        request->action_name = tree_state_ptr_->action_name;
        request->action_phase = static_cast<int32_t>(tree_state_ptr_->action_phase);
        request->is_interrupted = true; // ! temp
        while (!switch_action_client_->wait_for_service(std::chrono::milliseconds(50)))
        {
            RCLCPP_ERROR(this->get_logger(), "service %s not available.", switch_action_client_->get_service_name());
            return false;
        }
        auto result_future = switch_action_client_->async_send_request(request);
        std::future_status status = result_future.wait_until(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(50));
        if (status == std::future_status::ready)
        {
            auto result = result_future.get();
            if (result->is_accepted == true)
            {
                RCLCPP_INFO(this->get_logger(), "Service %s response: request accepted.", switch_action_client_->get_service_name());
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Service %s response: request refused!", switch_action_client_->get_service_name());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "UNKNOWN ERROR: service %s future is available but not ready.", switch_action_client_->get_service_name());
            return false;
        }
    }

    /**
     * @brief update the object with GetObjectRequest client
     * ! UNFINISHED
     * @return true
     * @return false
     */
    bool update_object()
    {
        // * send request to update the object
        auto request = std::make_shared<kios_interface::srv::GetObjectRequest::Request>();
        request->object_list = object_list_;
        while (!get_object_client_->wait_for_service(std::chrono::milliseconds(50)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service get_object_service not available, waiting ...");
        }
        auto result_future = get_object_client_->async_send_request(request);
        std::future_status status = result_future.wait_until(
            std::chrono::steady_clock::now() + std::chrono::seconds(1));
        if (status == std::future_status::ready)
        {
            auto result = result_future.get();
            if (result->is_success == true)
            {
                RCLCPP_INFO(this->get_logger(), "get_object_service: Service call succeeded.");
                try
                {
                    nlohmann::json object_data = nlohmann::json::parse(result->object_data);
                }
                catch (...)
                {
                    RCLCPP_FATAL(this->get_logger(), "get_object_service: ERROR IN JSON FILE PARSING!");
                    return false;
                }
                // ! TODO handle the object_data
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "get_object_service: Service call failed! Error message: %s", result->error_message);
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "get_object_service: Service call timed out!");
            return false;
        }

        // * reset the flag in node param
        std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("is_update_object", false)};
        this->set_parameters(all_new_parameters);
        return true;
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