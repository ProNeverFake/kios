#include "behavior_tree/action_node/contact.hpp"

namespace Insertion
{
    Contact::Contact(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : HyperMetaNode<BT::StatefulActionNode>(name, config, tree_state_ptr, task_state_ptr)
    {
        // initialize local context
        node_context_initialize();
    }

    /**
     * @brief Here to apply the success condition check.
     *
     * @return true
     * @return false
     */
    bool Contact::is_success()
    {
        // spdlog::info("CONTACT IS_SUCCESS?");

        // std::cerr << "CHECK TASK STATE CONTENT" << std::endl;
        // std::stringstream ss;
        // for (size_t i = 0; i < get_task_state_ptr()->tf_f_ext_k.size(); ++i)
        // {
        //     ss << get_task_state_ptr()->tf_f_ext_k[i];
        //     if (i != get_task_state_ptr()->tf_f_ext_k.size() - 1)
        //     { // if not the last element
        //         ss << ", ";
        //     }
        // }
        // std::string str = ss.str();
        // std::cout << str << std::endl;

        // std::cerr << "BEFORE CHECK CONTACT SUCCESS" << std::endl;
        if (get_task_state_ptr()->mios_state.tf_f_ext_k[2] > 7)
        {
            // std::cout << "CONTACT SUCCESS" << std::endl;
            mark_success();
            return true;
        }
        // std::cout << "CONTACT NOT SUCCESS" << std::endl;
        return false;
    }

    void Contact::update_tree_state()
    {
        std::cout << "CONTACT UPDATE TREE STATE" << std::endl;
        get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
        get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;
        get_tree_state_ptr()->object_name = get_node_context_ref().object_name;
    }

    void Contact::node_context_initialize()
    {
        std::cout << "CONTACT INITIALIZE" << std::endl;

        auto &node_context = get_node_context_ref();
        node_context.node_name = "CONTACT";
        node_context.action_name = "contact";
        node_context.object_name = "contact";
        node_context.action_phase = kios::ActionPhase::CONTACT;
        node_context.parameter["skill"]["action_name"] = "contact";
        node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::CONTACT;
    }

    BT::NodeStatus Contact::onStart()
    {
        std::cout << "CONTACT ON START" << std::endl;
        if (has_succeeded_once())
        {
            std::cout << "CONTACT HAS ONCE SUCCEEDED" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        if (is_success())
        {
            std::cout << "CONTACT ALREADY SUCCESS" << std::endl;

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "CONTACT GO RUNNING" << std::endl;

            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus Contact::onRunning()
    {
        // std::cout << "CONTACT ON RUNNING" << std::endl;
        // if (has_succeeded_once())
        // {
        //     std::cout << "CONTACT HAS ONCE SUCCEEDED == true" << std::endl;
        //     return BT::NodeStatus::SKIPPED;
        // }
        if (is_success())
        {
            std::cout << "CONTACT SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "CONTACT RUNNING" << std::endl;
            update_tree_state();
            return BT::NodeStatus::RUNNING;
        }
    }

    void Contact::onHalted()
    {
        // * interrupted behavior. do nothing.
        std::cout << "CONTACT ON HALTED" << std::endl;
    }

    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    // Contact::Contact(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
    //     : HyperMetaNode<BT::SyncActionNode>(name, config, tree_state_ptr, task_state_ptr)
    // {
    //     // initialize local context
    //     node_context_initialize();
    // }

    // /**
    //  * @brief Here to apply the success condition check.
    //  *
    //  * @return true
    //  * @return false
    //  */
    // bool Contact::is_success()
    // {
    //     std::cerr << "CHECK TASK STATE CONTENT" << std::endl;
    //     std::stringstream ss;
    //     for (size_t i = 0; i < get_task_state_ptr()->tf_f_ext_k.size(); ++i)
    //     {
    //         ss << get_task_state_ptr()->tf_f_ext_k[i];
    //         if (i != get_task_state_ptr()->tf_f_ext_k.size() - 1)
    //         { // if not the last element
    //             ss << ", ";
    //         }
    //     }
    //     std::string str = ss.str();
    //     std::cout << str << std::endl;

    //     std::cerr << "BEFORE CHECK CONTACT SUCCESS" << std::endl;

    //     if (get_task_state_ptr()->tf_f_ext_k[2] > 7)
    //     {
    //         mark_success();
    //         std::cout << "CONTACT SUCCESS" << std::endl;
    //         return true;
    //     }
    //     std::cout << "CONTACT NOT SUCCESS" << std::endl;
    //     return false;
    // }

    // void Contact::update_tree_state()
    // {
    //     std::cout << "APPROACH UPDATE TREE STATE" << std::endl;
    //     get_tree_state_ptr()->action_name = get_node_context_ref().action_name;
    //     get_tree_state_ptr()->action_phase = get_node_context_ref().action_phase;
    //     std::cout << "UPDATED VALUE: " << get_tree_state_ptr()->action_name << std::endl;
    // }

    // void Contact::node_context_initialize()
    // {
    //     auto &node_context = get_node_context_ref();
    //     node_context.node_name = "CONTACT";
    //     node_context.action_name = "contact";
    //     node_context.action_phase = kios::ActionPhase::CONTACT;
    //     node_context.parameter["skill"]["action_name"] = "contact";
    //     node_context.parameter["skill"]["action_phase"] = kios::ActionPhase::CONTACT;
    // }

    // BT::NodeStatus Contact::tick()
    // {
    //     if (has_succeeded_once())
    //     {
    //         return BT::NodeStatus::SUCCESS;
    //     }

    //     if (is_success())
    //     {
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     else
    //     {
    //         update_tree_state();
    //         return BT::NodeStatus::RUNNING;
    //     }
    // }

} // namespace Insertion
