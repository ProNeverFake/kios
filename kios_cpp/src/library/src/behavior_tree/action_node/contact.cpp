#include "behavior_tree/action_node/contact.hpp"

namespace Insertion
{
    Contact::Contact(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::ActionPhaseContext> context_ptr, std::shared_ptr<kios::RobotState> state_ptr)
        : MetaNode(name, config)
    {
        m_node_context_ptr = context_ptr;
        m_robot_state_ptr = state_ptr;
    }

    // BT::PortsList Contact::providedPorts()
    // {
    //     // amount of milliseconds that we want to sleep

    //     return {BT::InputPort<std::vector<double>>("target_position")};
    // }
    /**
     * @brief Here to apply the success condition check.
     *
     * @return true
     * @return false
     */
    bool Contact::is_success()
    {
        // TODO
        if (m_robot_state_ptr->TF_F_ext_K[2] > 5)
        {
            return true;
        }
        else
        {
            return false;
        }
        // return m_robot_state_ptr->is_contact_finished;
    }
    /**
     * @brief temporarily just set action_name
     *
     */
    void Contact::set_action_context()
    {
        m_node_context_ptr->parameter["skill"]["action_context"]["action_name"] = "contact";
        m_node_context_ptr->action_phase = kios::ActionPhase::CONTACT;
        m_node_context_ptr->action_name = "contact";
        m_node_context_ptr->parameter["skill"]["action_context"]["action_phase"] = kios::ActionPhase::CONTACT;
    }
    void Contact::node_context_initialize()
    {
        std::shared_ptr<kios::ActionPhaseContext> context_ptr = get_context_ptr();
        context_ptr->node_name = "contact";
        // todo add more command context here.
    }

    BT::NodeStatus Contact::onStart()
    {
        // getInput("target_position", target_position);
        if (is_success())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            set_action_context();
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus Contact::onRunning()
    {
        if (is_success())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    void Contact::onHalted()
    {
        // nothing to do here...
        std::cout << "Action stoped" << std::endl;
    }

} // namespace Insertion
