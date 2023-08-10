#include "behavior_tree/action_node/approach.hpp"

namespace Insertion
{
    Approach::Approach(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<ActionNodeContext> context_ptr, std::shared_ptr<RobotState> state_ptr)
        : MetaNode(name, config)
    {
        m_node_context_ptr = context_ptr;
        m_robot_state_ptr = state_ptr;
    }

    BT::PortsList Approach::providedPorts()
    {
        // amount of milliseconds that we want to sleep

        return {BT::InputPort<std::vector<double>>("target_position")};
    }
    /**
     * @brief Here to apply the success condition check.
     *
     * @return true
     * @return false
     */
    bool Approach::is_success()
    {
        // ! BUG
        return m_robot_state_ptr->is_approach_finished;
    };
    /**
     * @brief temporarily just set action_name
     *
     */
    void Approach::set_action_context()
    {
        m_node_context_ptr->parameter["skill"]["action_name"] = "approach";
        m_node_context_ptr->action_name = "approach";
        m_node_context_ptr->action_phase = ActionPhase::APPROACH;
        m_node_context_ptr->parameter["skill"]["action_phase"] = ActionPhase::APPROACH;
    }
    void Approach::node_context_initialize()
    {
        std::shared_ptr<ActionNodeContext> context_ptr = get_context_ptr();
        context_ptr->node_name = "approach";
        // todo add more command context here.
    };

    BT::NodeStatus Approach::onStart()
    {
        // getInput("target_position", target_position);
        // * get the current state from data_pool class
        // * check the state
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
    BT::NodeStatus Approach::onRunning()
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

    void Approach::onHalted()
    {
        // nothing to do here...
        std::cout << "Action stoped" << std::endl;
    }
    void Approach::action_parameter_initialize()
    {
        nlohmann::json parameter = {
            {"skill",
             {{"objects",
               {{"Container", "housing"},
                {"Approach", "app1"},
                {"Insertable", "ring"}}},
              {"time_max", 17},
              {"action_context",
               {{"action_name", "dummy_action"},
                {"action_phase", ActionPhase::INITIALIZATION}}},
              {"p0",
               {{"dX_d", {0.1, 1}},
                {"ddX_d", {0.5, 4}},
                {"DeltaX", {0, 0, 0, 0, 0, 0}},
                {"K_x", {1500, 1500, 1500, 600, 600, 600}}}},
              {"p1",
               {{"dX_d", {0.03, 0.1}},
                {"ddX_d", {0.5, 0.1}},
                {"K_x", {500, 500, 500, 600, 600, 600}}}},
              {"p2",
               {{"search_a", {10, 10, 0, 2, 2, 0}},
                {"search_f", {1, 1, 0, 1.2, 1.2, 0}},
                {"search_phi", {0, 3.14159265358979323846 / 2, 0, 3.14159265358979323846 / 2, 0, 0}},
                {"K_x", {500, 500, 500, 800, 800, 800}},
                {"f_push", {0, 0, 7, 0, 0, 0}},
                {"dX_d", {0.1, 0.5}},
                {"ddX_d", {0.5, 1}}}},
              {"p3",
               {{"dX_d", {0.1, 0.5}},
                {"ddX_d", {0.5, 1}},
                {"f_push", 7},
                {"K_x", {500, 500, 0, 800, 800, 800}}}}}},
            {"control",
             {{"control_mode", 0}}},
            {"user",
             {{"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
              {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
              {"F_ext_contact", {3.0, 2.0}}}}};
        // set the action parameter with set method.
        set_action_parameter(parameter);
    }

} // namespace Insertion
