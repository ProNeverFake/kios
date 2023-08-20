#include "behavior_tree/action_node/meta_node.hpp"

namespace Insertion
{
    // * MongoInterface
    MongoInterface::MongoInterface()
    {
        data_pool_register(m_entity_list);
        data_pool_update();
    }
    void MongoInterface::data_pool_register(std::vector<std::string> &reg_list)
    {
        // todo: check the reg_list
        for (auto key : reg_list)
        {
            // register a empty pair
            m_data_pool[key] = std::any();
        }
    }
    void MongoInterface::data_pool_update()
    {
        // TODO a real "update" method
        std::vector<double> tp = {0, 0, 0, 0, 0, 0, 0};
        m_data_pool["target_position"] = tp;
        m_data_pool["reach_force"] = double(7);
    }
    void MongoInterface::data_pool_fetch()
    {
        // TODO: fetch data from MongoDB
    }
    std::any MongoInterface::data_pool_lookup(const std::string &keyword) const
    {
        // Lookup the entity associated with the keyword in data_pool
        auto it = m_data_pool.find(keyword);
        if (it != m_data_pool.end())
        {
            return it->second;
        }
        else
        {
            return std::any(); // Return empty std::any if the keyword is not found
        }
    }

    // * MetaNode
    BT::PortsList MetaNode::providedPorts()
    {
        // amount of milliseconds that we want to sleep

        return {BT::InputPort<std::vector<double>>("target_position")};
    }

    std::shared_ptr<kios::ActionPhaseContext> MetaNode::get_context_ptr()
    {
        return std::make_shared<kios::ActionPhaseContext>(m_context);
    }

    void MetaNode::node_context_initialize()
    {}

    bool MetaNode::is_success()
    {
        return true;
    }

    nlohmann::json MetaNode::get_action_parameter()
    {
        return action_parameter;
    }
    void MetaNode::set_action_parameter(nlohmann::json parameter)
    {
        action_parameter = parameter;
    }

    /**
     * @brief default action parameter initializer.
     *
     */
    void MetaNode::action_parameter_initialize()
    {
        action_parameter = {
            {"skill",
             {{"objects",
               {{"Container", "housing"},
                {"Approach", "app1"},
                {"Insertable", "ring"}}},
              {"time_max", 17},
              {"action_context",
               {{"action_name", "dummy_action"},
                {"action_phase", kios::ActionPhase::INITIALIZATION}}},
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
    }
} // namespace Insertion
