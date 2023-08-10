#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "behavior_tree/action_node/action_context.hpp"

#include <any>
#include <unordered_map>
#include <string>

#include "spdlog/spdlog.h"

namespace Insertion
{

    // TODO a data storage class --- an interface for mongoDB
    class MongoInterface
    {
    public:
        MongoInterface();
        void data_pool_register(std::vector<std::string> &reg_list);
        std::any data_pool_lookup(const std::string &keyword) const;
        void data_pool_update();
        void data_pool_fetch();

    private:
        std::unordered_map<std::string, std::any> m_data_pool;
        std::vector<std::string> m_entity_list = {
            "target_position",
            "reach_force"};
    };
    /**
     * @brief a meta node class for insertion behavior tree node
     *
     */
    class MetaNode : public BT::StatefulActionNode
    {
    public:
        MetaNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config)
        {
        }
        // choose an action depending on the current situation.
        auto chose_action(); // todo
        std::shared_ptr<ActionNodeContext> get_context_ptr();
        nlohmann::json get_action_parameter();
        void set_action_parameter(nlohmann::json parameter);

    private:
        virtual void action_parameter_initialize();
        virtual void node_context_initialize();
        virtual bool is_success();       // here to set the check condition
        MongoInterface m_data_interface; // data member
        ActionNodeContext m_context;     // node and command context
        nlohmann::json action_parameter;
    };

} // namespace Insertion