#include "behavior_tree/action_node/meta_node.hpp"

namespace Insertion
{
    // * MongoInterface
    MongoInterface::MongoInterface()
    {
        // try
        // {
        //     data_pool_register(m_entity_list);
        //     data_pool_update();
        // }
        // catch (std::exception exception)
        // {
        //     // do nothing....
        // }
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
    std::shared_ptr<ActionNodeContext> MetaNode::get_context_ptr()
    {
        return std::make_shared<ActionNodeContext>(m_context);
    };

    void MetaNode::node_context_initialize()
    {}

    bool MetaNode::is_success()
    {
        return true;
    }
} // namespace Insertion
