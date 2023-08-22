#pragma once

#include <any>
#include <unordered_map>
#include <string>
#include "nlohmann/json.hpp"

#include "kios_utils/data_type.hpp"

namespace kios
{
    // ! DISCARD
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

    std::string action_phase_to_str(ActionPhase action_phase);

    std::string tree_phase_to_str(const TreePhase &tree_phase);

} // namespace kios
