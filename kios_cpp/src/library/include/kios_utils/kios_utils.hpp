#pragma once

#include <any>
#include <unordered_map>
#include <string>
#include "nlohmann/json.hpp"

#include "kios_utils/data_type.hpp"
#include "kios_utils/object.hpp"
#include "kios_utils/parameters.hpp"

#include "mirmi_utils/math.hpp"

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

    std::string tree_phase_to_str(const TreePhase &tree_phase);

    /**
     * @brief method to switch tree state.
     *
     * @param phase
     * @return true
     * @return false if asked to switch to an undefined phase.
     */
    bool switch_tree_phase(const std::string &phase, TreePhase &tree_phase);

    bool compare_homogeneous_matrix(const Eigen::Matrix<double, 4, 4> &T_1, const Eigen::Matrix<double, 4, 4> &T_2, double angular_threshold = 0.05, double translation_threshold = 0.01);
    // ! TODO
    bool compare_homogeneous_matrix_translation(const Eigen::Matrix<double, 4, 4> &T_1, const Eigen::Matrix<double, 4, 4> &T_2, double angular_threshold = 0.05, double translation_threshold = 0.01);

} // namespace kios
