#include "kios_utils/kios_utils.hpp"

namespace kios
{
    // * MongoInterface
    MongoInterface::MongoInterface()
    {
        data_pool_register(m_entity_list);
        data_pool_update();
    }
    void MongoInterface::data_pool_register(std::vector<std::string> &reg_list)
    {
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

    std::string action_phase_to_str(ActionPhase action_phase)
    {
        switch (action_phase)
        {
        case ActionPhase::INITIALIZATION: {
            return "initialization";
            break;
        }

        case ActionPhase::APPROACH: {
            return "approach";
            break;
        }

        case ActionPhase::CONTACT: {
            return "contact";
            break;
        }

        case ActionPhase::WIGGLE: {
            return "wiggle";
            break;
        }

        default: {
            return "UNDEFINED ACTION PHASE";
            break;
        }
        }
        return "UNDEFINED ACTION PHASE";
    }

    std::string tree_phase_to_str(const TreePhase &tree_phase)
    {
        switch (tree_phase)
        {
        case TreePhase::ERROR:
            return "ERROR";

        case TreePhase::IDLE:
            return "IDLE";

        case TreePhase::RESUME:
            return "RESUME";

        case TreePhase::PAUSE:
            return "PAUSE";

        case TreePhase::SUCCESS:
            return "SUCCESS";

        case TreePhase::FAILURE:
            return "FAILURE";

        case TreePhase::FINISH:
            return "FINISH";

        default:
            return "UNKNOWN";
        }
    }

    /**
     * @brief method to switch tree state.
     *
     * @param phase
     * @return true
     * @return false if asked to switch to an undefined phase.
     */
    bool switch_tree_phase(const std::string &phase, TreePhase &tree_phase)
    {
        if (phase == "RESUME")
        {
            tree_phase = kios::TreePhase::RESUME;
            return true;
        }
        if (phase == "PAUSE")
        {
            tree_phase = kios::TreePhase::PAUSE;
            return true;
        }
        if (phase == "FAILURE")
        {
            tree_phase = kios::TreePhase::FAILURE;
            return true;
        }
        if (phase == "SUCCESS")
        {
            tree_phase = kios::TreePhase::SUCCESS;
            return true;
        }
        if (phase == "IDLE")
        {
            tree_phase = kios::TreePhase::IDLE;
            return true;
        }
        if (phase == "FINISH")
        {
            tree_phase = kios::TreePhase::FINISH;
            return true;
        }
        if (phase == "ERROR")
        {
            tree_phase = kios::TreePhase::ERROR;
            return true;
        }
        return false;
    }

    bool compare_homogeneous_matrix(const Eigen::Matrix<double, 4, 4> &T_1, const Eigen::Matrix<double, 4, 4> &T_2, double angular_threshold, double translation_threshold)
    {
        double angular_distance = mirmi_utils::get_angular_distance(T_1, T_2);
        double translation_distance = mirmi_utils::get_linear_distance(T_1, T_2);
    }

} // namespace kios