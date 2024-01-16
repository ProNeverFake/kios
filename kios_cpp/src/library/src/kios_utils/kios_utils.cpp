#include "kios_utils/kios_utils.hpp"

namespace kios
{
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
        if (tree_phase == kios::TreePhase::ERROR || tree_phase == kios::TreePhase::FAILURE)
        {
            spdlog::warn("switch tree phase: An ERROR or a FAILURE phase is not handled. the current switch is skipped.");
            return false;
        }
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