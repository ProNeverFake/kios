#pragma once

#include <any>
#include <unordered_map>
#include <string>
#include "nlohmann/json.hpp"

#include "kios_utils/data_type.hpp"
#include "kios_utils/object.hpp"
#include "kios_utils/parameters.hpp"

#include "mirmi_utils/math.hpp"
#include "kios_utils/context_manager.hpp"

namespace kios
{
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
