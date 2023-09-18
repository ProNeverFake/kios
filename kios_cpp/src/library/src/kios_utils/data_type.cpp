#include "kios_utils/data_type.hpp"

namespace kios
{
    std::optional<std::string> action_phase_to_str(const ActionPhase &action_phase)
    {
        static const std::unordered_map<ActionPhase, std::string> action_phase_to_str_map{
            {ActionPhase::INITIALIZATION, "initialization"},
            {ActionPhase::APPROACH, "approach"},
            {ActionPhase::CONTACT, "contact"},
            {ActionPhase::WIGGLE, "wiggle"},
            {ActionPhase::JOINT_MOVE, "joint_move"},
            {ActionPhase::CARTESIAN_MOVE, "cartesian_move"},
            {ActionPhase::GRIPPER_FORCE, "gripper_force"},
            {ActionPhase::GRIPPER_MOVE, "gripper_move"},
        };

        auto it = action_phase_to_str_map.find(action_phase);
        if (it != action_phase_to_str_map.end())
        {
            return it->second;
        }
        else
        {
            return {};
        }
    }

    std::optional<ActionPhase> action_phase_from_str(const std::string &str)
    {
        static const std::unordered_map<std::string, ActionPhase> str_to_action_phase{
            {"initialization", ActionPhase::INITIALIZATION},
            {"approach", ActionPhase::APPROACH},
            {"contact", ActionPhase::CONTACT},
            {"wiggle", ActionPhase::WIGGLE},
            {"joint_move", ActionPhase::JOINT_MOVE},
            {"cartesian_move", ActionPhase::CARTESIAN_MOVE},
            {"gripper_force", ActionPhase::GRIPPER_FORCE},
            {"gripper_move", ActionPhase::GRIPPER_MOVE},
        };

        auto it = str_to_action_phase.find(str);
        if (it != str_to_action_phase.end())
        {
            return it->second;
        }
        else
        {
            return {};
        }
    }
} // namespace kios
