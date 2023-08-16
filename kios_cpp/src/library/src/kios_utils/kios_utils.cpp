#include "kios_utils/kios_utils.hpp"

namespace kios
{
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

}