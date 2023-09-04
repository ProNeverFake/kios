#include "behavior_tree/condition_node/condition_node.hpp"

namespace Insertion
{
    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    //* HAS OBJECT

    HasObject::HasObject(const std::string &name, const BT::NodeConfig &config, std::string obj_name, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : HyperMetaNode<BT::ConditionNode>(name, config, tree_state_ptr, task_state_ptr),
          object_name(obj_name)
    {
    }
    BT::NodeStatus HasObject::tick()
    {
        if (is_success())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    bool HasObject::is_success()
    {
        auto &obj_dict = get_task_state_ptr()->object_dictionary;
        if (obj_dict.find(object_name) != obj_dict.end())
        {
            std::cout << "HasObject::" << object_name << ": YES" << std::endl;
            return true;
        }
        else
        {
            std::cerr << "HasObject::" << object_name << ": NO" << std::endl;
            return false;
        }
    }

    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    //* AT POSITION

    AtPosition::AtPosition(const std::string &name, const BT::NodeConfig &config, std::string obj_name, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : HyperMetaNode<BT::ConditionNode>(name, config, tree_state_ptr, task_state_ptr),
          object_name(obj_name)
    {
    }

    BT::NodeStatus AtPosition::tick()
    {
        if (is_success())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    /**
     * @brief compare H matrix (distance). currently using O_T_OB and T_T_EE.
     *
     * @return true
     * @return false
     */
    bool AtPosition::is_success()
    {
        // * check if object exists first (maybe duplicated)
        auto &obj_dict = get_task_state_ptr()->object_dictionary;
        auto &mios_state = get_task_state_ptr()->mios_state;
        if (obj_dict.find(object_name) != obj_dict.end())
        {
            auto &O_T_OB = obj_dict.at(object_name).O_T_OB;
            auto &T_T_EE = mios_state.t_t_ee_matrix;
            double rot_distance = mirmi_utils::get_angular_distance(O_T_OB, T_T_EE);
            double trans_distance = mirmi_utils::get_linear_distance(O_T_OB, T_T_EE);
            if (rot_distance < 0.03 && trans_distance < 0.03)
            {
                std::cout << "AtPosition::" << object_name << ": YES" << std::endl;
                return true;
            }
            else
            {
                std::cout << "AtPosition::" << object_name << ": NO" << std::endl;
                return false;
            }
        }
        else
        {
            std::cerr << "AtPosition::" << object_name << ": OBJECT NOT FIND!" << std::endl;
            get_tree_state_ptr()->tree_phase = kios::TreePhase::ERROR;
            return false;
        }
    }
} // namespace Insertion