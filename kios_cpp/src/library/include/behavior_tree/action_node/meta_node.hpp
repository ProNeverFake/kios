#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <any>
#include <unordered_map>
#include <string>

#include "spdlog/spdlog.h"

#include "kios_utils/kios_utils.hpp"

namespace Insertion
{

    /**
     * @brief a meta node class for insertion behavior tree node
     *
     */
    class MetaNode : public BT::StatefulActionNode
    {
    public:
        MetaNode(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
            : BT::StatefulActionNode(name, config),
              tree_state_ptr_(tree_state_ptr),
              task_state_ptr_(task_state_ptr),
              node_context_()
        {
        }
        static BT::PortsList providedPorts();

        // shared context
        std::shared_ptr<kios::TreeState> get_tree_state_ptr();
        std::shared_ptr<kios::TaskState> get_task_state_ptr();

        // local context
        kios::ActionPhaseContext &get_node_context_ref();

        // * update tree state with this node's context
        virtual void update_tree_state() = 0;
        // * node context initializer
        virtual void node_context_initialize() = 0;

    private:
        //* shared objects among the entire tree
        std::shared_ptr<kios::TreeState> tree_state_ptr_;
        std::shared_ptr<kios::TaskState> task_state_ptr_;

        // * default values as member variable of the node
        kios::ActionPhaseContext node_context_; // default node context value

        virtual bool is_success(); // here to set the check condition
        bool is_switch_action();
    };

} // namespace Insertion