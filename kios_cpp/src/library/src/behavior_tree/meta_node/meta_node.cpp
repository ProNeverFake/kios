#include "behavior_tree/meta_node/meta_node.hpp"

namespace Insertion
{

    // * MetaNode
    BT::PortsList MetaNode::providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return {BT::InputPort<bool>("isOnceSucceeded")};
    }

    kios::ActionPhaseContext &MetaNode::get_node_context_ref()
    {
        return node_context_;
    }

    std::shared_ptr<kios::TreeState> MetaNode::get_tree_state_ptr()
    {
        return tree_state_ptr_;
    }

    std::shared_ptr<kios::TaskState> MetaNode::get_task_state_ptr()
    {
        return task_state_ptr_;
    }

    /**
     * @brief default success: mios say success
     *
     * @return true
     * @return false
     */
    bool MetaNode::is_success()
    {
        return task_state_ptr_->isActionSuccess;
    }

    bool MetaNode::is_switch_action()
    {
    }

} // namespace Insertion
