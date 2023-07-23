#include "behavior_tree/node_list.hpp"

// // This function must be implemented in the .cpp file to create
// // a plugin that can be loaded at run-time
// BT_REGISTER_NODES(factory)
// {
//     Insertion::RegisterNodes(factory);
// }

namespace Insertion
{
    TreeRoot::TreeRoot(std::shared_ptr<BT::Tree> behavior_tree, std::shared_ptr<ActionNodeContext> action_node_context)
    {
        m_tree_ptr = behavior_tree;
        m_context_ptr = action_node_context;
    }
    BT::NodeStatus TreeRoot::tick_once()
    {
        return m_tree_ptr->tickOnce();
    }
    BT::NodeStatus TreeRoot::tick_while_running()
    {
        return m_tree_ptr->tickWhileRunning();
    }
} // namespace Insertion

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////