#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <any>
#include <unordered_map>
#include <string>
#include <memory>

#include "spdlog/spdlog.h"

#include "behavior_tree/action_node/approach.hpp"
#include "behavior_tree/action_node/fit.hpp"
#include "behavior_tree/action_node/align.hpp"
#include "behavior_tree/action_node/push.hpp"
#include "behavior_tree/action_node/reach.hpp"

#include "behavior_tree/tree_map.hpp"

// BB CODE
namespace Insertion
{
    class TreeRoot
    {
    public:
        TreeRoot();
        void register_node();
        void initialize_tree();
        BT::NodeStatus tick_once();
        BT::NodeStatus tick_while_running();
        BT::NodeStatus get_tick_result();
        std::shared_ptr<ActionNodeContext> get_context_ptr();

    private:
        BT::BehaviorTreeFactory m_factory;
        BT::Tree m_tree;
        std::shared_ptr<ActionNodeContext> m_context_ptr;
    };

} // namespace Insertion

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
