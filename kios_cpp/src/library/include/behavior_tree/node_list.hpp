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
#include "behavior_tree/action_node/contact.hpp"
#include "behavior_tree/action_node/wiggle.hpp"

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
        std::shared_ptr<RobotState> get_state_ptr();
        bool is_action_switch();
        void update_state();

    private:
        std::string m_current_action_name = "dummy_action";
        BT::BehaviorTreeFactory m_factory;
        BT::Tree m_tree;
        std::shared_ptr<ActionNodeContext> m_context_ptr;
        std::shared_ptr<RobotState> m_state_ptr;
    };

} // namespace Insertion

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
