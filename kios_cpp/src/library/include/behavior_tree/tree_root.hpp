#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <string>
#include <memory>

#include "spdlog/spdlog.h"

#include "behavior_tree/meta_node/meta_node.hpp"
#include "behavior_tree/tree_map.hpp"

#include "behavior_tree/action_node/approach.hpp"
#include "behavior_tree/action_node/contact.hpp"
#include "behavior_tree/action_node/wiggle.hpp"

#include "behavior_tree/condition_node/condition_node.hpp"

// BB CODE
namespace Insertion
{
    class TreeRoot
    {
    public:
        TreeRoot(std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);
        void initialize_tree();
        BT::NodeStatus tick_once();
        BT::NodeStatus tick_while_running();
        BT::NodeStatus get_tick_result();
        std::shared_ptr<kios::TreeState> get_tree_state_ptr();
        std::shared_ptr<kios::TaskState> get_task_state_ptr();
        bool is_switch_action(); // ! discarded
        void update_state();     // ! discarded

    private:
        std::string current_action_name_ = "dummy_action";
        BT::BehaviorTreeFactory factory_;
        BT::Tree tree_;
        std::shared_ptr<kios::TreeState> tree_state_ptr_;
        std::shared_ptr<kios::TaskState> task_state_ptr_;
    };

} // namespace Insertion

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
