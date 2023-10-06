#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <string>
#include <memory>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include "behavior_tree/meta_node/meta_node.hpp"
#include "behavior_tree/tree_map.hpp"

// for demo
#include "behavior_tree/action_node/approach.hpp"
#include "behavior_tree/action_node/contact.hpp"
#include "behavior_tree/action_node/wiggle.hpp"

// general action nodes
#include "behavior_tree/action_node/cartesian_move.h"
#include "behavior_tree/action_node/joint_move.hpp"
#include "behavior_tree/action_node/gripper_force.hpp"
#include "behavior_tree/action_node/gripper_move.hpp"

#include "behavior_tree/condition_node/condition_node.hpp"

// #include "kios_utils/context_manager.hpp"
#include "kios_utils/kios_utils.hpp"

// BB CODE
namespace Insertion
{
    class TreeRoot
    {
    public:
        TreeRoot(std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);
        ~TreeRoot();
        bool initialize_tree();
        bool construct_tree(const std::string &tree_string);
        bool register_nodes();
        std::optional<std::vector<kios::NodeArchive>> archive_nodes();
        bool check_grounded_objects();

        BT::NodeStatus tick_once();
        BT::NodeStatus tick_while_running();
        BT::NodeStatus get_tick_result();
        std::shared_ptr<kios::TreeState> get_tree_state_ptr();
        std::shared_ptr<kios::TaskState> get_task_state_ptr();

    private:
        // kios::ContextClerk context_clerk_;

        // flag
        bool hasRegisteredNodes;

        // * BT rel
        BT::BehaviorTreeFactory factory_;
        BT::Tree tree_;

        // * state rel
        std::shared_ptr<kios::TreeState> tree_state_ptr_;
        std::shared_ptr<kios::TaskState> task_state_ptr_;

        void set_log();
    };

} // namespace Insertion

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
