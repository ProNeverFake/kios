#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <any>
#include <unordered_map>
#include <string>

#include "spdlog/spdlog.h"

#include "behavior_tree/action_node/approach.hpp"
#include "behavior_tree/action_node/fit.hpp"
#include "behavior_tree/action_node/align.hpp"
#include "behavior_tree/action_node/push.hpp"
#include "behavior_tree/action_node/reach.hpp"

#include "behavior_tree/action_node/action_context.hpp"
#include "behavior_tree/tree_map.hpp"

// BB CODE
namespace Insertion
{
    BT::BehaviorTreeFactory factory;
    void register_node()
    {
        // static GripperInterface grip_singleton;
        // factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
        // factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &grip_singleton));
        factory.registerNodeType<Approach>("Approach");
        factory.registerNodeType<Reach>("Reach");
    }

    BT::Tree the_tree = factory.createTreeFromText(test_tree);

    class TreeRoot
    {
    public:
        TreeRoot::TreeRoot(std::shared_ptr<BT::Tree> behavior_tree, std::shared_ptr<ActionNodeContext> action_node_context);

        BT::NodeStatus tick_once();
        BT::NodeStatus tick_while_running();
        BT::NodeStatus get_tick_result(); // !
        // std::shared_ptr<ActionNodeContext> get_context_ptr();

    private:
        std::shared_ptr<BT::Tree> m_tree_ptr;
        BT::NodeStatus m_tick_result;
        std::shared_ptr<ActionNodeContext> m_context_ptr;
    };

    TreeRoot tree_root(std::make_shared<BT::Tree>(the_tree), std::make_shared<ActionNodeContext>(node_context));

} // namespace Insertion

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
