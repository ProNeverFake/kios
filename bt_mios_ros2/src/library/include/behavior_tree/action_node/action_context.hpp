#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

namespace Insertion
{
    /**
     * @brief
     *
     */
    struct ActionNodeContext
    {
        std::string node_name;
        std::string command;
    };

    ActionNodeContext node_context;

    // class ActionContext
    // {
    // public:
    //     ActionContext(BT::Tree &behavior_tree);

    //     BT::NodeStatus tick_once();
    //     BT::NodeStatus get_tick_result(); // !
    //     std::shared_ptr<ActionNodeContext> get_context_ptr();

    // private:
    //     ActionNodeContext m_tree_action_context;
    // };

    // ActionContext tree_root;

} // namespace Insertion
