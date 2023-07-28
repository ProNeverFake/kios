#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <memory>

namespace Insertion
{
    /**
     * @brief
     *
     */
    struct ActionNodeContext
    {
        std::string node_name;
        std::string action_name;
        std::string command;
    };

    struct RobotState
    {
        std::vector<double> q;
        std::vector<double> F_ext;
        bool is_approach_finished = false;
    };

    // extern std::shared_ptr<ActionNodeContext> node_context_ptr;

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
