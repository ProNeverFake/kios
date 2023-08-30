#include "behavior_tree/meta_node/meta_node.hpp"

namespace Insertion
{
    /**
     * @brief condition node to judge if the object exists. check the object dictionary in task_state.
     * ! you must provide object name when initializing this.
     */
    class HasObject : public HyperMetaNode<BT::ConditionNode>
    {
    public:
        HasObject(const std::string &name, const BT::NodeConfig &config, std::string obj_name, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);
        BT::NodeStatus tick() override;
        bool is_success() override;

        // empty override
        void update_tree_state() override{};
        void node_context_initialize() override{};

    private:
        std::string object_name;
    };

    /**
     * @brief condition node to judge if the robot is at the position of the object. compare the current H and the object's H.
     * ! dito. And current distance difference tolerance is fixed.
     */
    class AtPosition : public HyperMetaNode<BT::ConditionNode>
    {
    public:
        AtPosition(const std::string &name, const BT::NodeConfig &config, std::string obj_name, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);
        BT::NodeStatus tick() override;
        bool is_success() override;

        // empty override
        void update_tree_state() override{};
        void node_context_initialize() override{};

    private:
        std::string object_name;
    };
} // namespace Insertion
