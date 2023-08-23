#include "behavior_tree/meta_node/meta_node.hpp"

namespace Insertion
{
    class HasObject : public HyperMetaNode<BT::ConditionNode>
    {
    public:
        HasObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);
        BT::NodeStatus tick() override;
        bool is_success() override;

        // empty override
        void update_tree_state() override{};
        void node_context_initialize() override{};

    private:
    };

    class AtPosition : public HyperMetaNode<BT::ConditionNode>
    {
    public:
        AtPosition(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);
        BT::NodeStatus tick() override;
        bool is_success() override;

        // empty override
        void update_tree_state() override{};
        void node_context_initialize() override{};

    private:
    };
} // namespace Insertion
