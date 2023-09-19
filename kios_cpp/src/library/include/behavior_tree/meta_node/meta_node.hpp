#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <any>
#include <unordered_map>
#include <string>

#include "spdlog/spdlog.h"

#include "kios_utils/kios_utils.hpp"

#include "mirmi_utils/math.hpp"

namespace Insertion
{

    /**
     * @brief a meta node class for insertion behavior tree node
     *
     */
    class MetaNode : public BT::StatefulActionNode
    {
    public:
        MetaNode(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
            : BT::StatefulActionNode(name, config),
              tree_state_ptr_(tree_state_ptr),
              task_state_ptr_(task_state_ptr),
              node_context_()
        {
        }
        static BT::PortsList providedPorts();

        // shared context
        std::shared_ptr<kios::TreeState> get_tree_state_ptr();
        std::shared_ptr<kios::TaskState> get_task_state_ptr();

        // local context
        kios::ActionPhaseContext &get_node_context_ref();

        // * update tree state with this node's context
        virtual void update_tree_state() = 0;
        // * node context initializer
        virtual void node_context_initialize() = 0;

    private:
        //* shared objects among the entire tree
        std::shared_ptr<kios::TreeState> tree_state_ptr_;
        std::shared_ptr<kios::TaskState> task_state_ptr_;

        // * default values as member variable of the node
        kios::ActionPhaseContext node_context_; // default node context value

        virtual bool is_success(); // here to set the check condition
        bool is_switch_action();
    };

    /**
     * @brief A hypermeta node that provides common member variables and member functions that are necessary in kios application.
     * Can be used to declare any kind of node from behaviortree.CPP lib.
     *
     */
    template <class T>
    class HyperMetaNode : public T
    {
    public:
        HyperMetaNode(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
            : T(name, config),
              tree_state_ptr_(tree_state_ptr),
              task_state_ptr_(task_state_ptr),
              node_context_(),
              hasSucceededOnce(false),
              node_archive_()
        {
        }

        // /**
        //  * @brief
        //  *
        //  */
        // void initialize_archive()
        // {
        //     // * assign action phase
        //     action_phase_ = get_node_context_ref().action_phase;
        //     // * read archive from input port
        //     BT::Expected<int> action_group = getInput<int>("action_group");
        //     BT::Expected<int> action_id = getInput<int>("action_id");
        //     BT::Expected<std::string> description = getInput<std::string>("description");

        //     if (!action_group)
        //     {
        //         // ! now only 0 group for test.
        //         // throw BT::RuntimeError("missing required input [action_group]: ", action_group.error());
        //         action_group_ = 0;
        //     }
        //     else
        //     {
        //         action_group_ = action_group.value();
        //     }

        //     if (!action_id)
        //     {
        //         throw BT::RuntimeError("missing required input [action_id]: ", action_id.error());
        //     }
        //     else
        //     {
        //         action_id = action_id.value();
        //     }
        //     if (!description)
        //     {
        //         throw BT::RuntimeError("missing required input [description]: ", description.error());
        //     }
        //     else
        //     {
        //         description_ = description.value();
        //     }
        // }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("isOnceSucceeded"),
                BT::InputPort<int>("action_group"),
                BT::InputPort<int>("action_id"),
                BT::InputPort<std::string>("description")};
        }

        // shared context
        std::shared_ptr<kios::TreeState> get_tree_state_ptr()
        {
            return tree_state_ptr_;
        }

        std::shared_ptr<kios::TaskState> get_task_state_ptr()
        {
            return task_state_ptr_;
        }

        // local context
        kios::ActionPhaseContext &get_node_context_ref()
        {
            return node_context_;
        }

        void mark_success()
        {
            hasSucceededOnce = true;
        }

        bool has_succeeded_once()
        {
            return hasSucceededOnce;
        }

        // ! MUST OVERRIDE
        /////////////////////////////////////////////////////////////////
        // * update tree state with this node's context
        virtual void update_tree_state() = 0;
        // * node context initializer
        virtual void node_context_initialize() = 0;
        virtual bool is_success() = 0;
        /////////////////////////////////////////////////////////////////
        /**
         * @brief if mios have returned SUCCESS (in task state), consume it and return true. else return false.
         * Used by actions that detect success condition on mios's side.
         *
         * @return true
         * @return false
         */
        bool consume_mios_success()
        {
            if (task_state_ptr_->isActionSuccess)
            {
                mark_success();
                // * mios succeeded. consume it and return true
                task_state_ptr_->isActionSuccess = false;
                return true;
            }
            else
            {
                // * mios hasn't succeed. do nothing and return false
                return false;
            }
        }

        // std::tuple<int, int, std::string, kios::ActionPhase> get_archive()
        // { // ! TODO
        //     return std::make_tuple<int, int, std::string, kios::ActionPhase>(
        //         action_group_,
        //         action_id_,
        //         description_,
        //         action_phase_);
        // }

        // virtual bool is_switch_action();

        // ! CANNOT OVERRIDE THIS: FINAL OVERRIDE IN STATEFULACTION!
        // BT::NodeStatus tick() override;

        kios::NodeArchive &get_archive_ref()
        {
            return node_archive_;
        }

    private:
        kios::NodeArchive node_archive_;

        //* only run once flag
        bool hasSucceededOnce; // ! this will be DISCARDED after the integration of RunOnceNode

        //* shared objects among the entire tree
        std::shared_ptr<kios::TreeState> tree_state_ptr_;
        std::shared_ptr<kios::TaskState> task_state_ptr_;

        // * default values as member variable of the node
        kios::ActionPhaseContext node_context_; // default node context value
    };

    class KiosActionNode : public HyperMetaNode<BT::StatefulActionNode>
    {
    public:
        KiosActionNode(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
            : HyperMetaNode<BT::StatefulActionNode>(name, config, tree_state_ptr, task_state_ptr)
        {
        }

        /**
         * @brief initialize the action node archive for context manager. can only be called after the action node is registered in bt_factory.
         *
         */
        void initialize_archive()
        {
            auto &archive = get_archive_ref();
            // * read archive from input port
            archive.action_phase = get_node_context_ref().action_phase;
            BT::Expected<int> action_group = getInput<int>("action_group");
            BT::Expected<int> action_id = getInput<int>("action_id");
            BT::Expected<std::string> description = getInput<std::string>("description");

            if (!action_group)
            {
                // ! now only 0 group for test.
                // throw BT::RuntimeError("missing required input [action_group]: ", action_group.error());
                archive.action_group = 0;
            }
            else
            {
                archive.action_group = action_group.value();
            }

            if (!action_id)
            {
                throw BT::RuntimeError("missing required input [action_id]: ", action_id.error());
            }
            else
            {
                archive.action_id = action_id.value();
            }
            if (!description)
            {
                throw BT::RuntimeError("missing required input [description]: ", description.error());
            }
            else
            {
                archive.description = description.value();
            }
        }

    private:
    };

    class KiosConditionNode : public HyperMetaNode<BT::ConditionNode>
    {
    public:
        KiosConditionNode(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
            : HyperMetaNode<BT::ConditionNode>(name, config, tree_state_ptr, task_state_ptr)
        {
        }

        void initialize_archive()
        {
            auto &archive = get_archive_ref();
            // * read archive from input port
            archive.action_phase = kios::ActionPhase::CONDITION;
            BT::Expected<int> action_group = getInput<int>("action_group");
            BT::Expected<int> action_id = getInput<int>("action_id");
            BT::Expected<std::string> description = getInput<std::string>("description");

            if (!action_group)
            {
                // ! now only 0 group for test.
                // throw BT::RuntimeError("missing required input [action_group]: ", action_group.error());
                archive.action_group = 0;
            }
            else
            {
                archive.action_group = action_group.value();
            }

            if (!action_id)
            {
                archive.action_id = 0; // condition node default id
            }
            else
            {
                archive.action_id = action_id.value();
            }
            if (!description)
            {
                archive.description = "a condition node.";
            }
            else
            {
                archive.description = description.value();
            }
        }

    private:
    };
    // /////////////////////////////////////////////////////////////////////
    // /////////////////////////////////////////////////////////////////////
    // // * META CONDITION NODE
    // class HasObject : public HyperMetaNode<BT::ConditionNode>
    // {
    // public:
    //     HasObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);
    //     BT::NodeStatus tick() override;
    //     bool is_success() override;

    //     // empty override
    //     void update_tree_state() override{};
    //     void node_context_initialize() override{};

    // private:
    // };

    // class AtPosition : public HyperMetaNode<BT::ConditionNode>
    // {
    // public:
    //     AtPosition(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr);
    //     BT::NodeStatus tick() override;
    //     bool is_success() override;

    //     // empty override
    //     void update_tree_state() override{};
    //     void node_context_initialize() override{};

    // private:
    // };

} // namespace Insertion