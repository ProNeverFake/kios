#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <any>
#include <unordered_map>
#include <string>

#include "spdlog/spdlog.h"

#include "kios_utils/kios_utils.hpp"

#include "mirmi_utils/math.hpp"

namespace BT
{
    /**
     * @brief object grounding method in BT
     *
     * @tparam
     * @param str
     * @return std::vector<std::string>
     */
    template <>
    inline std::vector<std::string> convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');
        // if (parts.size() != 2) // ! TODO
        // {
        //     throw RuntimeError("invalid input)");
        // }
        // else
        // {
        //     std::vector<std::string> output;

        //     for (auto &item : parts)
        //     {
        //         output.push_back(convertFromString<std::string>(item));
        //     }

        //     return output;
        // }
        std::vector<std::string> output;

        for (auto &item : parts)
        {
            output.push_back(convertFromString<std::string>(item));
        }

        return output;
    }
} // namespace BT

namespace Insertion
{

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
              node_archive_(),
              object_keys_(),
              object_names_()
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("isOnceSucceeded")};
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

        // ! MAY OVERRIDE
        /////////////////////////////////////////////////////////////////
        virtual void on_success()
        {
            tree_state_ptr_->isSucceeded = true;
            // do nothing.
        }
        /////////////////////////////////////////////////////////////////

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
                spdlog::info("From mios: {} succeeded.", node_context_.action_name);
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

        // ! CANNOT OVERRIDE THIS: FINAL OVERRIDE IN STATEFULACTION!
        // BT::NodeStatus tick() override;

        kios::NodeArchive &get_archive_ref()
        {
            return node_archive_;
        }

        std::vector<std::string> &get_object_names_ref()
        {
            return object_names_;
        }

        std::vector<std::string> &get_obejct_keys_ref()
        {
            return object_keys_;
        }

        void test_objects()
        {
            spdlog::error("objects test: ");
            for (auto &item : object_names_)
            {
                spdlog::error(item);
            }
        }

    private:
        kios::NodeArchive node_archive_;
        std::vector<std::string> object_keys_;
        std::vector<std::string> object_names_;

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

        static BT::PortsList providedPorts() // ! CHANGE
        {
            return {
                BT::InputPort<bool>("isOnceSucceeded"),
                BT::InputPort<int>("action_group"),
                BT::InputPort<int>("action_id"),
                BT::InputPort<std::string>("description"),
                BT::InputPort<std::vector<std::string>>("objects")}; // ! ADD
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
            BT::Expected<std::vector<std::string>> objects = getInput<std::vector<std::string>>("objects"); // ! ADD

            int ag = 0; // default/current action group

            if (!action_group)
            {
                // ! now only 0 group for test.
                // throw BT::RuntimeError("missing required input [action_group]: ", action_group.error());
                archive.action_group = ag;
            }
            else
            {
                archive.action_group = action_group.value();
                ag = action_group.value();
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
                // throw BT::RuntimeError("missing required input [description]: ", description.error());
                spdlog::warn("Action node " + std::to_string(ag) + "-" + std::to_string(action_id.value()) + " has no description.");
                // here pass. use the default value of the struct.
            }
            else
            {
                archive.description = description.value();
            }
            if (!objects)
            {
                spdlog::warn("Action node " + std::to_string(ag) + "-" + std::to_string(action_id.value()) + " grounds no objects.");
            }
            else
            {
                auto &objs = get_object_names_ref();
                objs = objects.value();
            }

            test_objects();
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
    // * a possible condition node imp.
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