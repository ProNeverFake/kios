#include "behavior_tree/tree_root.hpp"

namespace Insertion
{
    TreeRoot::TreeRoot(std::shared_ptr<kios::TreeState> tree_state_ptr, std::shared_ptr<kios::TaskState> task_state_ptr)
        : tree_state_ptr_(tree_state_ptr),
          task_state_ptr_(task_state_ptr)
    {
        // * set spdlog
        std::string verbosity = "trace";
        spdlog::level::level_enum info_level;
        if (verbosity == "trace")
        {
            info_level = spdlog::level::trace;
        }
        else if (verbosity == "debug")
        {
            info_level = spdlog::level::debug;
        }
        else if (verbosity == "info")
        {
            info_level = spdlog::level::info;
        }
        else
        {
            info_level = spdlog::level::info;
        }

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(info_level);
        console_sink->set_pattern("[kios][tree_root][%^%l%$] %v");

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/kios_tree_root.txt", true);
        file_sink->set_level(spdlog::level::debug);

        auto logger = std::shared_ptr<spdlog::logger>(new spdlog::logger("kios", {console_sink, file_sink}));
        logger->set_level(info_level);
        spdlog::set_default_logger(logger);
        spdlog::info("spdlog: initialized.");

        // * run tree initialization method
        initialize_tree();
    }

    void TreeRoot::initialize_tree()
    {
        // * register nodes
        // * condition nodes
        factory_.registerNodeType<HasObject>("HasObjectApproch", "approach", tree_state_ptr_, task_state_ptr_);
        factory_.registerNodeType<HasObject>("HasObjectContact", "contact", tree_state_ptr_, task_state_ptr_);
        factory_.registerNodeType<AtPosition>("AtPositionApproch", "approach", tree_state_ptr_, task_state_ptr_);
        factory_.registerNodeType<AtPosition>("AtPositionContact", "contact", tree_state_ptr_, task_state_ptr_);
        // * demo action nodes
        factory_.registerNodeType<Approach>("Approach", tree_state_ptr_, task_state_ptr_);
        factory_.registerNodeType<Contact>("Contact", tree_state_ptr_, task_state_ptr_);
        factory_.registerNodeType<Wiggle>("Wiggle", tree_state_ptr_, task_state_ptr_);

        // * general action nodes
        factory_.registerNodeType<CartesianMove>("CartesianMove", tree_state_ptr_, task_state_ptr_);
        factory_.registerNodeType<JointMove>("JointMove", tree_state_ptr_, task_state_ptr_);

        // * generate tree
        tree_ = factory_.createTreeFromText(test_tree);
    }

    // ! DISCARDED
    // ! HERE TRY TO GENERATE TREE FROM C++ CODE BUT NOW XML GENERATOR IS PREDERED
    // void TreeRoot::construct_tree()
    // {
    //     BT::NodeConfiguration config;
    //     // Create nodes
    //     auto root_node = std::make_shared<BT::SequenceNode>("root");
    //     auto reactive_seq = std::make_shared<BT::ReactiveSequence>("reactive_seq");
    //     auto approach_node = std::make_shared<Approach>("approach", config, tree_state_ptr_, task_state_ptr_);
    //     auto contact_node = std::make_shared<Contact>("contact", config, tree_state_ptr_, task_state_ptr_);

    //     reactive_seq->EnableException(false);
    //     // Set parent-child relationships
    //     root_node->addChild(reactive_seq.get());
    //     reactive_seq->addChild(approach_node.get());
    //     reactive_seq->addChild(contact_node.get());

    //     // Store nodes in a container to manage their lifetime
    //     std::vector<std::shared_ptr<BT::TreeNode>> nodes;
    //     nodes.push_back(std::move(root_node));
    //     nodes.push_back(std::move(reactive_seq));
    //     nodes.push_back(std::move(approach_node));
    //     nodes.push_back(std::move(contact_node));

    //     // Create a tree from the root node
    //     BT::Tree tree(root_node.get());

    //     // Tick the tree as needed
    //     while (some_condition)
    //     {
    //         tree.tickRoot();
    //         // Add your logic here
    //     }
    // }

    std::shared_ptr<kios::TreeState> TreeRoot::get_tree_state_ptr()
    {
        return tree_state_ptr_;
    }
    std::shared_ptr<kios::TaskState> TreeRoot::get_task_state_ptr()
    {
        return task_state_ptr_;
    }

    /**
     * @brief only tick once, return running immediately if a node is running
     *
     * @return BT::NodeStatus
     */
    BT::NodeStatus TreeRoot::tick_once()
    {
        return tree_.tickOnce();
    }

    /**
     * @brief tick the tree, block until it return success
     *
     * @return BT::NodeStatus
     */
    BT::NodeStatus TreeRoot::tick_while_running()
    {
        return tree_.tickWhileRunning();
    }

} // namespace Insertion

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////