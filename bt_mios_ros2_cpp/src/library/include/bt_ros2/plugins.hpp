#pragma once

#include <filesystem>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "bt_ros2/ros_node_params.hpp"

// Use this macro to generate a plugin for:
//
// - BT::RosActionNode
// - BT::RosServiceNode
// - BT::RosServiceNode
// - BT::RosTopicPubNode
// - BT::RosTopicSubNode
//
// - First argument: type to register (name of the class)
// - Second argument: string with the registration name
//
// Usage example:
//   CreateRosNodePlugin(MyClassName, "MyClassName");

#define CreateRosNodePlugin(TYPE, REGISTRATION_NAME)                                                \
    BTCPP_EXPORT void                                                                               \
    BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory &factory, const BT::RosNodeParams &params) \
    {                                                                                               \
        factory.registerNodeType<TYPE>(REGISTRATION_NAME, params);                                  \
    }

/**
 * @brief RegisterRosNode function used to load a plugin and register
 * the containing Node definition.
 *
 * @param factory   the factory where the node should be registered.
 * @param filepath  path to the plugin.
 * @param params    parameters to pass to the instances of the Node.
 */
inline void RegisterRosNode(BT::BehaviorTreeFactory &factory, const std::filesystem::path &filepath, const BT::RosNodeParams &params)
{
    BT::SharedLibrary loader(filepath);
    typedef void (*Func)(BT::BehaviorTreeFactory &, const BT::RosNodeParams &);
    auto func = (Func)loader.getSymbol("BT_RegisterRosNodeFromPlugin");
    func(factory, params);
}
